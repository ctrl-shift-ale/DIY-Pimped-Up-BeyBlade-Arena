/*
 * =============================================================================
 *  PIMPED-UP BEYBLADE ARENA BY ALESSANDRO QUARANTA
 * =============================================================================
 *  Hardware
 *  --------
 *  Input  : Piezo microphone on A1
 *  Output : 60-LED WS2812 strip on pin 10
 *           5 laser modules on pins 5, 6, 12, 7, 8
 *
 *  Behaviour overview
 *  -------------------
 *  LEDS
 *    - Quiet (no sound / release expired) → LEDs off
 *    - Soft hit  (0 < loudness < 1.0)         → flash animation, brightness ∝ loudness;
 *                                           quieter secondary hits during the
 *                                           release window add brightness instead
 *                                           of restarting the animation
 *    - Hard hit  (loudness == 1.0)            → collision animation (runs to
 *                                           completion, ignores new sound)
 *    - After collision                    → back to default (flash / off)
 *
 *  LASERS
 *    - intensity < intensityThreshold     → all lasers ON (steady)
 *    - intensity >= intensityThreshold    → all lasers blink in unison;
 *                                           blink rate ∝ intensity
 *
 *  INTENSITY
 *    - Each detected NEW_SOUND adds loudness to intensity (capped at intensityCap)
 *    - Intensity decays exponentially toward 0 over time
 *
 *  STANDBY (idle)
 *    - If no sound detected for IDLE_TIMEOUT ms → enter standby state
 *    - LEDs cycle colours; lasers sweep one at a time
 *    - Any new sound → immediately exit standby, return to default
 * =============================================================================
 */

#include <FastLED.h>
#include <string.h>
#include <math.h>

// ============================================================================
// PIN DEFINITIONS
// ============================================================================
#define LED_PIN      10
#define LASER_PIN_A   5
#define LASER_PIN_B   6
#define LASER_PIN_C  12
#define LASER_PIN_D   7
#define LASER_PIN_E   8
#define ONBOARD_LED  13      // TEST_PIEZO mirrors gate state here

#define PIEZO_PIN    A1

#define NUM_LEDS     60
#define NUM_LASERS    5

// ============================================================================
// SYSTEM MODE
// ============================================================================
#define OPERATIVE                    -1
#define TEST_PIEZO                    1  // print raw piezo readings to Serial
#define TEST_LED_BASIC                2  // cycle LED colours
#define TEST_LED_FLASH                3  // flash at increasing fake loudness (0.2→1.0)
#define TEST_LED_COLLISION_ANIMATION  4  // run the collision animation at ~60 fps
#define TEST_LASERS                   5  // sweep lasers one at a time

int SYSTEM_MODE = OPERATIVE;

// ============================================================================
// DEBUG
// ============================================================================
#define DEBUG false

// ============================================================================
// SOUND STATE
// ============================================================================
/*
 *  After a peak, a release window of LED_FLASH_RELEASE_TIME ms stays open.
 *  The decayed reference level declines linearly to 0 over that window.
 *
 *  NO_SOUND        - quiet, no open release window
 *  NEW_SOUND       - louder than the current decayed reference → new peak
 *  SECONDARY_SOUND - audible but quieter than reference → add brightness
 *  REL_SOUND       - silent but release window still ticking
 */
#define NO_SOUND        0
#define NEW_SOUND       1
#define SECONDARY_SOUND 2
#define REL_SOUND       3

int SOUND_STATE = NO_SOUND;

// ============================================================================
// LED STATE
// ============================================================================
/*
 *  LED_FLASH_NEW      - taskAudio armed a flash; taskLED latches params this tick
 *  LED_FLASH_ONGOING  - flash counting down
 *  LED_COLLISION_NEW  - taskAudio triggered a collision; taskLED inits anim this tick
 *  LED_COLLISION_ONGOING - collision animation running
 */
#define LED_STANDBY             0
#define LED_OFF                 1
#define LED_FLASH_NEW           2
#define LED_FLASH_ONGOING       3
#define LED_COLLISION_NEW       4
#define LED_COLLISION_ONGOING   5

int LED_STATE = LED_OFF;

// ============================================================================
// LASER STATE
// ============================================================================
#define LASERS_STANDBY      0
#define LASERS_ALL_ON       1
#define LASERS_FLASHING_ALL 2

int LASERS_STATE     = LASERS_ALL_ON;
int LASERS_STATE_HIS = LASERS_ALL_ON;

// ============================================================================
// AUDIO / SIGNAL PARAMETERS
// ============================================================================
#define NOISE_GATE    80        // raw ADC floor — below this = silence
#define SIGNAL_MAX    0.8f      // ADC normalised fraction treated as full scale
#define SAMPLE_WINDOW 200       // peak-hold window in ms

// ============================================================================
// LED PARAMETERS
// ============================================================================
const float FLASH_MAX_BRIGHTNESS     = 200.0f;  // 0–255
const int   LED_FLASH_RELEASE_TIME   = 500;     // flash fade-out duration (ms)
const int   LED_COLLISION_ANIMATION_DUR = 2500; // target collision animation duration (ms)

// ============================================================================
// LASER PARAMETERS
// ============================================================================
const float LASERS_BLINK_CPS_MAX = 20.0f;  // max blink cycles-per-second

// ============================================================================
// TIMING  (all in ms)
// ============================================================================
#define RATE_AUDIO       25
#define RATE_LED         50
#define RATE_LASER       25   // matches RATE_AUDIO; required for accurate 20 Hz blink
#define RATE_INTENSITY  200

#define RATE_HALFDECAY_INTENSITY 3000   // time for intensity to halve
#define IDLE_TIMEOUT            10000   // silence duration before standby

// ============================================================================
// SCALING EXPONENTS
// ============================================================================
const float PIEZO_EXPONENT = 2.0f;   // raw ADC → loudness curve
const float DECAY_EXPONENT = 2.0f;   // flash brightness decay envelope curve

// ============================================================================
// HARDWARE
// ============================================================================
CRGB leds[NUM_LEDS];
const int lasers[NUM_LASERS] = { LASER_PIN_A, LASER_PIN_B,
                                 LASER_PIN_C, LASER_PIN_D, LASER_PIN_E };
int lasersState[NUM_LASERS]  = { LOW, LOW, LOW, LOW, LOW };

// ============================================================================
// SHARED AUDIO STATE  (written by taskAudio, read by other tasks)
// ============================================================================
float loudness          = 0.0f;  // loudness of the most recent window  [0, 1]
float loudnessExtraPeak = 0.0f;  // extra brightness queued for ongoing flash

// ============================================================================
// PEAK RELEASE TRACKING  (private to taskAudio — file-scope statics)
// ============================================================================
/*
 *  lastPeakLoudness : loudness of the most recent peak
 *  lastPeakTime     : millis() when that peak was detected
 *  peakHoldTimer    : counts up to SAMPLE_WINDOW; peak is held until then
 *  windowPeak       : highest raw reading seen so far this window
 *
 *  We perform peak-hold inside taskAudio (rather than readMicLevel) so
 *  readMicLevel can stay a simple instantaneous scaled read.
 */
static float         lastPeakLoudness = 0.0f;
static unsigned long lastPeakTime     = 0;
static int           peakHoldTimer    = 0;
static float         windowPeak       = 0.0f;

// ============================================================================
// INTENSITY
// ============================================================================
float intensity       = 0.0f;
float targetIntensity = 0.0f;

const float intensityThreshold = 1.0f;  // below this → lasers steady on
const float intensityCap       = 3.0f;  // intensity is clamped here

const int INTENSITY_TURNS_FOR_DECAY =
    RATE_HALFDECAY_INTENSITY / RATE_INTENSITY;

float intensityReductionPerTurn = 0.0f;
bool  intensityRefreshed        = false;

// ============================================================================
// STANDBY (idle) TRACKING
// ============================================================================
unsigned long lastSoundTime = 0;
bool          systemIdle    = false;

// ============================================================================
// TASK SCHEDULER
// ============================================================================
struct Task {
    void (*callback)();
    unsigned long interval;
    unsigned long lastRun;
    bool enabled;

    void run(unsigned long now) {
        if (!enabled) return;
        if (now - lastRun >= interval) {
            lastRun = now;
            callback();
        }
    }
};

void taskAudio();
void taskLED();
void taskLaser();
void taskIntensity();
void taskTestPiezo();
void taskTestLEDBasic();
void taskTestLEDFlash();
void taskTestCollisionAnimation();
void taskTestLasers();

Task tasks[] = {
    { taskAudio,     RATE_AUDIO,     0, true },
    { taskLED,       RATE_LED,       0, true },
    { taskLaser,     RATE_LASER,     0, true },
    { taskIntensity, RATE_INTENSITY, 0, true },
};
const int NUM_TASKS = sizeof(tasks) / sizeof(tasks[0]);

/*
 *  Test task table — indices must match TEST_* constants above:
 *    0 TEST_PIEZO
 *    1 TEST_LED_BASIC
 *    2 TEST_LED_FLASH
 *    3 TEST_LED_COLLISION_ANIMATION
 *    4 TEST_LASERS
 */
Task testTasks[] = {
    { taskTestPiezo,              50, 0, false },
    { taskTestLEDBasic,         1000, 0, false },
    { taskTestLEDFlash,      RATE_LED, 0, false },   // LED render rate; arming is handled internally
    { taskTestCollisionAnimation, 16, 0, false },   // ~60 fps
    { taskTestLasers,            100, 0, false },
};
const int NUM_TEST_TASKS = sizeof(testTasks) / sizeof(testTasks[0]);

unsigned long now = 0;  // updated once per loop() tick

// ============================================================================
// UTILITY
// ============================================================================

/*
 *  Maps val from [minIn, maxIn] to [minOut, maxOut] on a power curve.
 *  exponent > 1 → slow start / fast finish
 *  exponent < 1 → fast start / slow finish
 */
float expScale(float val, float minIn, float maxIn,
               float minOut, float maxOut, float exponent) {
    val = constrain(val, minIn, maxIn);
    float n = (val - minIn) / (maxIn - minIn);
    return minOut + pow(n, exponent) * (maxOut - minOut);
}

/*
 *  Wraps val into the closed range [minVal, maxVal] using modular arithmetic.
 */
int wrap(int val, int minVal, int maxVal) {
    int range = maxVal - minVal + 1;
    return ((val - minVal) % range + range) % range + minVal;
}

// ============================================================================
// SIGNAL PROCESSING
// ============================================================================

/*
 *  Applies noise gate + exponential scaling to a raw ADC reading.
 *  Returns a normalised value in [0, 1].
 */
float scalePiezoInput(int rawValue) {
    if (rawValue <= NOISE_GATE) return 0.0f;
    float normalized = (float)(rawValue - NOISE_GATE) / (1023.0f - NOISE_GATE);
    return constrain(
        expScale(normalized, 0.0f, SIGNAL_MAX, 0.0f, 1.0f, PIEZO_EXPONENT),
        0.0f, 1.0f);
}

/*
 *  Returns an instantaneous scaled ADC reading.
 *  Peak-hold windowing is handled inside taskAudio.
 */
float readMicLevel() {
    return scalePiezoInput(analogRead(PIEZO_PIN));
}

// ============================================================================
// LASER HELPERS
// ============================================================================

void setLaser(int idx, int state) {
    digitalWrite(lasers[idx], state);
    lasersState[idx] = state;
}
void allLasersOff() { for (int i = 0; i < NUM_LASERS; i++) setLaser(i, LOW);  }
void allLasersOn()  { for (int i = 0; i < NUM_LASERS; i++) setLaser(i, HIGH); }

/*
 *  Standby laser animation: sweeps one laser on at a time at ~1 Hz.
 */
void laserStandbyAnimation() {
    static unsigned long lastChange = 0;
    static int laserIdx = 0;

    if (now - lastChange >= 500) {
        lastChange = now;
        setLaser(wrap(laserIdx - 1, 0, NUM_LASERS - 1), LOW);
        setLaser(laserIdx, HIGH);
        if (DEBUG) { Serial.print("Standby laser: "); Serial.println(laserIdx); }
        laserIdx = wrap(laserIdx + 1, 0, NUM_LASERS - 1);
    }
}

// ============================================================================
// LED ANIMATION HELPERS
// ============================================================================

/*
 *  Sets every LED to the same HSV colour and pushes to the strip.
 */
void ledFlashAnimation(int hue, int val) {
    for (int i = 0; i < NUM_LEDS; i++) leds[i] = CHSV(hue, 255, val);
    FastLED.show();
}

/*
 *  Two-dot approach + explosion animation.
 *  Must be called every RATE_LED ms; maintains internal state across calls.
 *  Returns true only when the complete sequence has finished.
 *
 *  Duration scaling
 *  ----------------
 *  The natural animation duration (measured by simulation) is ~960 ms.
 *  A time-scale factor = LED_COLLISION_ANIMATION_DUR / 960 is applied to
 *  every speed/timing constant at init time so the full sequence fits exactly
 *  into the requested duration.
 */
bool ledCollisionAnimation() {
    // Natural reference duration the speed constants were tuned for (ms)
    static const float NATURAL_DURATION_MS = 960.0f;

    static int colour_1 = 0, colour_2 = 0;
    static int brightness = 20;
    static int dotPos = 0, dotSize = 1;
    static int state = 3;   // 1=moving  2=explosion  3=finished/needs-init
    static unsigned long lastUpdate = 0;

    // These are set at init time with the duration scale applied
    static int scaledStartSpeed    = 100;
    static int scaledEndSpeed      = 2;
    static int scaledXplStartSpeed = 7;

    static const float expAcc = 0.5f;
    static int  currentSpeed = 100;
    static int  timer = 0, xplStep = 0;

    static const int colourWhite[3]  = { 45,   0, 255 };
    static const int colourYellow[3] = { 45, 255, 255 };
    static const int colourRed[3]    = {  0, 255, 255 };
    static int currentColourCore[3]   = { 45,   0, 255 };
    static int currentColourCorona[3] = { 45, 255, 255 };

    static int  coronaPosLeft = 0, coronaPosRight = 0;
    static int  coronaLeftDots[27]  = { 0 };
    static int  coronaRightDots[27] = { 0 };
    static bool doneCore = false, doneCorona = false;

    const int coronaOffsetInit = 3;

    // ---- STATE 3: Initialise / reset for a new run ----------------------
    if (state == 3) {
        // Compute scale factor so the full animation fills the requested duration
        float scale = (float)LED_COLLISION_ANIMATION_DUR / NATURAL_DURATION_MS;
        scaledStartSpeed    = max(1, (int)(100.0f * scale));
        scaledEndSpeed      = max(1, (int)(  2.0f * scale));
        scaledXplStartSpeed = max(1, (int)(  7.0f * scale));

        fill_solid(leds, NUM_LEDS, CRGB::Black);
        int rndPol = random(2) == 0 ? -1 : 1;
        dotPos = 0;  dotSize = 1;  brightness = 20;
        doneCore = false;  doneCorona = false;
        colour_1 = random(256);
        colour_2 = wrap(colour_1 + 128 + random(64) * rndPol, 0, 255);
        leds[0]            = CHSV(colour_1, 255, brightness);
        leds[NUM_LEDS - 1] = CHSV(colour_2, 255, brightness);
        currentSpeed = scaledStartSpeed;
        timer = 0;  xplStep = 0;
        state = 1;
        lastUpdate = now;
        FastLED.show();
        return false;
    }

    int deltaTime = (int)(now - lastUpdate);
    timer += deltaTime;

    // ---- STATE 1: Dots moving toward centre -----------------------------
    if (state == 1) {
        float movement = (float)timer / (float)currentSpeed;
        if (movement >= 0.75f) {
            // Erase previous positions
            for (int c = 0; c < dotSize; c++) {
                leds[dotPos - c]                = CRGB::Black;
                leds[NUM_LEDS - dotPos - 1 + c] = CRGB::Black;
            }
            dotSize = constrain((int)movement, 1, 5);
            dotPos += (int)round(movement + 0.5f);

            if (dotPos >= (NUM_LEDS / 2) - 1) {
                // Reached centre → begin explosion
                state = 2;
                coronaPosLeft  = (NUM_LEDS / 2) - coronaOffsetInit - 1;
                coronaPosRight = (NUM_LEDS / 2) + coronaOffsetInit;
                xplStep = 0;
                currentSpeed = scaledXplStartSpeed;
            } else {
                // Brighten and advance
                brightness = (int)expScale((float)dotPos, 0.0f, 29.0f,
                                           20.0f, 300.0f, 4.0f);
                int bt = brightness;
                for (int c = 0; c < dotSize; c++) {
                    bt = constrain((int)((float)bt * (1.0f - (float)c / 10.0f)), 0, 255);
                    leds[dotPos - c]                = CHSV(colour_1, 255, bt);
                    leds[NUM_LEDS - dotPos - 1 + c] = CHSV(colour_2, 255, bt);
                }
                currentSpeed = (int)expScale((float)dotPos, 0.0f, 29.0f,
                                             (float)scaledStartSpeed,
                                             (float)scaledEndSpeed, expAcc);
                timer = 0;
            }
        }
    }

    // ---- STATE 2: Explosion ---------------------------------------------
    if (state == 2) {
        const int cL = (NUM_LEDS / 2) - 1;
        const int cR = (NUM_LEDS / 2);
        memcpy(currentColourCore,   colourWhite,  sizeof(colourWhite));
        memcpy(currentColourCorona, colourYellow, sizeof(colourYellow));

        if (xplStep < 4) {
            currentColourCore[2] = map(xplStep, 0, 3, brightness, 255);
            for (int o = 0; o < 3; o++) {
                leds[cL - o] = CHSV(currentColourCore[0],
                                    currentColourCore[1], currentColourCore[2]);
                leds[cR + o] = leds[cL - o];
            }
        } else if (xplStep < 10) {
            for (int o = 1; o < 3; o++) {
                currentColourCorona[1] = map(xplStep, 4, 9,
                                             colourWhite[1], colourYellow[1]);
                leds[cL - o] = CHSV(currentColourCorona[0],
                                    currentColourCorona[1], currentColourCorona[2]);
                leds[cR + o] = leds[cL - o];
            }
        } else if (xplStep < 16) {
            leds[cL] = CHSV(currentColourCore[0], currentColourCore[1],
                            map(xplStep, 10, 15, 255, 0));
            leds[cR] = leds[cL];
            for (int o = 1; o < 3; o++) {
                currentColourCorona[0] = map(xplStep, 10, 15,
                                             colourYellow[0], colourRed[0]);
                currentColourCorona[2] = map(xplStep, 10, 15, 255, 0);
                leds[cL - o] = CHSV(currentColourCorona[0],
                                    currentColourCorona[1], currentColourCorona[2]);
                leds[cR + o] = leds[cL - o];
            }
        } else {
            doneCore = true;
        }

        if (xplStep > 0) {
            if (xplStep < 8) {
                for (int idx = (xplStep - 1) * 4;
                         idx < (xplStep - 1) * 4 + 4 && idx < 27; idx++) {
                    coronaLeftDots[idx]  = 255;
                    coronaRightDots[idx] = 255;
                }
            }
            if (xplStep > 1) {
                for (int idx = 0; idx < (xplStep - 1) * 4 && idx < 27; idx++) {
                    if (coronaLeftDots[idx]  > 0) coronaLeftDots[idx]  -= 256 / 16;
                    if (coronaRightDots[idx] > 0) coronaRightDots[idx] -= 256 / 16;
                }
            }
            if (coronaLeftDots[26] <= 0 && xplStep > 8) doneCorona = true;

            for (int idx = 0; idx < 27; idx++) {
                leds[26 - idx] = CHSV(0, 255, constrain(coronaLeftDots[idx],  0, 255));
                leds[33 + idx] = CHSV(0, 255, constrain(coronaRightDots[idx], 0, 255));
            }
            if (doneCore && doneCorona) state = 3;
        }
        xplStep++;
    }

    lastUpdate = now;
    FastLED.show();
    return (state == 3);
}

/*
 *  Standby LED animation: cycles through colours at 1 Hz.
 */
void ledStandbyAnimation() {
    static unsigned long lastChange = 0;
    static int step = 0;

    if (now - lastChange >= 1000) {
        lastChange = now;
        FastLED.clear();
        switch (step) {
            case 0: fill_solid(leds, NUM_LEDS, CRGB::White);  break;
            case 1: fill_solid(leds, NUM_LEDS, CRGB::Red);    break;
            case 2: fill_solid(leds, NUM_LEDS, CRGB::Green);  break;
            case 3: fill_solid(leds, NUM_LEDS, CRGB::Blue);   break;
            case 4: /* off */                                  break;
        }
        FastLED.setBrightness(128);
        FastLED.show();
        FastLED.setBrightness(255);   // restore global brightness for normal ops
        step = (step + 1) % 5;
    }
}

// ============================================================================
// INTENSITY HELPER
// ============================================================================

/*
 *  Adds val to intensity (capped at intensityCap), recalculates the half-decay
 *  target and per-tick reduction, and flags taskIntensity for a counter reset.
 */
void refreshIntensity(float val) {
    intensity += val;
    if (intensity > intensityCap) intensity = intensityCap;
    targetIntensity = intensity / 2.0f;
    intensityReductionPerTurn =
        (intensity - targetIntensity) / (float)INTENSITY_TURNS_FOR_DECAY;
    intensityRefreshed = true;
}

// ============================================================================
// TASK CALLBACKS — NORMAL OPERATION
// ============================================================================

/*
 *  taskAudio
 *  ---------
 *  Runs every RATE_AUDIO ms.  Performs peak-hold over SAMPLE_WINDOW ms,
 *  classifies the completed window, and arms LED / laser state changes.
 *
 *  Peak-hold: within each SAMPLE_WINDOW the highest instantaneous reading is
 *  retained; at the end of the window that peak is processed.
 *
 *  Release-window classification:
 *    NEW_SOUND       – this window's peak > decayed reference (new louder hit)
 *    SECONDARY_SOUND – audible but below the decayed reference (softer hit)
 *    REL_SOUND       – silent but release window still open
 *    NO_SOUND        – silent and release window closed
 */
void taskAudio() {
    // --- Peak-hold over SAMPLE_WINDOW ------------------------------------
    float sample = readMicLevel();
    if (sample > windowPeak) windowPeak = sample;

    peakHoldTimer += RATE_AUDIO;
    if (peakHoldTimer < SAMPLE_WINDOW) return;   // window not yet complete

    // Window complete — commit the peak and reset for the next window
    loudness = windowPeak;
    windowPeak    = 0.0f;
    peakHoldTimer = 0;

    // --- Compute linearly-decayed reference level ------------------------
    float decayedPeak = 0.0f;
    if (lastPeakLoudness > 0.0f) {
        unsigned long elapsed = now - lastPeakTime;
        if (elapsed < (unsigned long)LED_FLASH_RELEASE_TIME) {
            decayedPeak = lastPeakLoudness *
                (1.0f - (float)elapsed / (float)LED_FLASH_RELEASE_TIME);
        } else {
            lastPeakLoudness = 0.0f;   // release window expired
        }
    }

    // --- Classify this window --------------------------------------------
    if (loudness > decayedPeak && loudness > 0.0f) {
        SOUND_STATE      = NEW_SOUND;
        lastPeakLoudness = loudness;
        lastPeakTime     = now;

    } else if (loudness > 0.0f && decayedPeak > 0.0f) {
        // Audible but softer than the fading reference
        SOUND_STATE = SECONDARY_SOUND;

    } else if (loudness == 0.0f && decayedPeak > 0.0f) {
        SOUND_STATE = REL_SOUND;

    } else {
        SOUND_STATE = NO_SOUND;
    }

    // --- Standby exit ----------------------------------------------------
    if (loudness > 0.0f) {
        lastSoundTime = now;
        if (systemIdle) {
            systemIdle   = false;
            LED_STATE    = LED_OFF;
            LASERS_STATE = LASERS_ALL_ON;
            if (DEBUG) Serial.println("--- Exiting standby ---");
        }
    }

    // --- Intensity accumulation ------------------------------------------
    if (SOUND_STATE == NEW_SOUND) refreshIntensity(loudness);

    // --- LED state selection (collision is never interrupted) ------------
    if (LED_STATE != LED_COLLISION_NEW && LED_STATE != LED_COLLISION_ONGOING) {
        if (SOUND_STATE == NEW_SOUND) {
            LED_STATE = (loudness >= 1.0f) ? LED_COLLISION_NEW : LED_FLASH_NEW;
        } else if (SOUND_STATE == SECONDARY_SOUND) {
            loudnessExtraPeak += loudness * 0.5f;
        }
    }

    if (DEBUG) {
        Serial.print("Loudness: ");      Serial.print(loudness, 3);
        Serial.print(" | Decay ref: ");  Serial.print(decayedPeak, 3);
        Serial.print(" | Intensity: ");  Serial.print(intensity, 3);
        Serial.print(" | Sound: ");      Serial.print(SOUND_STATE);
        Serial.print(" | LED: ");        Serial.println(LED_STATE);
    }
}

/*
 *  taskIntensity
 *  -------------
 *  Runs every RATE_INTENSITY ms.  Smoothly decays intensity toward 0,
 *  halving the target each RATE_HALFDECAY_INTENSITY ms.
 *  Determines whether lasers are steady or blinking.
 */
void taskIntensity() {
    static int counter = 1;

    if (intensityRefreshed) {
        counter = 1;
        intensityRefreshed = false;
    }

    if (counter >= INTENSITY_TURNS_FOR_DECAY) {
        intensity       = targetIntensity;
        targetIntensity /= 2.0f;
        intensityReductionPerTurn =
            (intensity - targetIntensity) / (float)INTENSITY_TURNS_FOR_DECAY;
        counter = 1;
    } else if (intensity > targetIntensity) {
        intensity -= intensityReductionPerTurn;
        if (intensity < 0.05f) intensity = 0.0f;
    }

    if (intensity > 0.0f) counter++;

    LASERS_STATE = (intensity < intensityThreshold) ? LASERS_ALL_ON
                                                    : LASERS_FLASHING_ALL;
}

/*
 *  taskLED
 *  -------
 *  Drives the LED strip every RATE_LED ms.
 *
 *  LED_OFF              → strip dark
 *  LED_FLASH_NEW        → latch flash params, fall through to render first frame
 *  LED_FLASH_ONGOING    → count down, render exponential fade; fold in any
 *                         queued loudnessExtraPeak and clear it each tick
 *  LED_COLLISION_NEW    → initialise collision animation (one tick)
 *  LED_COLLISION_ONGOING→ advance collision animation; revert to LED_OFF when done
 *  LED_STANDBY          → colour-cycle animation
 */
void taskLED() {
    static float flashBrightness = 0.0f;
    static int   flashHue        = 0;
    static int   flashTimer      = 0;

    // ---- Standby timeout ------------------------------------------------
    if (!systemIdle
            && LED_STATE != LED_COLLISION_NEW
            && LED_STATE != LED_COLLISION_ONGOING
            && (now - lastSoundTime >= IDLE_TIMEOUT)) {
        systemIdle = true;
        LED_STATE  = LED_STANDBY;
        if (DEBUG) Serial.println("--- Entering standby ---");
    }

    switch (LED_STATE) {

        case LED_OFF:
            fill_solid(leds, NUM_LEDS, CRGB::Black);
            FastLED.show();
            break;

        case LED_FLASH_NEW:
            // Latch parameters; rotate hue; fall through to render frame 0
            flashTimer      = LED_FLASH_RELEASE_TIME;
            flashBrightness = loudness * FLASH_MAX_BRIGHTNESS;
            flashHue        = wrap(flashHue + random(63, 191), 0, 255);
            LED_STATE       = LED_FLASH_ONGOING;
            /* fall through */

        case LED_FLASH_ONGOING:
            flashTimer -= RATE_LED;
            if (flashTimer <= 0) {
                fill_solid(leds, NUM_LEDS, CRGB::Black);
                FastLED.show();
                loudnessExtraPeak = 0.0f;
                LED_STATE = LED_OFF;
                break;
            }
            {
                float envelope = expScale((float)flashTimer, 0.0f,
                                          (float)LED_FLASH_RELEASE_TIME,
                                          0.0f, 1.0f, DECAY_EXPONENT);
                int bri = constrain(
                    (int)((flashBrightness + loudnessExtraPeak) * envelope),
                    0, 255);
                ledFlashAnimation(flashHue, bri);
                loudnessExtraPeak = 0.0f;   // consumed this tick
            }
            break;

        case LED_COLLISION_NEW:
            ledCollisionAnimation();           // initialises the animation (state 3→1)
            LED_STATE = LED_COLLISION_ONGOING;
            break;

        case LED_COLLISION_ONGOING:
            if (ledCollisionAnimation()) {     // returns true when sequence ends
                LED_STATE = LED_OFF;
                if (DEBUG) Serial.println("Collision animation complete.");
            }
            break;

        case LED_STANDBY:
            ledStandbyAnimation();
            break;
    }
}

/*
 *  taskLaser
 *  ---------
 *  LASERS_ALL_ON       → all on (applied once on state change)
 *  LASERS_FLASHING_ALL → blink in unison; rate ∝ intensity (sqrt curve)
 *  LASERS_STANDBY      → sweep animation
 */
void taskLaser() {
    static unsigned long lastUpdate        = 0;
    static long          blinkCountdown   = 0;   // long (32-bit on AVR) prevents
                                                 // overflow when task is paused
                                                 // for extended periods
    static bool          lasersCurrentlyOn = false;

    if (systemIdle) LASERS_STATE = LASERS_STANDBY;

    // ---- State-change transitions ---------------------------------------
    if (LASERS_STATE != LASERS_STATE_HIS) {
        if (LASERS_STATE == LASERS_ALL_ON) {
            allLasersOn();
            lasersCurrentlyOn = true;
        }
        blinkCountdown = 0;   // force immediate toggle on first blink tick
        lastUpdate     = now; // prevent huge elapsed on first blink subtraction
    }

    // ---- Steady on -------------------------------------------------------
    if (LASERS_STATE == LASERS_ALL_ON) {
        LASERS_STATE_HIS = LASERS_STATE;
        return;
    }

    // ---- Blink mode ------------------------------------------------------
    if (LASERS_STATE == LASERS_FLASHING_ALL) {
        float rate   = expScale(constrain(intensity, intensityThreshold, intensityCap),
                                intensityThreshold, intensityCap,
                                1.0f, LASERS_BLINK_CPS_MAX, 0.5f);
        int   period = (int)(1000.0f / rate);

        long delta = (long)(now - lastUpdate);
        // Clamp delta to one period so a paused/stale task tick never drives
        // blinkCountdown further negative than needed to trigger one toggle.
        if (delta > (long)period) delta = (long)period;
        blinkCountdown -= delta;
        lastUpdate = now;

        if (blinkCountdown <= 0) {
            blinkCountdown = period;
            if (lasersCurrentlyOn) { allLasersOff(); lasersCurrentlyOn = false; }
            else                   { allLasersOn();  lasersCurrentlyOn = true;  }
        }
    }

    // ---- Standby sweep ---------------------------------------------------
    if (LASERS_STATE == LASERS_STANDBY) {
        laserStandbyAnimation();
    }

    LASERS_STATE_HIS = LASERS_STATE;
}

// ============================================================================
// TEST CALLBACKS
// ============================================================================

void taskTestPiezo() {
    int raw = analogRead(PIEZO_PIN);
    Serial.print("Raw ADC: ");   Serial.print(raw);
    Serial.print(" | Gate: ");   Serial.print(raw > NOISE_GATE ? "YES" : "NO");
    Serial.print(" | Scaled: "); Serial.println(scalePiezoInput(raw), 3);
    digitalWrite(ONBOARD_LED, raw > NOISE_GATE ? HIGH : LOW);
}

void taskTestLEDBasic() {
    static int step = 0;
    FastLED.clear();
    switch (step) {
        case 0: fill_solid(leds, NUM_LEDS, CRGB::White);  Serial.println("WHITE");  break;
        case 1: fill_solid(leds, NUM_LEDS, CRGB::Red);    Serial.println("RED");    break;
        case 2: fill_solid(leds, NUM_LEDS, CRGB::Green);  Serial.println("GREEN");  break;
        case 3: fill_solid(leds, NUM_LEDS, CRGB::Blue);   Serial.println("BLUE");   break;
        case 4: Serial.println("OFF"); break;
    }
    FastLED.show();
    step = (step + 1) % 5;
}

/*
 *  taskTestLEDFlash
 *  ----------------
 *  Called every RATE_LED ms by the scheduler — LED animation runs at full
 *  frame rate (smooth).  Two independent internal timers handle:
 *    - armTimer    : arms a new flash every 1000 ms, cycling through fake
 *                    loudness values 0.2 -> 0.4 -> 0.6 -> 0.8 -> 1.0
 *    - serialTimer : prints the current brightness to Serial every 100 ms
 *                    (readout cadence only — independent of LED render rate)
 *
 *  Serial output when DEBUG is true:
 *    On arm      : "FLASH  loud=X.XX  hue=NNN  bri=NNN"
 *    Every 100ms : "  bri=NNN"
 *    On expire   : "  bri=0 (done)"
 */
void taskTestLEDFlash() {
    static const float   fakeLoudness[]  = { 0.2f, 0.4f, 0.6f, 0.8f, 1.0f };
    static const int     NUM_STEPS       = 5;
    static int           stepIdx         = 0;

    static float         flashBrightness = 0.0f;
    static int           flashHue        = 0;
    static int           flashTimer      = 0;     // ms remaining in release window
    static bool          flashRunning    = false;

    static unsigned long armTimer        = 0;     // timestamp of last arm
    static unsigned long serialTimer     = 0;     // timestamp of last serial print

    // ---- Arm a new flash every 1000 ms ----------------------------------
    if (now - armTimer >= 1000) {
        armTimer = now;

        float fakeL     = fakeLoudness[stepIdx];
        stepIdx         = (stepIdx + 1) % NUM_STEPS;

        flashTimer      = LED_FLASH_RELEASE_TIME;
        flashBrightness = fakeL * FLASH_MAX_BRIGHTNESS;
        flashHue        = wrap(flashHue + random(63, 191), 0, 255);
        flashRunning    = true;
        serialTimer     = now;   // reset serial cadence at the start of each flash

        if (DEBUG) {
            int bri0 = constrain((int)flashBrightness, 0, 255);
            Serial.print("FLASH  loud="); Serial.print(fakeL, 2);
            Serial.print("  hue=");       Serial.print(flashHue);
            Serial.print("  bri=");       Serial.println(bri0);
        }
    }

    // ---- Render LED frame (called every RATE_LED ms by scheduler) -------
    if (!flashRunning) {
        fill_solid(leds, NUM_LEDS, CRGB::Black);
        FastLED.show();
        return;
    }

    flashTimer -= RATE_LED;

    if (flashTimer <= 0) {
        fill_solid(leds, NUM_LEDS, CRGB::Black);
        FastLED.show();
        flashRunning = false;
        if (DEBUG) Serial.println("  bri=0 (done)");
        return;
    }

    float envelope = expScale((float)flashTimer, 0.0f,
                              (float)LED_FLASH_RELEASE_TIME,
                              0.0f, 1.0f, DECAY_EXPONENT);
    int bri = constrain((int)(flashBrightness * envelope), 0, 255);
    ledFlashAnimation(flashHue, bri);

    // ---- Print brightness to Serial every 100 ms (independent of LED rate)
    if (DEBUG && (now - serialTimer >= 100)) {
        serialTimer = now;
        Serial.print("  bri="); Serial.println(bri);
    }
}

void taskTestCollisionAnimation() {
    ledCollisionAnimation();
}

void taskTestLasers() {
    static int idx = 0;
    setLaser(wrap(idx - 1, 0, NUM_LASERS - 1), LOW);
    setLaser(idx, HIGH);
    if (DEBUG) { Serial.print("Laser on: "); Serial.println(idx); }
    idx = wrap(idx + 1, 0, NUM_LASERS - 1);
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    Serial.begin(9600);

    pinMode(ONBOARD_LED, OUTPUT);
    digitalWrite(ONBOARD_LED, LOW);

    FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(255);
    FastLED.clear();
    FastLED.show();

    for (int i = 0; i < NUM_LASERS; i++) {
        pinMode(lasers[i], OUTPUT);
        setLaser(i, HIGH);   // start with all lasers on
    }

    analogReference(DEFAULT);
    lastSoundTime = millis();

    if (SYSTEM_MODE == OPERATIVE) {
        Serial.println("=== NORMAL MODE ===");
    } else {
        for (int i = 0; i < NUM_TASKS;      i++) tasks[i].enabled     = false;
        for (int i = 0; i < NUM_TEST_TASKS; i++) testTasks[i].enabled = false;

        if (SYSTEM_MODE >= 0 && SYSTEM_MODE < NUM_TEST_TASKS) {
            testTasks[SYSTEM_MODE].enabled = true;
            Serial.print("=== TEST MODE ");
            Serial.print(SYSTEM_MODE);
            Serial.println(" ===");
        }
    }
}

// ============================================================================
// LOOP
// ============================================================================

void loop() {
    now = millis();
    if (SYSTEM_MODE == OPERATIVE) {
        for (int i = 0; i < NUM_TASKS;      i++) tasks[i].run(now);
    } else {
        for (int i = 0; i < NUM_TEST_TASKS; i++) testTasks[i].run(now);
    }
}
