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
 *    - Idle (no sound)             → all LEDs off
 *    - Soft hit (0 < loud < 1.0)   → flash animation, brightness ∝ loudness;
 *                                    during the flash release window a quieter
 *                                    secondary hit adds brightness instead of
 *                                    restarting the animation
 *    - Hard hit (loud == 1.0)      → collision animation (runs to completion,
 *                                    ignores any new sound until it finishes)
 *    - After collision             → back to default (flash / off)
 *
 *  LASERS
 *    - intensity < 0.5             → all lasers ON (steady)
 *    - intensity >= 0.5            → all lasers blink in unison;
 *                                    blink rate ∝ intensity
 *
 *  INTENSITY
 *    - Each detected sound adds to intensity (capped at 5.0)
 *    - Intensity decays exponentially toward 0 over time
 *
 *  IDLE
 *    - If no sound detected for IDLE_TIMEOUT ms → enter idle state
 *      (placeholder animation; extend as desired)
 *    - Any new sound → immediately exit idle, return to default
 * =============================================================================
 */

#include <FastLED.h>
#include <string.h>
#include <math.h>

// ============================================================================
// PIN DEFINITIONS
// ============================================================================
#define LED_PIN      10      // WS2812 data pin
#define LASER_PIN_A   5
#define LASER_PIN_B   6
#define LASER_PIN_C  12
#define LASER_PIN_D   7
#define LASER_PIN_E   8
#define ONBOARD_LED  13      // used by TEST_PIEZO to mirror gate output

#define PIEZO_PIN    A1

#define NUM_LEDS     60
#define NUM_LASERS    5

// ============================================================================
// TEST / OPERATIVE MODE
// ============================================================================
#define OPERATIVE        -1
#define TEST_PIEZO        0   // print raw piezo readings to Serial
#define TEST_LED_BASIC    1   // cycle LED colours
#define TEST_BANGERCLASH  2   // run the collision animation at ~60 fps
#define TEST_LASERS       3   // sweep lasers one at a time

int TEST_MODE = OPERATIVE;

// ============================================================================
// DEBUG
// ============================================================================
#define DEBUG false

// ============================================================================
// SOUND STATE
// ============================================================================
/*
 *  After a peak is detected we keep the flash animation alive for
 *  LED_FLASH_RELEASE_TIME ms (the "release window").  During that window,
 *  quieter hits are classified as SECONDARY_SOUND so taskLED can add extra
 *  brightness instead of restarting the flash.
 *
 *  NO_SOUND        - nothing above the noise gate and no active release window
 *  NEW_SOUND       - loudest hit of a new burst (louder than the decaying peak)
 *  SECONDARY_SOUND - hit during the release window but quieter than the peak
 *  REL_SOUND       - release window active, no new hit this window
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
 *  LED_FLASH_NEW     - taskAudio just armed a new flash; taskLED must
 *                      latch parameters and begin the animation this tick
 *  LED_FLASH_RUNNING - flash animation is counting down
 *  The two states let taskLED distinguish "first tick" from "ongoing" without
 *  sharing SOUND_STATE as a trigger (which would be consumed on the same tick).
 */
#define LED_IDLE          0
#define LED_OFF           1
#define LED_FLASH_NEW     2   // armed by taskAudio, consumed by taskLED
#define LED_FLASH_RUNNING 3   // flash countdown in progress
#define LED_COLLISION     4

int LED_STATE = LED_OFF;

// ============================================================================
// LASER STATE
// ============================================================================
#define LASERS_ALL_ON       0
#define LASERS_FLASHING_ALL 1

int LASERS_STATE     = LASERS_ALL_ON;
int LASERS_STATE_HIS = LASERS_ALL_ON;

// ============================================================================
// AUDIO / SIGNAL PARAMETERS
// ============================================================================
#define NOISE_GATE    50        // raw ADC floor
#define SIGNAL_MAX    0.3f      // ADC fraction treated as full scale
#define SAMPLE_WINDOW 200       // mic window length in ms

// ============================================================================
// LED PARAMETERS
// ============================================================================
const float FLASH_MAX_BRIGHTNESS = 180.0f;  // 0-255

// ============================================================================
// TIMING  (all in ms)
// ============================================================================
#define RATE_AUDIO               25
#define RATE_LED                 50
#define RATE_LASER              100
#define RATE_INTENSITY          500

#define LED_FLASH_RELEASE_TIME  500    // flash fade-out duration
#define RATE_HALFDECAY_INTENSITY 3000  // time for intensity to halve
#define IDLE_TIMEOUT            10000  // silence before entering idle

// ============================================================================
// SCALING EXPONENTS
// ============================================================================
float PIEZO_EXPONENT = 2.0f;   // raw -> loudness curve
float DECAY_EXPONENT = 2.0f;   // flash brightness fade curve

// ============================================================================
// HARDWARE
// ============================================================================
CRGB leds[NUM_LEDS];
const int lasers[NUM_LASERS]  = { LASER_PIN_A, LASER_PIN_B,
                                  LASER_PIN_C, LASER_PIN_D, LASER_PIN_E };
int lasersState[NUM_LASERS]   = { LOW, LOW, LOW, LOW, LOW };

// ============================================================================
// SHARED AUDIO STATE  (written by taskAudio, read by other tasks)
// ============================================================================
float loudness          = 0.0f;  // current window loudness [0, 1]
float loudnessExtraPeak = 0.0f;  // extra brightness queued for ongoing flash

// ============================================================================
// PEAK RELEASE TRACKING  (owned by taskAudio)
// ============================================================================
/*
 *  After a peak is detected, lastPeakLoudness decays linearly over
 *  LED_FLASH_RELEASE_TIME ms.  While it is > 0 the release window is active.
 *  A new hit is "new" only if its loudness exceeds the current decayed value.
 *
 *  These are file-scope statics so they persist across calls while remaining
 *  logically private to taskAudio.
 */
static float         lastPeakLoudness = 0.0f;
static unsigned long lastPeakTime     = 0;

// ============================================================================
// INTENSITY
// ============================================================================
float intensity       = 0.0f;
float targetIntensity = 0.0f;

const int INTENSITY_TURNS_FOR_DECAY =
    RATE_HALFDECAY_INTENSITY / RATE_INTENSITY;

float intensityReductionPerTurn = 0.0f;
bool  intensityRefreshed        = false;

// ============================================================================
// IDLE TRACKING
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
void taskTestLED();
void taskTestLasers();
void taskTestCollisionAnimation();

Task tasks[] = {
    { taskAudio,     RATE_AUDIO,     0, true },
    { taskLED,       RATE_LED,       0, true },
    { taskLaser,     RATE_LASER,     0, true },
    { taskIntensity, RATE_INTENSITY, 0, true },
};
const int NUM_TASKS = sizeof(tasks) / sizeof(tasks[0]);

Task testTasks[] = {
    { taskTestPiezo,              50, 0, false },
    { taskTestLED,              1000, 0, false },
    { taskTestLasers,            100, 0, false },
    { taskTestCollisionAnimation, 16, 0, false },
};
const int NUM_TEST_TASKS = sizeof(testTasks) / sizeof(testTasks[0]);

unsigned long now = 0;

// ============================================================================
// UTILITY
// ============================================================================

float expScale(float val, float minIn, float maxIn,
               float minOut, float maxOut, float exponent) {
    val = constrain(val, minIn, maxIn);
    float n = (val - minIn) / (maxIn - minIn);
    return minOut + pow(n, exponent) * (maxOut - minOut);
}

int wrap(int val, int minVal, int maxVal) {
    int range = maxVal - minVal + 1;
    return ((val - minVal) % range + range) % range + minVal;
}

// ============================================================================
// SIGNAL PROCESSING
// ============================================================================

float scalePiezoInput(int rawValue) {
    if (rawValue <= NOISE_GATE) return 0.0f;
    float normalized = (float)(rawValue - NOISE_GATE) / (1023.0f - NOISE_GATE);
    return constrain(
        expScale(normalized, 0.0f, SIGNAL_MAX, 0.0f, 1.0f, PIEZO_EXPONENT),
        0.0f, 1.0f);
}

/*
 *  Peak-detector with sample window.
 *  Returns the scaled peak once per SAMPLE_WINDOW, NAN while accumulating.
 */
float readMicLevel() {
    static int timer = 0;
    static int peak  = 0;
    int s = analogRead(PIEZO_PIN);

    if (timer >= SAMPLE_WINDOW) {
        timer = 0;
        float result = scalePiezoInput(peak);
        peak = 0;
        return result;
    }
    if (s > peak) peak = s;
    timer += RATE_AUDIO;
    return NAN;
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

// ============================================================================
// LED ANIMATION HELPERS
// ============================================================================

void ledFlashAnimation(int hue, int val) {
    for (int i = 0; i < NUM_LEDS; i++) leds[i] = CHSV(hue, 255, val);
    FastLED.show();
}

/*
 *  Two-dot collision animation.  Call every RATE_LED ms.
 *  Returns true only when the full sequence has finished.
 */
bool ledCollisionAnimation() {
    static int colour_1 = 0, colour_2 = 0;
    static int brightness = 20;
    static int dotPos = 0, dotSize = 1;
    static int state = 3;   // 1=moving, 2=explosion, 3=finished/init
    static unsigned long lastUpdate = 0;

    static const int   startSpeed    = 100, endSpeed = 2;
    static const int   xplStartSpeed = 7;
    static const float expAcc        = 0.5f;
    static int  currentSpeed = startSpeed;
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

    // ---- Init / reset ----
    if (state == 3) {
        fill_solid(leds, NUM_LEDS, CRGB::Black);
        int rndPol  = random(2) == 0 ? -1 : 1;
        dotPos = 0;  dotSize = 1;  brightness = 20;
        doneCore = false;  doneCorona = false;
        colour_1 = random(256);
        colour_2 = wrap(colour_1 + 128 + random(64) * rndPol, 0, 255);
        leds[0]            = CHSV(colour_1, 255, brightness);
        leds[NUM_LEDS - 1] = CHSV(colour_2, 255, brightness);
        currentSpeed = startSpeed;  timer = 0;  xplStep = 0;
        state = 1;
        lastUpdate = now;
        FastLED.show();
        return false;
    }

    int deltaTime = (int)(now - lastUpdate);
    timer += deltaTime;

    // ---- Dots moving toward centre ----
    if (state == 1) {
        float movement = (float)timer / (float)currentSpeed;
        if (movement >= 0.75f) {
            for (int c = 0; c < dotSize; c++) {
                leds[dotPos - c]                = CRGB::Black;
                leds[NUM_LEDS - dotPos - 1 + c] = CRGB::Black;
            }
            dotSize = constrain((int)movement, 1, 5);
            dotPos += (int)round(movement + 0.5f);
            if (dotPos >= (NUM_LEDS / 2) - 1) {
                state = 2;
                coronaPosLeft  = (NUM_LEDS / 2) - coronaOffsetInit - 1;
                coronaPosRight = (NUM_LEDS / 2) + coronaOffsetInit;
                xplStep = 0;
                currentSpeed = xplStartSpeed;
            } else {
                brightness = (int)expScale((float)dotPos, 0.0f, 29.0f,
                                           20.0f, 300.0f, 4.0f);
                int bt = brightness;
                for (int c = 0; c < dotSize; c++) {
                    bt = constrain((int)((float)bt * (1.0f - (float)c / 10.0f)), 0, 255);
                    leds[dotPos - c]                = CHSV(colour_1, 255, bt);
                    leds[NUM_LEDS - dotPos - 1 + c] = CHSV(colour_2, 255, bt);
                }
                currentSpeed = (int)expScale((float)dotPos, 0.0f, 29.0f,
                                             (float)startSpeed, (float)endSpeed, expAcc);
                timer = 0;
            }
        }
    }

    // ---- Explosion ----
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

// ============================================================================
// INTENSITY HELPER
// ============================================================================

void refreshIntensity(float val) {
    intensity += val;
    if (intensity > 5.0f) intensity = 5.0f;
    targetIntensity = intensity / 2.0f;
    intensityReductionPerTurn =
        (intensity - targetIntensity) / (float)INTENSITY_TURNS_FOR_DECAY;
    intensityRefreshed = true;
}

// ============================================================================
// TASK CALLBACKS - NORMAL OPERATION
// ============================================================================

/*
 *  taskAudio
 *  ---------
 *  Reads the microphone every RATE_AUDIO ms.  At the end of each SAMPLE_WINDOW
 *  it classifies the hit and updates LED_STATE accordingly.
 *
 *  Peak-release logic
 *  ------------------
 *  When a peak is detected we record its loudness and the timestamp.  On every
 *  subsequent window we compute how much of the release window has elapsed and
 *  derive a linearly-decayed reference level:
 *
 *      decayedPeak = lastPeakLoudness * (1 - elapsed / RELEASE_TIME)
 *
 *  A new window is classified as:
 *    NEW_SOUND       - loudness > decayedPeak  (louder than the fading peak)
 *    SECONDARY_SOUND - 0 < loudness <= decayedPeak, release window still open
 *    REL_SOUND       - loudness == 0, release window still open (no new hit)
 *    NO_SOUND        - loudness == 0 and release window closed
 */
void taskAudio() {
    float s = readMicLevel();
    if (isnan(s)) return;

    loudness = s;

    // --- Compute the linearly-decayed reference level -------------------
    float decayedPeak = 0.0f;
    if (lastPeakLoudness > 0.0f) {
        unsigned long elapsed = now - lastPeakTime;
        if (elapsed < (unsigned long)LED_FLASH_RELEASE_TIME) {
            // Linear decay: peak loudness -> 0 over the release window
            decayedPeak = lastPeakLoudness *
                (1.0f - (float)elapsed / (float)LED_FLASH_RELEASE_TIME);
        } else {
            // Release window has expired
            lastPeakLoudness = 0.0f;
            decayedPeak      = 0.0f;
        }
    }

    // --- Classify the current window ------------------------------------
    if (loudness > decayedPeak && loudness > 0.0f) {
        // New hit louder than the fading reference -> genuine new peak
        SOUND_STATE      = NEW_SOUND;
        lastPeakLoudness = loudness;
        lastPeakTime     = now;

    } else if (loudness > decayedPeak / 4.0f && decayedPeak > 0.0f) {
        // Audible but softer than the fading reference
        SOUND_STATE = SECONDARY_SOUND;

    } else if (loudness == 0.0f && decayedPeak > 0.0f) {
        // Silence but release window still ticking
        SOUND_STATE = REL_SOUND;

    } else {
        // Silence and no open release window
        SOUND_STATE = NO_SOUND;
    }

    // --- Idle exit ------------------------------------------------------
    if (loudness > 0.0f) {
        lastSoundTime = now;
        if (systemIdle) {
            systemIdle   = false;
            LED_STATE    = LED_OFF;
            LASERS_STATE = LASERS_ALL_ON;
            if (DEBUG) Serial.println("--- Exiting idle ---");
        }
    }

    // --- Intensity accumulation -----------------------------------------
    if (SOUND_STATE == NEW_SOUND) {
        refreshIntensity(loudness);
    }

    // --- LED state selection (collision animation is never interrupted) --
    if (LED_STATE != LED_COLLISION) {
        if (SOUND_STATE == NEW_SOUND) {
            if (loudness >= 1.0f) {
                LED_STATE = LED_COLLISION;
            } else {
                // Arm a fresh flash; taskLED will latch parameters when it
                // sees LED_FLASH_NEW on its next tick
                LED_STATE = LED_FLASH_NEW;
            }
        } else if (SOUND_STATE == SECONDARY_SOUND) {
            // Quieter hit during an active flash: queue a brightness boost.
            // Half the secondary loudness is added as extra contribution.
            loudnessExtraPeak += loudness * 0.5f;
        }
        // REL_SOUND and NO_SOUND: taskLED's own countdown handles fading
    }

    if (DEBUG) {
        Serial.print("Loudness: ");        Serial.print(loudness, 3);
        Serial.print(" | DecayedPeak: ");  Serial.print(decayedPeak, 3);
        Serial.print(" | Intensity: ");    Serial.print(intensity, 3);
        Serial.print(" | SoundState: ");   Serial.print(SOUND_STATE);
        Serial.print(" | LEDState: ");     Serial.println(LED_STATE);
    }
}

/*
 *  taskIntensity
 *  -------------
 *  Decays intensity toward 0 over time; halves the target each period.
 *  Sets laser mode based on the current intensity level.
 */
void taskIntensity() {
    static int counter = 1;

    if (intensityRefreshed) {
        counter = 1;
        intensityRefreshed = false;
    }

    if (intensity < targetIntensity) intensity = targetIntensity;

    if (counter >= INTENSITY_TURNS_FOR_DECAY) {
        intensity       = targetIntensity;
        targetIntensity /= 2.0f;
        intensityReductionPerTurn =
            (intensity - targetIntensity) / (float)INTENSITY_TURNS_FOR_DECAY;
        counter = 1;
    } else if (intensity > targetIntensity) {
        intensity -= intensityReductionPerTurn;
        if (intensity < 0.01f) intensity = 0.0f;
    }

    if (intensity > 0.0f) {
        counter++;
    }
    

    LASERS_STATE = (intensity < 0.5f) ? LASERS_ALL_ON : LASERS_FLASHING_ALL;
}

/*
 *  taskLED
 *  -------
 *  Drives the LED strip every RATE_LED ms.
 *
 *  LED_OFF           -> strip dark
 *  LED_FLASH_NEW     -> latch new flash parameters, transition to RUNNING
 *  LED_FLASH_RUNNING -> count down the release timer, render fading flash;
 *                       any accumulated loudnessExtraPeak is folded in once
 *                       per tick and then cleared
 *  LED_COLLISION     -> run collision animation until it signals completion
 *  LED_IDLE          -> gentle slow pulse (placeholder)
 */
void taskLED() {
    static float flashBrightness = 0.0f;
    static int   flashHue        = 0;
    static int   flashTimer      = 0;

    // ---- Idle timeout check --------------------------------------------
    if (!systemIdle
            && LED_STATE != LED_COLLISION
            && (now - lastSoundTime >= IDLE_TIMEOUT)) {
        systemIdle = true;
        LED_STATE  = LED_IDLE;
        if (DEBUG) Serial.println("--- Entering idle ---");
    }

    switch (LED_STATE) {

        // ----------------------------------------------------------------
        case LED_OFF:
            fill_solid(leds, NUM_LEDS, CRGB::Black);
            FastLED.show();
            break;

        // ----------------------------------------------------------------
        case LED_FLASH_NEW:
            /*
             *  Latch the flash parameters for this burst.
             *  Rotate the hue so consecutive flashes look distinct.
             *  Transition immediately to RUNNING so the release timer starts.
             *  Fall through so the first frame renders this same tick.
             */
            flashTimer      = LED_FLASH_RELEASE_TIME;
            flashBrightness = loudness * FLASH_MAX_BRIGHTNESS;
            flashHue        = wrap(flashHue + random(63, 191), 0, 255);
            LED_STATE       = LED_FLASH_RUNNING;
            /* fall through */

        // ----------------------------------------------------------------
        case LED_FLASH_RUNNING:
            flashTimer -= RATE_LED;
            if (flashTimer <= 0) {
                fill_solid(leds, NUM_LEDS, CRGB::Black);
                FastLED.show();
                loudnessExtraPeak = 0.0f;   // discard any leftover boost
                LED_STATE = LED_OFF;
                break;
            }
            {
                /*
                 *  Brightness = (base + extra boost) * exponential envelope.
                 *  loudnessExtraPeak accumulates secondary hits and is consumed
                 *  (zeroed) each LED tick so boosts from multiple secondary
                 *  hits stack correctly without double-counting.
                 */
                float envelope = expScale((float)flashTimer, 0.0f,
                                          (float)LED_FLASH_RELEASE_TIME,
                                          0.0f, 1.0f, DECAY_EXPONENT);
                int bri = constrain(
                    (int)((flashBrightness + loudnessExtraPeak) * envelope),
                    0, 255);
                ledFlashAnimation(flashHue, bri);
                loudnessExtraPeak = 0.0f;   // consumed
            }
            break;

        // ----------------------------------------------------------------
        case LED_COLLISION: {
            bool done = ledCollisionAnimation();
            if (done) {
                LED_STATE = LED_OFF;
                if (DEBUG) Serial.println("Collision animation complete.");
            }
            break;
        }

        // ----------------------------------------------------------------
        case LED_IDLE: {
            /*
             *  Placeholder: slow blue-white pulse.
             *  Replace or extend with richer animations as desired.
             */
            static float idlePhase = 0.0f;
            idlePhase += 0.05f;
            if (idlePhase > TWO_PI) idlePhase -= TWO_PI;
            int idleBri = (int)(40.0f + 30.0f * sinf(idlePhase));
            fill_solid(leds, NUM_LEDS, CHSV(160, 200, idleBri));
            FastLED.show();
            break;
        }
    }
}

/*
 *  taskLaser
 *  ---------
 *  LASERS_ALL_ON       -> steady on (applied once on state change)
 *  LASERS_FLASHING_ALL -> blink in unison; rate ∝ intensity via sqrt curve
 */
void taskLaser() {
    static unsigned long lastToggle       = 0;
    static int           blinkCountdown   = 0;
    static bool          lasersCurrentlyOn = false;

    // ---- Apply state change once ----------------------------------------
    if (LASERS_STATE != LASERS_STATE_HIS) {
        if (LASERS_STATE == LASERS_ALL_ON) {
            allLasersOn();
            lasersCurrentlyOn = true;
        }
        blinkCountdown   = 0;   // force an immediate toggle on the next blink tick
        LASERS_STATE_HIS = LASERS_STATE;
    }

    if (LASERS_STATE == LASERS_ALL_ON) return;

    // ---- Blink mode -----------------------------------------------------
    if (LASERS_STATE == LASERS_FLASHING_ALL) {
        // Map intensity [0.5, 3.0] -> blink rate [1, 20] Hz (sqrt curve so
        // even low intensities produce a visible blink)
        float rate   = expScale(constrain(intensity, 0.5f, 3.0f),
                                0.5f, 3.0f, 1.0f, 20.0f, 0.5f);
        int   period = (int)(1000.0f / rate);   // ms per half-period

        blinkCountdown -= (int)(now - lastToggle);
        lastToggle = now;

        if (blinkCountdown <= 0) {
            blinkCountdown = period;
            if (lasersCurrentlyOn) { allLasersOff(); lasersCurrentlyOn = false; }
            else                   { allLasersOn();  lasersCurrentlyOn = true;  }
        }
    }
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

void taskTestLED() {
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

void taskTestLasers() {
    static int idx = 0;
    setLaser(wrap(idx - 1, 0, NUM_LASERS - 1), LOW);
    setLaser(idx, HIGH);
    if (DEBUG) { Serial.print("Laser on: "); Serial.println(idx); }
    idx = wrap(idx + 1, 0, NUM_LASERS - 1);
}

void taskTestCollisionAnimation() {
    ledCollisionAnimation();
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
        setLaser(i, LOW);
    }

    analogReference(DEFAULT);
    lastSoundTime = millis();

    if (TEST_MODE == OPERATIVE) {
        Serial.println("=== NORMAL MODE ===");
    } else {
        for (int i = 0; i < NUM_TASKS;      i++) tasks[i].enabled     = false;
        for (int i = 0; i < NUM_TEST_TASKS; i++) testTasks[i].enabled = false;
        if (TEST_MODE >= 0 && TEST_MODE < NUM_TEST_TASKS) {
            testTasks[TEST_MODE].enabled = true;
            Serial.print("=== TEST MODE ");
            Serial.print(TEST_MODE);
            Serial.println(" ===");
        }
    }
}

// ============================================================================
// LOOP
// ============================================================================

void loop() {
    now = millis();
    if (TEST_MODE == OPERATIVE) {
        for (int i = 0; i < NUM_TASKS;      i++) tasks[i].run(now);
    } else {
        for (int i = 0; i < NUM_TEST_TASKS; i++) testTasks[i].run(now);
    }
}
