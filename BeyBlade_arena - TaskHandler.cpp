#include <FastLED.h>
#include <string.h>
#include <math.h>

// ============================================================================
// PIN DEFINITIONS
// ============================================================================

// OUTPUTS
#define LED_PIN     10      // NeoPixel data pin
#define LASER_PIN_A 5       // Laser A data pin 
#define LASER_PIN_B 6       // Laser B data pin
#define LASER_PIN_C 12      // Laser C data pin
#define LASER_PIN_D 7       // Laser D data pin
#define LASER_PIN_E 8       // Laser E data pin
#define ONBOARD_LED   13

// INPUTS
#define MIC_PIN       A1

// NUM OF CONTROLLED DEVICES
#define NUM_LEDS    60
#define NUM_LASERS   5

// ============================================================================
// TEST MODES
// ============================================================================
#define OPERATIVE          -1     
#define TEST_PIEZO         0    // Test piezo microphone readings
#define TEST_LED_BASIC     1    // Test LED strip with color cycle
#define TEST_BANGERCLASH   2    // Test "BangerCrash" LED animation      
#define TEST_LASERS        3    // Test Lasers by turning them on and off  

int TEST_MODE = OPERATIVE;

// ============================================================================
// DEBUG
// ============================================================================
#define DEBUG true

// ============================================================================
// LED STATE
// ============================================================================
#define LED_IDLE 0
#define LED_NEW_ANIMATION       1
#define LED_ANIMATION_ONGOING   2
#define LED_ANIMATION_SWIRL     3 // placeholder

int LED_STATE = LED_IDLE;

// ============================================================================
// LASERS STATE
// ============================================================================
#define LASERS_OFF              0
#define LASERS_ALL_ON           1
#define LASERS_FLASHING_ALL     2
#define LASERS_FLASHING_RANDOM  3 // placeholder
#define LASERS_FLASHING_SWIRL   4 // placeholder

int LASERS_STATE = LASERS_ALL_ON;
int LASERS_STATE_HIS = LASERS_STATE;

// ============================================================================
// LED CLASH RESPONSIVE ANIMATION TYPES
// ============================================================================
#define VMETER        0
#define FLASH         1
#define COLLISION     2

int LED_CLASH_RESPONSE_ANIMATION = FLASH;
int LED_DEFAULT_ANIMATION = FLASH;

// ============================================================================
// DETECTED SOUD STATES
// ============================================================================
#define NO_SOUND        0
#define NEW_SOUND       1
#define SUS_SOUND       2

int SOUND_STATE = NO_SOUND;

// ============================================================================
// AUDIO PARAMETERS
// ============================================================================
#define NOISE_GATE      50
#define SIGNAL_MAX      0.3f
#define SAMPLE_WINDOW   200    // how often we detect a peak

// ============================================================================
// LEDS PARAMETERS
// ============================================================================
#define FLASH_ANIMATION_MAX_BRIGHTESS   180.0f; //range 0.0 to 255.0

// ============================================================================
// TIMING PARAMETERS  (all in ms)
// ============================================================================
#define RATE_AUDIO                      25   // how often we sample the mic
#define RATE_LED                        50   // how often we update LED animations
#define RATE_LASER                     100   // how often we update laser animations
#define RATE_INTENSITY                 500   // how often we update calculation of intensity
#define LED_FLASH_RELEASE_TIME         500   // led flash release duration
#define LED_VMETER_CELL_RELEASE_TIME    50   // led Vmeter single-cell fade release duration
#define RATE_HALFDECAY_INTENSITY      3000  // the amount of time intensity takes to halve

// ============================================================================
// SCALING
// ============================================================================
float PIEZO_EXPONENT    = 2.0f;
float DECAY_EXPONENT    = 2.0f;

// ============================================================================
// HARDWARE ARRAYS
// ============================================================================
CRGB leds[NUM_LEDS];
const int lasers[NUM_LASERS] = {LASER_PIN_A, LASER_PIN_B, LASER_PIN_C, LASER_PIN_D, LASER_PIN_E};
int   lasersState[NUM_LASERS] = {LOW, LOW, LOW, LOW, LOW};

// ============================================================================
// SHARED AUDIO STATE  (written by audio task, read by LED / laser tasks)
// ============================================================================
float loudness    = 0.0f;
float loudnessHis = 0.0f;
float loudnessExtraPeak = 0.0f;

// ============================================================================
// INTENSITY
// ============================================================================
float intensity = 0.0f;
float targetIntensity = 0.0f;
int intensityTurnsForDecay = RATE_HALFDECAY_INTENSITY /  RATE_INTENSITY; 
float intensityReductionPerTurn = 0.0
bool intensityRefreshed = false;

// ============================================================================
// TASK SCHEDULER
// ============================================================================
// Each Task holds a function pointer, an interval, and the timestamp of its
// last execution.  The scheduler calls run() every loop() tick and fires the
// callback when the interval has elapsed.

struct Task {
    void (*callback)();      // function to call
    unsigned long interval;  // period in ms
    unsigned long lastRun;   // timestamp of last execution
    bool enabled;

    void run(unsigned long now) {
        if (!enabled) return;
        if (now - lastRun >= interval) {
            lastRun = now;
            callback();
        }
    }
};

// Forward-declare every task callback so we can build the table before the
// function bodies appear below.

// Operative mode 
void taskAudio();
void taskLED();
void taskLaser();
void taskIntensity();

// Test modes
void taskTestPiezo();
void taskTestLED();
void taskTestLasers();
void taskTestCollisionAnimation();

// Task table — edit intervals or toggle enabled here.
Task tasks[] = {
    { taskAudio,          RATE_AUDIO,  0, true },
    { taskLED,            RATE_LED,    0, true },
    { taskLaser,          RATE_LASER,  0, true },
    { taskIntensity,      RATE_INTENSITY,  0, true },
};
const int NUM_TASKS = sizeof(tasks) / sizeof(tasks[0]);

// Test-mode task table (only one is active at runtime, chosen in setup())
Task testTasks[] = {
    { taskTestPiezo,       50,  0, false },
    { taskTestLED,       1000,  0, false },
    { taskTestLasers,     100,  0, false },
    { taskTestCollisionAnimation, 16,  0, false },  // ~60 fps
};
const int NUM_TEST_TASKS = sizeof(testTasks) / sizeof(testTasks[0]);
unsigned long now = millis();

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
    return constrain(expScale(normalized, 0.0f, SIGNAL_MAX, 0.0f, 1.0f, PIEZO_EXPONENT), 0.0f, 1.0f);
}

float readMicLevel() {
    static int timer = 0;
    static int peak = 0;
    int s = analogRead(MIC_PIN);
    bool end_window = (timer >= SAMPLE_WINDOW);
    if (end_window) {
        timer = 0;
        float scaled_signal = scalePiezoInput(peak);
        peak = 0;
        return scaled_signal;
    } else {
        if (s > peak) peak = s;
        timer += RATE_AUDIO;
        return NAN;
    }   
}

// ============================================================================
// LASER HELPERS
// ============================================================================

void setLaser(int idx, int state) {
    digitalWrite(lasers[idx], state);
    lasersState[idx] = state;
}

void allLasersOff() {
    for (int i = 0; i < NUM_LASERS; i++) setLaser(i, LOW);
}

void allLasersOn() {
    for (int i = 0; i < NUM_LASERS; i++) setLaser(i, HIGH);
}

// ============================================================================
// LED ANIMATION HELPERS
// ============================================================================

void ledFlashAnimation(int hue, int val) {
    for (int i = 0; i < NUM_LEDS; i++) leds[i] = CHSV(hue, 255, val);
    FastLED.show();
}

// Returns true while animation is still running
bool ledVmeterAnimation(float signal) {
    static const int greenEnd  = 24;
    static const int yellowEnd = 40;
    static const int orangeEnd = 52;

    static int  nLedsUsedPeak = 0;
    static unsigned long lastFade = 0;

    if (SOUND_STATE == NEW_SOUND) {
        int target = (int)(signal * NUM_LEDS);
        for (int i = 0; i < NUM_LEDS; i++) {
            if (i < target) {
                if      (i < greenEnd)  leds[i] = CRGB::Green;
                else if (i < yellowEnd) leds[i] = CRGB::Yellow;
                else if (i < orangeEnd) leds[i] = CRGB::Orange;
                else                    leds[i] = CRGB::Red;
            } else {
                leds[i] = CRGB::Black;
            }
        }
        nLedsUsedPeak = target;
        lastFade = now;
    }

    if (SOUND_STATE == SOUND_SUS && now - lastFade >= LED_VMETER_CELL_RELEASE_TIME) {
        if (nLedsUsedPeak > 0) {
            leds[--nLedsUsedPeak] = CRGB::Black;
        }
        lastFade = now;
    }

    FastLED.show();
    return (nLedsUsedPeak > 0);
}

bool ledCollisionAnimation() {
    // --- same logic as original, just receives millis() value ---
    static int colour_1 = 0, colour_2 = 0;
    static int brightness = 20;
    static int dotPos = 0, dotSize = 1;
    static int state = 3; // 1: dots moving; 2: explosion; 3 = inactive/end
    static unsigned long lastUpdate = 0;
    static const int startSpeed = 100, endSpeed = 2;
    static const int xplStartSpeed = 7;
    static const float expAcc = 0.5f;
    static int currentSpeed = startSpeed;
    static int timer = 0, xplStep = 0;
    static const int colourWhite[3]  = {45,   0, 255};
    static const int colourYellow[3] = {45, 255, 255};
    static const int colourRed[3]    = { 0, 255, 255};
    static int currentColourCore[3]   = {45,   0, 255};
    static int currentColourCorona[3] = {45, 255, 255};
    static int coronaPosLeft = 0, coronaPosRight = 0;
    static int coronaLeftDots[27]  = {0};
    static int coronaRightDots[27] = {0};
    static bool doneCore = false, doneCorona = false;
    const int coronaOffsetInit = 3;

    if (state == 3) { // INIT new run
        fill_solid(leds, NUM_LEDS, CRGB::Black);
        int rndPol = random(2) == 0 ? -1 : 1;
        dotPos = 0; dotSize = 1; brightness = 20;
        doneCore = false; doneCorona = false;
        colour_1 = random(256);
        colour_2 = wrap(colour_1 + 128 + random(64) * rndPol, 0, 255);
        leds[0]           = CHSV(colour_1, 255, brightness);
        leds[NUM_LEDS - 1] = CHSV(colour_2, 255, brightness);
        currentSpeed = startSpeed; timer = 0; xplStep = 0;
        state = 1; //dots moving
        lastUpdate = now;
        FastLED.show();
        return false;
    }

    int deltaTime = (int)(now - lastUpdate);
    timer += deltaTime;

    if (state == 1) { // dots moving
        float movement = (float)timer / (float)currentSpeed;
        if (movement >= 0.75f) {
            for (int c = 0; c < dotSize; c++) {
                leds[dotPos - c]                    = CRGB::Black;
                leds[NUM_LEDS - dotPos - 1 + c]     = CRGB::Black;
            }
            dotSize = constrain((int)movement, 1, 5);
            dotPos += (int)round(movement + 0.5f);

            if (dotPos >= (NUM_LEDS / 2) - 1) {
                state = 2; // explosion
                coronaPosLeft  = (NUM_LEDS / 2) - coronaOffsetInit - 1;
                coronaPosRight = (NUM_LEDS / 2) + coronaOffsetInit;
                xplStep = 0;
                currentSpeed = xplStartSpeed;
            } else {
                brightness = (int)expScale((float)dotPos, 0.0f, 29.0f, 20.0f, 300.0f, 4.0f);
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

    if (state == 2) { // explosion
        const int cL = (NUM_LEDS / 2) - 1;
        const int cR = (NUM_LEDS / 2);
        memcpy(currentColourCore,   colourWhite,  sizeof(colourWhite));
        memcpy(currentColourCorona, colourYellow, sizeof(colourYellow));

        if (xplStep < 4) {
            currentColourCore[2] = map(xplStep, 0, 3, brightness, 255);
            for (int o = 0; o < 3; o++) {
                leds[cL - o] = CHSV(currentColourCore[0], currentColourCore[1], currentColourCore[2]);
                leds[cR + o] = leds[cL - o];
            }
        } else if (xplStep < 10) {
            for (int o = 1; o < 3; o++) {
                currentColourCorona[1] = map(xplStep, 4, 9, colourWhite[1], colourYellow[1]);
                leds[cL - o] = CHSV(currentColourCorona[0], currentColourCorona[1], currentColourCorona[2]);
                leds[cR + o] = leds[cL - o];
            }
        } else if (xplStep < 16) {
            leds[cL] = CHSV(currentColourCore[0], currentColourCore[1], map(xplStep, 10, 15, 255, 0));
            leds[cR] = leds[cL];
            for (int o = 1; o < 3; o++) {
                currentColourCorona[0] = map(xplStep, 10, 15, colourYellow[0], colourRed[0]);
                currentColourCorona[2] = map(xplStep, 10, 15, 255, 0);
                leds[cL - o] = CHSV(currentColourCorona[0], currentColourCorona[1], currentColourCorona[2]);
                leds[cR + o] = leds[cL - o];
            }
        } else {
            doneCore = true;
        }

        if (xplStep > 0) {
            if (xplStep < 8) {
                for (int idx = (xplStep - 1) * 4; idx < (xplStep - 1) * 4 + 4; idx++) {
                    if (idx < 27) { coronaLeftDots[idx] = 255; coronaRightDots[idx] = 255; }
                }
            }
            if (xplStep > 1) {
                for (int idx = 0; idx < (xplStep - 1) * 4 && idx < 27; idx++) {
                    if (coronaLeftDots[idx]  > 0) { coronaLeftDots[idx]  -= 256 / 16; }
                    if (coronaRightDots[idx] > 0) { coronaRightDots[idx] -= 256 / 16; }
                }
            }
            if (coronaLeftDots[26] <= 0 && xplStep > 8) doneCorona = true;

            for (int idx = 0; idx < 27; idx++) {
                leds[26 - idx] = CHSV(0, 255, constrain(coronaLeftDots[idx],  0, 255));
                leds[33 + idx] = CHSV(0, 255, constrain(coronaRightDots[idx], 0, 255));
            }

            if (doneCore && doneCorona) { state = 3; }
        }
        xplStep++;
    }

    lastUpdate = now;
    FastLED.show();
    return (state == 3);
}

// ============================================================================
// TASK CALLBACKS
// ============================================================================

// --- AUDIO TASK ---
// Reads mic level and updates shared loudness + SOUND_STATE.
void taskAudio() {
    float s = readMicLevel(); // returns value in range 0.0 (noise gate floor) , 1.0 (max threhsold ceiling), or -1.0 when the sample window hasn't finished
    
    if (!isnan(s)) { { // wait until the audio sample window is updated
        loudness = s; // pass the value at the end of sample window

        // UPDATE SOUND_STATE
        if (loudness == 0.0f) { 
            SOUND_STATE = NO_SOUND;
        } else {
            SOUND_STATE = (SOUD_STATE == NO_SOUND) ? NEW_SOUND : SUS_SOUND;
        }

        // SELECT LED ANIMATION BASED ON CONDITIONS
        if (LED_CLASH_RESPONSE_ANIMATION != COLLISION) { //no collision animation going on
        
            if (loudness == 1.0f) {
                LED_STATE = LED_NEW_ANIMATION; //LED_ANIMATION_ONGOING
                LED_CLASH_RESPONSE_ANIMATION = COLLISION;          
            } else if (loudness > 0.0f) { // if it's flash or v-meter and some audio has been detected
                if (loudness > loudnessHis) { // new flash animation if new audio sample window is louder than the previous one
                    if (LED_CLASH_RESPONSE_ANIMATION == FLASH) {
                        LED_STATE = LED_NEW_ANIMATION;
                    }
                } else { // if new audio sample window is not louder than the previous one, yet is louder than 1/4 of the previous one, a minor flash will be added to the flash animation (colour of the flash won't change though)
                    if (LED_CLASH_RESPONSE_ANIMATION == FLASH) {
                        if ((LED_STATE = LED_ANIMATION_ONGOING) && (loudnessHis > loudness / 4.0)) {
                            loudnessExtraPeak += loudnessHis / 2.0;
                        }
                    }
                }
            }
            
        } //else { // no noise detected at all



        if (DEBUG) {
            Serial.print("Loudness: "); Serial.print(loudness, 3);
            Serial.print(" | LED State: "); Serial.print(LED_STATE);
            Serial.print(" | LED Animation: "); Serial.println(LED_CLASH_RESPONSE_ANIMATION);
        }

        loudnessHis = loudness;
    }
}

void refreshIntensity(float val) {
    intensity += val;
    if (intensity > 5.0) {
        intensity = 5.0f;
    }
    targetIntensity = intensity / 2.0 ;
    intensityReductionPerTurn = targetIntensity / (float)(intensityTurnsForDecay);
    intensityRefreshed = true;
}

// --- INTENSITY TASK ---
// Intensity naturally decreases with time. Controls laser animations
void taskIntensity() {
    static int counter = 1;
    if (intensity < targetIntensity) {
        intensity = targetIntensity; // intensity should never be below target Intensity
    }
    if (intensityRefreshed) {
        counter = 1; // reset counter
    }
    if (counter == intensityTurnsForDecay) { // if no intensity refresh for the duration of the half decay, proceed to calculate next half decay
        intensity = targetIntensity;
        targetIntensity /= 2.0;
        counter = 1;
    } else if (intensity > targetIntensity) { 
        intensity -= intensityReductionPerTurn;
        if (intensity < 0.1) {
            intensity = 0.0;
        }
    }

    if (intensity < 0.5) {
        LASERS_STATE = LASERS_ALL_ON;
    } else {
        LASERS_STATE = LASERS_FLASHING_ALL;
    }
}
// --- LED TASK ---
// Drives whichever LED animation is selected, based on the current STATE.
void taskLED() {
    static float  flashBrightness_f = 0.0f;
    static int    flashHue           = 0;
    static int    clashTimer         = 0;
    static bool   active             = false;

    if (!active) { // turn all leds off
        if (SOUND_STATE == NO_SOUND) {
            fill_solid(leds, NUM_LEDS, CRGB::Black);
            FastLED.show();
            return;
        }
        active = true; // new signal arrived
    }

    // --- FLASH ---
    if (LED_CLASH_RESPONSE_ANIMATION == FLASH) {
        if (SOUND_STATE == NEW_SOUND) {
            clashTimer = LED_FLASH_RELEASE_TIME;
            flashBrightness_f = loudness * FLASH_ANIMATION_MAX_BRIGHTESS;
            // Pick a hue that won't look the same as last time
            flashHue = wrap(flashHue + random(63, 191), 0, 255);
            SOUND_STATE = SOUND_SUS;
        } else if (SOUND_STATE == SOUND_SUS) {        
            clashTimer -= RATE_LED;
            if (clashTimer <= 0) {
                fill_solid(leds, NUM_LEDS, CRGB::Black);
                FastLED.show();
                active = false;
                return;
            }
            
            // CALCULATE THE BRIGHTNESS
            int bri = (int)(flashBrightness_f + loudnessExtraPeak) *
                expScale((float)clashTimer, 0.0f, (float)LED_FLASH_RELEASE_TIME, 0.0f, 1.0f, DECAY_EXPONENT));
            ledFlashAnimation(flashHue, bri);
            loudnessExtraPeak = 0.0; // reset value
        }
    }

    // --- VMETER ---
    if (LED_CLASH_RESPONSE_ANIMATION == VMETER) {
        active = ledVmeterAnimation(loudness);
    }

    // --- COLLISION ---
    if (LED_CLASH_RESPONSE_ANIMATION == COLLISION) {
        bool done = ledCollisionAnimation();
        if (done) {
            LED_CLASH_RESPONSE_ANIMATION = LED_DEFAULT_ANIMATION; // collision animation finished; back to default
        }

    }
}

// --- LASER TASK ---
// Drives the laser animations.
void taskLaser() {
    
    static unsigned long lastChange = 0;
    static int laserIdx = 0;
    static int countDown = 0;
    static bool allOn = false;

    float rate;

    // ALL LASERS ON
    if ( (LASERS_STATE = LASERS_ALL_ON) && (LASERS_STATE != LASERS_STATE_HIS) ) {
        allLasersOn();
        allOn = true;
    }

    // ALL LASERS OFF
    if ( (LASERS_STATE = LASERS_ALL_OFF) && (LASERS_STATE != LASERS_STATE_HIS) ) {
        allLasersOff();
        allOn = false;
    }

    // ALL LASERS FLASHING - RATE DETERMINED BY INTENSITY
    if (LASERS_STATE == LASERS_FLASHING_ALL) {
        // Map intensity → rate (1–20 Hz) and number of lasers on (1–NUM_LASERS)
        rate = expScale(constrain(intensity,0.0f,3.0f), 0.0f,1.0f, 1.0f, 20.0f, 0.5f);
    }

    int period = (int)(1000.0f / rate); // ms per step
    countDown -= (int)(now - lastChange);
    lastChange = now;

    if (countDown <= 0) {
        countDown = period; //reset
        if (allOn) {
            allLasersOff();
        } else {
            allLasersOn();
        }
        allOn = (!allOn);
    }

    LASERS_STATE = LASERS_STATE_HIS;
}

// ============================================================================
// TEST TASK CALLBACKS
// ============================================================================

void taskTestPiezo() {
    int raw = analogRead(MIC_PIN);
    Serial.print("Raw ADC: "); Serial.print(raw);
    Serial.print(" | Gate: "); Serial.print(raw > NOISE_GATE ? "YES" : "NO");
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
    ledCollisionAnimation(millis());
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
    }

    analogReference(DEFAULT);

    // --- Configure the task tables for the selected mode ---
    if (TEST_MODE == OPERATIVE) {
        Serial.println("=== NORMAL MODE ===");
        // All three normal tasks are already enabled by default in the table.

    } else {
        // In test mode disable all normal tasks and enable only the matching test task.
        for (int i = 0; i < NUM_TASKS; i++)      tasks[i].enabled = false;
        for (int i = 0; i < NUM_TEST_TASKS; i++) testTasks[i].enabled = false;

        if (TEST_MODE >= 0 && TEST_MODE < NUM_TEST_TASKS) {
            testTasks[TEST_MODE].enabled = true;
            Serial.print("=== TEST MODE "); Serial.print(TEST_MODE); Serial.println(" ===");
        }
    }
}

// ============================================================================
// MAIN LOOP  — just a scheduler tick
// ============================================================================

void loop() {
    now = millis();

    if (TEST_MODE == OPERATIVE) {
        for (int i = 0; i < NUM_TASKS; i++)      tasks[i].run(now);
    } else {
        for (int i = 0; i < NUM_TEST_TASKS; i++) testTasks[i].run(now);
    }
}
