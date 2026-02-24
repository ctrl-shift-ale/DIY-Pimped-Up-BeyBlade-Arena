#include <FastLED.h>
#include <string.h>
#include <math.h>

// ============================================================================
// PIN DEFINITIONS
// ============================================================================
#define LED_PIN       10
#define LASER_PIN_A    5
#define LASER_PIN_B    6
#define LASER_PIN_C    7
#define LASER_PIN_D    8
#define ONBOARD_LED   13
#define MIC_PIN       A1

#define NUM_LEDS    60
#define NUM_LASERS   4

// ============================================================================
// TEST MODES
// ============================================================================
#define NO_TEST           -1
#define TEST_PIEZO         0
#define TEST_LED           1
#define TEST_BANGERCLASH   2
#define TEST_LASERS        3

int TEST_MODE = NO_TEST;

// ============================================================================
// DEBUG
// ============================================================================
#define DEBUG true

// ============================================================================
// ANIMATION TYPES
// ============================================================================
#define NO_ANIMATION -1
#define VMETER        0
#define FLASH         1

int CLASH_ANIMATION = FLASH;

// ============================================================================
// STATES
// ============================================================================
#define OFF           0
#define CLASH_NEW     1
#define CLASH_ONGOING 2

int STATE = OFF;

// ============================================================================
// AUDIO PARAMETERS
// ============================================================================
#define NOISE_GATE      50
#define SIGNAL_MAX       0.3f
#define SAMPLE_WINDOW   20    // ms — kept short so audio task doesn't hog the CPU

// ============================================================================
// TIMING PARAMETERS  (all in ms)
// ============================================================================
#define RATE_AUDIO          25   // how often we sample the mic
#define RATE_LED            50   // how often we update LED animations
#define RATE_LASER          30   // how often we update laser animations
#define FLASH_SETTLE       500   // flash fade duration
#define VMETER_CELL_FADE    50   // single-cell fade step (vmeter)

// ============================================================================
// SCALING
// ============================================================================
float PIEZO_EXPONENT    = 2.0f;
float DECAY_EXPONENT    = 2.0f;

// ============================================================================
// HARDWARE ARRAYS
// ============================================================================
CRGB leds[NUM_LEDS];
const int lasers[NUM_LASERS] = {LASER_PIN_A, LASER_PIN_B, LASER_PIN_C, LASER_PIN_D};
int   lasersState[NUM_LASERS] = {LOW, LOW, LOW, LOW};

// ============================================================================
// SHARED AUDIO STATE  (written by audio task, read by LED / laser tasks)
// ============================================================================
float loudness    = 0.0f;
float loudnessHis = 0.0f;

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
void taskAudio();
void taskLED();
void taskLaser();
void taskTestPiezo();
void taskTestLED();
void taskTestLasers();
void taskTestBangerClash();

// Task table — edit intervals or toggle enabled here.
Task tasks[] = {
    { taskAudio,          RATE_AUDIO,  0, true },
    { taskLED,            RATE_LED,    0, true },
    { taskLaser,          RATE_LASER,  0, true },
};
const int NUM_TASKS = sizeof(tasks) / sizeof(tasks[0]);

// Test-mode task table (only one is active at runtime, chosen in setup())
Task testTasks[] = {
    { taskTestPiezo,       50,  0, false },
    { taskTestLED,       1000,  0, false },
    { taskTestLasers,     100,  0, false },
    { taskTestBangerClash, 16,  0, false },  // ~60 fps
};
const int NUM_TEST_TASKS = sizeof(testTasks) / sizeof(testTasks[0]);

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
    unsigned long start = millis();
    int peak = 0;
    while (millis() - start < SAMPLE_WINDOW) {
        int s = analogRead(MIC_PIN);
        if (s > peak) peak = s;
    }
    return scalePiezoInput(peak);
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

// ============================================================================
// LED ANIMATION HELPERS
// ============================================================================

void ledFlashAnimation(int hue, int val) {
    for (int i = 0; i < NUM_LEDS; i++) leds[i] = CHSV(hue, 255, val);
    FastLED.show();
}

// Returns true while animation is still running
bool ledVmeterAnimation(float signal, unsigned long now) {
    static const int greenEnd  = 24;
    static const int yellowEnd = 40;
    static const int orangeEnd = 52;

    static int  nLedsUsedPeak = 0;
    static unsigned long lastFade = 0;

    if (STATE == CLASH_NEW) {
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

    if (STATE == CLASH_ONGOING && now - lastFade >= VMETER_CELL_FADE) {
        if (nLedsUsedPeak > 0) {
            leds[--nLedsUsedPeak] = CRGB::Black;
        }
        lastFade = now;
    }

    FastLED.show();
    return (nLedsUsedPeak > 0);
}

bool bangerClashAnimation(unsigned long now) {
    // --- same logic as original, just receives millis() value ---
    static int colour_1 = 0, colour_2 = 0;
    static int brightness = 20;
    static int dotPos = 0, dotSize = 1;
    static int state = 3; // 3 = inactive/end
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
        state = 1;
        lastUpdate = now;
        FastLED.show();
        return true;
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
                state = 2;
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
    return (state != 3);
}

// ============================================================================
// TASK CALLBACKS
// ============================================================================

// --- AUDIO TASK ---
// Reads mic level and updates shared loudness + STATE.
void taskAudio() {
    loudness = readMicLevel();

    if (loudness > 0.0f) {
        STATE = (loudness > loudnessHis) ? CLASH_NEW : CLASH_ONGOING;
    } else {
        STATE = OFF;
    }

    if (DEBUG) {
        Serial.print("Loudness: "); Serial.print(loudness, 3);
        Serial.print(" | State: "); Serial.println(STATE);
    }

    loudnessHis = loudness;
}

// --- LED TASK ---
// Drives whichever LED animation is selected, based on the current STATE.
void taskLED() {
    static float  flashBrightness_f = 0.0f;
    static int    flashHue           = 0;
    static int    clashTimer         = 0;
    static bool   active             = false;

    if (CLASH_ANIMATION == NO_ANIMATION) return;

    unsigned long now = millis();

    if (!active) {
        if (STATE == OFF) {
            fill_solid(leds, NUM_LEDS, CRGB::Black);
            FastLED.show();
            return;
        }
        active = true; // new signal arrived
    }

    // --- FLASH ---
    if (CLASH_ANIMATION == FLASH) {
        if (STATE == CLASH_NEW) {
            clashTimer       = FLASH_SETTLE;
            flashBrightness_f = loudness * 255.0f;
            // Pick a hue that won't look the same as last time
            flashHue = wrap(flashHue + random(63, 191), 0, 255);
        }

        if (STATE == CLASH_ONGOING || STATE == CLASH_NEW) {
            if (STATE == CLASH_ONGOING) {
                clashTimer -= RATE_LED;
                if (clashTimer <= 0) {
                    fill_solid(leds, NUM_LEDS, CRGB::Black);
                    FastLED.show();
                    active = false;
                    return;
                }
            }
            int bri = (int)(flashBrightness_f *
                expScale((float)clashTimer, 0.0f, (float)FLASH_SETTLE, 0.0f, 1.0f, DECAY_EXPONENT));
            ledFlashAnimation(flashHue, bri);
        }
    }

    // --- VMETER ---
    if (CLASH_ANIMATION == VMETER) {
        active = ledVmeterAnimation(loudness, now);
    }
}

// --- LASER TASK ---
// Drives the laser swirl effect, also reacting to STATE.
void taskLaser() {
    // The swirl rate and number of lasers on can be modulated by loudness.
    // Right now: idle = slow 1-laser sweep; on clash = fast multi-laser burst.

    static unsigned long lastChange = 0;
    static int laserIdx = 0;
    static int countDown = 0;

    unsigned long now = millis();

    float rate;
    int   nOn;

    if (STATE == OFF) {
        rate = 1.0f;  // 1 sweep per second while quiet
        nOn  = 1;
    } else {
        // Map loudness → rate (1–20 Hz) and number of lasers on (1–NUM_LASERS)
        rate = expScale(loudness, 0.0f, 1.0f, 1.0f, 20.0f, 0.5f);
        nOn  = 1 + (int)(loudness * (NUM_LASERS - 1));
    }

    int period = (int)(1000.0f / rate); // ms per step
    countDown -= (int)(now - lastChange);
    lastChange = now;

    if (countDown <= 0) {
        countDown = period;
        allLasersOff();
        // Turn on nOn consecutive lasers starting at laserIdx
        for (int i = 0; i < nOn; i++) {
            setLaser(wrap(laserIdx + i, 0, NUM_LASERS - 1), HIGH);
        }
        laserIdx = wrap(laserIdx + 1, 0, NUM_LASERS - 1);
    }
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

void taskTestBangerClash() {
    bangerClashAnimation(millis());
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

    // --- Configure the task tables for the selected mode ---
    if (TEST_MODE == NO_TEST) {
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
    unsigned long now = millis();

    if (TEST_MODE == NO_TEST) {
        for (int i = 0; i < NUM_TASKS; i++)      tasks[i].run(now);
    } else {
        for (int i = 0; i < NUM_TEST_TASKS; i++) testTasks[i].run(now);
    }
}
