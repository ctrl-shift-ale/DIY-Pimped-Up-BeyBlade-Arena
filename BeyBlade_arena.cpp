#include <FastLED.h>
#include <string.h>
#include <math.h>

// ============================================================================
// PIN DEFINITIONS
// ============================================================================
#define LED_PIN 5          // NeoPixel data pin (can use D2-D12)
#define MIC_PIN A1         // Piezo mic on analog pin A1
#define NUM_LEDS 60        // Number of LEDs in strip
#define ONBOARD_LED 13     // Onboard LED pin for debugging

// ============================================================================
// TEST MODES
// ============================================================================
#define NO_TEST -1
#define TEST_PIEZO 0       // Test piezo microphone readings
#define TEST_LED 1         // Test LED strip with color cycle
#define TEST_BANGERCLASH 2 // Test "BangerCrash" LED animation

int TEST_MODE = TEST_BANGERCLASH;   // Change to enable test mode

// ============================================================================
// DEBUG SETTINGS
// ============================================================================
#define DEBUG true         // Enable serial debug output

// ============================================================================
// ANIMATION TYPES
// ============================================================================
#define NO_ANIMATION -1
#define VMETER 0           // Volume meter (green->yellow->orange->red)
#define FLASH 1            // Random color flash

int CLASH_ANIMATION = FLASH; // Select animation type

// ============================================================================
// LED STATES
// ============================================================================
#define OFF 0              // LEDs off, idle state
#define CLASH_NEW 1        // New peak detected, animation starting
#define CLASH_ONGOING 2    // Animation in progress, fading
#define PATTERN 3          // Reserved for future pattern mode

int STATE = OFF;

// ============================================================================
// AUDIO PARAMETERS
// ============================================================================
#define NOISE_GATE 50      // Threshold value (0-1023 for 10-bit ADC)
#define SIGNAL_MAX 0.3     // signal above this will be scaled to 1.0
#define SAMPLE_WINDOW 50   // Sample window in ms for peak detection
#define UPDATE_RATE_MAIN 25 // Main loop update rate in ms

// ============================================================================
// ANIMATION TIMING PARAMETERS
// ============================================================================
#define UPDATE_RATE_LED 50    // Intensity update rate in ms - it must be > UPDATE_RATE_MAIN
#define FLASH_SETTLING_TIME 500     // Time for flash animation to fade (flash animation only) (ms)
#define VMETER_LED_CELL_FADE_TIME 50 // Time for single LED cell to fade (v-meter mode only) (ms)

// ============================================================================
// SCALING PARAMETERS
// ============================================================================
// Controls exponential curve (1.0 = linear, >1 = compressed, <1 = expanded)
float SCALE_PIEZO_INPUT_EXPONENT = 2.0;  // Input signal scaling
float SCALE_BRIGHTNESS_DECAY_EXPONENT = 2.0;    // Intensity decay scaling

// ============================================================================
// LED ARRAY
// ============================================================================
CRGB leds[NUM_LEDS];

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================
float loudness = 0.0;       // Current loudness (0.0 - 1.0)
float loudnessHis = 0.0;    // Previous loudness value for peak detection
float intensity = 0.0;      // PLACEHOLDER FOR FUTURE DEVELOPMENT

unsigned long lastUpdate = 0;           // Main loop timing

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

/**
 * Exponential scaling function with custom curve
 * 
 * @param val: Input value to scale
 * @param minIn: Minimum input value
 * @param maxIn: Maximum input value
 * @param minOut: Minimum output value
 * @param maxOut: Maximum output value
 * @param exponent: Curve shape (1.0=linear, >1=compressed, <1=expanded)
 * @return: Scaled value
 */
float expScale(float val, float minIn, float maxIn, float minOut, float maxOut, float exponent) {
  // Constrain input to valid range
    val = constrain(val, minIn, maxIn);

    // Normalize to 0-1 range
    float normalized = (val - minIn) / (maxIn - minIn);

    // Apply exponential curve
    float scaled = pow(normalized, exponent);

    // Map to output range
    return minOut + (scaled * (maxOut - minOut));
}

/**
 * Wrap value within a range (circular)
 * 
 * @param val: Value to wrap
 * @param minVal: Minimum value
 * @param maxVal: Maximum value (inclusive)
 * @return: Wrapped value
 */
int wrap(int val, int minVal, int maxVal) {
    int range = maxVal - minVal + 1;
    return ((val - minVal) % range + range) % range + minVal;
}

// ============================================================================
// SIGNAL PROCESSING
// ============================================================================

/**
 * Scale piezoelectric sensor input with noise gate and exponential curve
 * 
 * @param rawValue: Raw ADC reading (0-1023)
 * @return: Scaled value (0.0 - 1.0)
 */
float scalePiezoInput(int rawValue) {
    // Apply noise gate
    if (rawValue <= NOISE_GATE) return 0.0;

    // Normalize above threshold (0.0 - 1.0)
    float normalized = (float)(rawValue - NOISE_GATE) / (1023.0 - NOISE_GATE);

    // Apply exponential scaling with configured exponent
    float scaledValue = expScale(normalized, 0.0, SIGNAL_MAX, 0.0, 1.0, SCALE_PIEZO_INPUT_EXPONENT);

    return constrain(scaledValue, 0.0, 1.0);
}

/**
 * Read microphone with peak-to-peak detection over sample window
 * 
 * @return: Scaled loudness value (0.0 - 1.0)
 */
float readMicLevel() {
    unsigned long startMillis = millis();
    int signalMax = 0;
    //int signalMin = 1023;

    // Collect data for sample window
    while (millis() - startMillis < SAMPLE_WINDOW) {
        int sample = analogRead(MIC_PIN);
        if (sample > signalMax) signalMax = sample;
        //if (sample < signalMin) signalMin = sample;
    }

    //int peakToPeak = signalMax; // - signalMin;
    return scalePiezoInput(signalMax);
}

// ============================================================================
// TEST FUNCTIONS
// ============================================================================

/**
 * Test piezoelectric microphone - prints raw values and blinks onboard LED
 */
void testPiezo() {
    static unsigned long lastPrint = 0;
    static int lastLedState = LOW;
    
    int rawValue = analogRead(MIC_PIN);
  
  // Print detailed values every 100ms
    if (millis() - lastPrint >= 100) {
        lastPrint = millis();

        Serial.print("Raw ADC: ");
        Serial.print(rawValue);
        Serial.print(" | Above Gate: ");
        Serial.print(rawValue > NOISE_GATE ? "YES" : "NO");
        Serial.print(" | Scaled: ");
        Serial.println(scalePiezoInput(rawValue), 3);
    }
  
    // Blink onboard LED when signal exceeds noise gate
    if (rawValue > NOISE_GATE) {
            digitalWrite(ONBOARD_LED, HIGH);
            lastLedState = HIGH;
    } else {
            // Keep LED on for 50ms after sound stops (visual persistence)
            static unsigned long ledOffTime = 0;
        if (lastLedState == HIGH) {
            ledOffTime = millis();
            lastLedState = LOW;
        }
        if (millis() - ledOffTime > 50) {
            digitalWrite(ONBOARD_LED, LOW);
        }
    }
}

/**
 * Test LED strip - cycles through colors
 */
void testLEDStrip() {
    static unsigned long lastChange = 0;
    static int testState = 0;
  
    if (millis() - lastChange >= 1000) {
        lastChange = millis();
        
        // Clear strip
        FastLED.clear();
        
        // Cycle through colors
        switch (testState) {
        case 0: // White
            fill_solid(leds, NUM_LEDS, CRGB::White);
            Serial.println("WHITE");
            break;
        case 1: // Red
            fill_solid(leds, NUM_LEDS, CRGB::Red);
            Serial.println("RED");
            break;
        case 2: // Green
            fill_solid(leds, NUM_LEDS, CRGB::Green);
            Serial.println("GREEN");
            break;
        case 3: // Blue
            fill_solid(leds, NUM_LEDS, CRGB::Blue);
            Serial.println("BLUE");
            break;
        case 4: // Off
            Serial.println("OFF");
            break;
        }
        
        FastLED.show();
        testState = (testState + 1) % 5;
    }
}

// ============================================================================
// LED ANIMATION FUNCTIONS
// ============================================================================

/**
 * Flash animation - solid color with brightness based on intensity
 * 
 * @param hue: HSV hue value (0-255)
 * @param val: Brightness value (0-255)
 */
void ledFlashAnimation(int hue, int val) {
    static const int sat = 255; // Full saturation
    
    for (int i = 0; i < NUM_LEDS; i++) {
        leds[i] = CHSV(hue, sat, val);
    }
    FastLED.show();
}

/**
 * V-meter animation - progressive color zones (green->yellow->orange->red)
 * 
 * @param signal: Signal level (0.0 - 1.0)
 * @param currentMillis: Current timestamp for fade timing
 * @return: true if animation is still active, false if complete
 */
bool ledVmeterAnimation(float signal, unsigned long currentMillis) {
 
  // LED color zone thresholds
    static const int greenLedsIdx = 24;   // 0-23: Green
    static const int yellowLedsIdx = 40;  // 24-39: Yellow
    static const int orangeLedsIdx = 52;  // 40-51: Orange
                                            // 52-59: Red
    
    static int nLedsTarget = 0;
    static int nLedsUsedPeak = 0;         // Peak LED count
    static unsigned long lastFadeUpdate = 0;
    
    
    
    // Update peak on new signal or rising level
    if (STATE == CLASH_NEW) {     
        // Calculate number of LEDs to light based on signal
        nLedsTarget = (int)(signal * NUM_LEDS); 
        // Light LEDs up to peak with color zones
        for (int i = 0; i < NUM_LEDS; i++) {
            if (i < nLedsTarget) {
                if (i < greenLedsIdx) {
                    leds[i] = CRGB::Green;
                } else if (i < yellowLedsIdx) {
                    leds[i] = CRGB::Yellow;
                } else if (i < orangeLedsIdx) {
                    leds[i] = CRGB::Orange;
                } else {
                    leds[i] = CRGB::Red;
                }
            } else {
                leds[i] = CRGB::Black;
            }
        }
        nLedsUsedPeak = nLedsTarget;
        lastFadeUpdate = currentMillis;
    }
    // Gradually fade down peak when in CLASH_ONGOING state
    if (STATE == CLASH_ONGOING) {
        if (currentMillis - lastFadeUpdate >= VMETER_LED_CELL_FADE_TIME) {
            nLedsUsedPeak--;
      
            // Clear LEDs above new peak
            if (nLedsUsedPeak >= 0) {
                leds[nLedsUsedPeak] = CRGB::Black;
                /*
                for (int i = nLedsUsedPeak; i < NUM_LEDS; i++) {
                    leds[i] = CRGB::Black;
                }
                    */
            }
            lastFadeUpdate = currentMillis; 
        }    
    }
  
    //FastLED.show();

    
  
    // Return true if animation still active (peak > 0)
    return (nLedsUsedPeak > 0);
}

    /**
 * Banger Clash animation - two lights from the edges of the strip move to the center and collide creating an explosion
 * 
 * @param signal: Signal level (0.0 - 1.0)
 * @param currentMillis: Current timestamp for fade timing
 * @return: true if animation is still active, false if complete
 */
bool bangerClashAnimation(int current_millis) {
    static int colour_1 = 0;
    static int colour_2 = 0;
    static int brightness = 20;
    static int dotPos = 0;
    static int dotSize = 1;
    static int dotSizeHis = 1;
    static int state = 3; // 0 : init, 1 : dots moving, 2 : collision, 3: end
    static int lastUpdate = 0;
    static const int startSpeed = 100; // ms per cell
    static const int endSpeed = 2; // ms per cell
    static const int xplStartSpeed = 7; // ms per cell
    static const int xplEndSpeed = 50; // ms per cell
    static const float expAcceleration = 0.5;
    static int currentSpeed = startSpeed;
    static int timer = 0;
    static int xplStep = 0;
    static const int colourWhite[3] = {45, 0, 255}; // hue, sat, bri
    static const int colourYellow[3] = {45, 255, 255}; // hue, sat, bri
    static const int colourRed[3] = {0, 255, 255}; // hue, sat, bri
    static int currentColourCore[3] = {45, 0, 255 }; 
    static int currentColourCorona[3] = {45, 255, 255 };
    const int coronaPosOffsetInit = 3;
    static int coronaPosLeft =0;
    static int coronaPosRight =0;
    static int coronaLeftDots[27] = {0};
    static int coronaRightDots[27] = {0};
    static bool doneCore = false;
    static bool doneCorona = false;
    // Debug output
    if (DEBUG) {
        Serial.print("Function call bangerClashAnimation. State ");
        Serial.println(state);
    }
    if (state == 3) { // INACTIVE
        state = 0; // new
        fill_solid(leds, NUM_LEDS, CRGB::Black);
        int rndPol = random(2);
        if (rndPol == 0) {
            rndPol = -1;
        }
        dotPos = 0;
        dotSize = 1;
        dotSizeHis = 1;
        brightness = 20;
        doneCore = false;
        doneCorona = false;
        // dot 1 on edge of led strip
        colour_1 = random(256); // dot 1
        leds[0] = CHSV(colour_1, 255, brightness);
        // dot 2 on other edge of led strip
        colour_2 = wrap(colour_1 + 128 + (random(64)*rndPol),0,255);
        leds[NUM_LEDS - 1] = CHSV(colour_2, 255, brightness);
        
        if (DEBUG) {
            Serial.print("Dot 1. Hue: ");
            Serial.println(colour_1);
            Serial.print("Dot 2. Hue: ");
            Serial.println(colour_2);
            Serial.print("Dot position: ");
            Serial.println(dotPos);
        }

        currentSpeed = startSpeed; // speed is calculated as time for the dots to move to their next position
        timer = 0;
        state = 1; //dots moving
        lastUpdate = current_millis;

    } else { // ACTIVE
        int deltaTime = current_millis - lastUpdate;
        timer += deltaTime;
        if (state == 1) {       // moving  
            
            float movement = (float)(timer)/(float)(currentSpeed);
            if (movement >= 0.75) { // dot moves to next cell 
                // switch off the leds from previous step 
                for (int cnt = 0; cnt < dotSize; cnt++) {           
                    leds[dotPos - cnt] = CRGB::Black;
                    leds[NUM_LEDS - dotPos - 1 + cnt] = CRGB::Black;
                }
                // calculate 
                dotSize = constrain((int)movement,1,5);
                dotPos += (int)round(movement+0.5);
                if (dotPos  >= (NUM_LEDS / 2) - 1) { // dots collide
                    state = 2; // collision
                    
                    coronaPosLeft = (NUM_LEDS / 2) - coronaPosOffsetInit - 1;
                    coronaPosRight = (NUM_LEDS / 2) + coronaPosOffsetInit;
                    if (DEBUG) {    
                        Serial.print("Corona LEFT dot pos init: ");
                        Serial.println(coronaPosLeft);
                        Serial.print("Corona RIGHT dot pos init: ");
                        Serial.println(coronaPosRight);
                    }
                    xplStep = 0;
                    currentSpeed = xplStartSpeed;
                } else {
                    brightness = (int)expScale((float)dotPos,0.0,29.0,(float)20,(float)300,4.0);
                    //brightness = constrain(brightness,0,255);
                    int brightnessTail = brightness;
                    for (int cnt = 0; cnt < dotSize; cnt++) {   
                        brightnessTail = constrain(int( (float) brightnessTail * (1.0 - ( (float)cnt / 10.0) )),
                          0,255);     
                        leds[dotPos - cnt] = CHSV(colour_1, 255, brightnessTail);
                        leds[NUM_LEDS - dotPos - 1 + cnt] = CHSV(colour_2, 255, brightnessTail);
                    }
                    
                    currentSpeed = (int)expScale((float)dotPos,0.0,29.0,(float)startSpeed,(float)endSpeed,expAcceleration);
                    if (DEBUG) {
                        Serial.print("Dot size: ");
                        Serial.print(dotSize);
                        Serial.print(", position: ");
                        Serial.print(dotPos);
                        Serial.print(". Speed: ");
                        Serial.println(currentSpeed);

                        if (state == 2) {
                            Serial.println("Switching to state 2");
                        }
                    }
                    timer = 0;
                }            
            }
            
        }
        if (state == 2) { // collision
            
            const int centerPos[2] =  { (NUM_LEDS / 2) - 1, (NUM_LEDS / 2) };
            

            memcpy(currentColourCore, colourWhite, sizeof(colourWhite));
            currentColourCore[2] = brightness ; // will transition to bright white
            memcpy(currentColourCorona, colourYellow, sizeof(colourYellow));

            // CORE
            if (xplStep < 4) { // turn to bright white (explosion first part)
                currentColourCore[2] = map(xplStep,0,3,brightness,255);   
                for (int posOffset = 0; posOffset < 3; posOffset++) {
                    leds[centerPos[0] - posOffset] = CHSV(currentColourCore[0], currentColourCore[1], currentColourCore[2]);
                    leds[centerPos[1] + posOffset] = CHSV(currentColourCore[0], currentColourCore[1], currentColourCore[2]);
                }
            } else if (xplStep < 10) {
                for (int posOffset = 1; posOffset < 3; posOffset++) {
                    currentColourCorona[1] = map(xplStep,4,9,colourWhite[1],colourYellow[1]); // sat interpolation
                    leds[centerPos[0] - posOffset] = CHSV(currentColourCorona[0], currentColourCorona[1], currentColourCorona[2]);
                    leds[centerPos[1] + posOffset] = CHSV(currentColourCorona[0], currentColourCorona[1], currentColourCorona[2]);
                }
            } else if (xplStep < 16) {
                leds[centerPos[0]] = CHSV(currentColourCore[0], currentColourCore[1], map(xplStep,10,15,255,0));; // bri interpolation
                leds[centerPos[1]] = leds[centerPos[0]];
                for (int posOffset = 1; posOffset < 3; posOffset++) {
                    currentColourCorona[0] = map(xplStep,10,15,colourYellow[0],colourRed[0]); // hue interpolation
                    currentColourCorona[2] = map(xplStep,10,15,255,0); // bri interpolation
                    leds[centerPos[0] - posOffset] = CHSV(currentColourCorona[0], currentColourCorona[1], currentColourCorona[2]);
                    leds[centerPos[1] + posOffset] = CHSV(currentColourCorona[0], currentColourCorona[1], currentColourCorona[2]);
                }
            } else {
                doneCore = true;
                Serial.println("END OF CORE XPLS");
            }

            if (xplStep > 0) {
                if (xplStep < 8) {
                    for (int idx = (xplStep-1)*4; idx < (xplStep-1)*4 + 4; idx++) {
                        if (idx < 27) {
                            coronaLeftDots[idx] = 255;
                            coronaRightDots[idx] = 255;
                        }
                    }
                } 
                
                if (xplStep > 1) {  
                    for (int idx = 0; idx < (xplStep-1)*4; idx++) {
                        if (idx < 27) {
                            if (coronaLeftDots[idx] > 0) {
                                coronaLeftDots[idx] -= 256/16;
                                coronaRightDots[idx] -= 256/16;
                            } 
                        }
                    }
                }
                if ( (coronaLeftDots[26] <= 0)
                &&  (xplStep > 8) ) {
                    doneCorona = true;
                }
                
                Serial.println("~~~~~~~~~~~~~~~~~~~~~~~");
                Serial.print("STEP ");
                Serial.println(xplStep );
                Serial.print("brightness of last led: ");
                Serial.println(coronaLeftDots[26]);
                Serial.println("~~~~~~~~~~~~~~~~~~~~~~~");

                for (int idx = 0; idx < 27; idx++) {
                    leds[26 - idx] = CHSV(0, 255, constrain(coronaLeftDots[idx],0,255));
                    leds[33 + idx] = CHSV(0, 255, constrain(coronaRightDots[idx],0,255));
                }

                if ( (doneCore) && (doneCorona) ) {
                    state = 3;
                    Serial.println("END OF EXPLOSION");
                }
            }
            /*
            //RED CORONA
            float movement = (float)(timer)/(float)(currentSpeed);
            if (movement >= 0.75) { // dot moves to next cell 
                
                // calculate 
                dotSize = constrain((int)movement,1,30);
                if (DEBUG) {    
                    Serial.print("CORONA LOOP. Corona dot size due to speed: ");
                    Serial.println(dotSize);
                }
                coronaPosLeft -= dotSize; //(int)round(movement+0.5); // goes down as it goes closer to edge
                coronaPosRight += dotSize; //(int)round(movement+0.5);
                if (coronaPosLeft + dotSize  < 0) {
                    state = 3; // end
                    fill_solid(leds, NUM_LEDS, CRGB::Black);
                } else {
                    brightness = 255;
                    //brightness = (int)expScale((float)dotPos,0.0,29.0,(float)20,(float)300,4.0);
                    //brightness = constrain(brightness,0,255);
                    //int brightnessTail = brightness;
                    
                    for (int cnt = 1; cnt <= dotSize; cnt++) {   
                        //brightnessTail = constrain(int( (float) brightnessTail * (1.0 - ( (float)cnt / 10.0) )),
                        //  0,255); 
                        int pos = coronaPosLeft + cnt;
                        if ( (pos > dotSize * -1) && (pos < centerPos[0] - coronaPosOffsetInit)) {
                            if (DEBUG) {    
                                Serial.print("Left size. Dot @pos: ");
                                Serial.println(pos);
                            }
                            leds[pos] = CHSV(0, 255, brightness);
                        }
                        
                        pos = constrain(coronaPosRight - cnt, centerPos[1] + coronaPosOffsetInit, 100);
                        if ( (pos > NUM_LEDS + dotSize ) && (pos > centerPos[1] + coronaPosOffsetInit)) {                            
                            if (DEBUG) {    
                                Serial.print("Right size. Dot @pos: ");
                                Serial.println(pos);
                            }
                            leds[pos] = CHSV(0, 255, brightness);
                        }
                    }
                    
                    currentSpeed = (int)expScale((float)dotPos,0.0,29.0,(float)xplStartSpeed,(float)xplEndSpeed,0.25);
                    if (DEBUG) {
                       
                        Serial.print(". Speed: ");
                        Serial.println(currentSpeed);

                        Serial.println("++++++++++++++++++++++++++++++++++++");
                        Serial.println("++++++++++++++++++++++++++++++++++++");
                    }
                    
                }  
                timer = 0;          
            }
            */

            xplStep++;
        }
    }

    lastUpdate = current_millis;
    FastLED.show();
    return (state != 3);
}
// ============================================================================
// SETUP
// ============================================================================

void setup() {
    Serial.begin(9600);
  
    // Initialize onboard LED for debugging
    pinMode(ONBOARD_LED, OUTPUT);
    digitalWrite(ONBOARD_LED, LOW);
    
    // Initialize FastLED library
    FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(255);
    FastLED.clear();
    FastLED.show();
    
    // Set analog reference to default (5V on Nano)
    analogReference(DEFAULT);
  
    // Print test mode information
    if (TEST_MODE == TEST_PIEZO) {
        Serial.println("=== PIEZO TEST MODE ENABLED ===");
        Serial.println("Tap the piezo to see readings");
        Serial.println("Onboard LED will light when signal exceeds noise gate");
        Serial.print("Noise Gate: ");
        Serial.println(NOISE_GATE);
        Serial.println("================================");
    } else if (TEST_MODE == TEST_LED) {
        Serial.println("=== LED STRIP TEST MODE ENABLED ===");
        Serial.println("LEDs will cycle: WHITE -> RED -> GREEN -> BLUE -> OFF");
        Serial.print("Number of LEDs: ");
        Serial.println(NUM_LEDS);
        Serial.println("====================================");
    } else if (TEST_MODE == TEST_BANGERCLASH) {
        Serial.println("=== LED BANGERCLASH ANIMATION TEST MODE ENABLED ===");
        Serial.print("Number of LEDs: ");
        Serial.println(NUM_LEDS);
        Serial.println("====================================");
    } else {
        Serial.println("=== NORMAL MODE ===");
        Serial.print("Clash Animation: ");
        Serial.println(CLASH_ANIMATION == FLASH ? "FLASH" : "VMETER");
        Serial.println("===================");
    }
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
    // ========================================
    // TEST MODE
    // ========================================
    if (TEST_MODE == TEST_PIEZO) {
        testPiezo();
        return;
    }
  
    if (TEST_MODE == TEST_LED) {
        testLEDStrip();
        return;
    }

    if (TEST_MODE == TEST_BANGERCLASH) {
        bangerClashAnimation(millis());
    }
    
    if (TEST_MODE == NO_TEST) {
        // ========================================
        // NORMAL OPERATION MODE
        // ========================================
        
        unsigned long currentMillis = millis();
        static unsigned long lastUpdateLed = 0;
        static int clashTimer = 0;
        static float flashAnimationBrightness_f = 0.0; // Range 0.0 to 255.0
        static int flashAnimationHue = 0;              // Range 0 to 255
        static bool isActiveClashAnimation = false;
        
        // Main update rate throttling
        if (currentMillis - lastUpdate >= UPDATE_RATE_MAIN) {
            
            // Read and process audio signal
            loudness = readMicLevel();
                
            // Debug output
            if (DEBUG) {
                Serial.print("Loudness: ");
                Serial.print(loudness, 3);
                Serial.print(" | State: ");
                Serial.println(STATE);
            }
        
            // main/animation update at separate rate (main must me the fastest)
            if (currentMillis - lastUpdateLed >= UPDATE_RATE_LED) {
        
                // ========================================
                // CLASH ANIMATIONS
                // ========================================
                // Two types:
                // - FLASH: Random color flash with brightness proportional to loudness
                // - VMETER: Green->yellow->orange->red progressive meter
            
                if (CLASH_ANIMATION != NO_ANIMATION) {
                    // Check if there's active audio signal
                    if (!isActiveClashAnimation) {
                        STATE = OFF;
                        //FastLED.clear();
                        //FastLED.show();
                        isActiveClashAnimation = (loudness > 0.0);
                    }
            
                    if (isActiveClashAnimation) {

                        // Detect new loudness peak (trigger animation reset)
                        if (loudness > loudnessHis) {
                            STATE = CLASH_NEW;
                        } else {
                            // No new peak - fade out / release
                            STATE = CLASH_ONGOING;
                        }
                    
                        // CLASH ANIMATIONS
                        if (CLASH_ANIMATION == FLASH) {
                            int flashAnimationBrightness = 0;
                            if (STATE == CLASH_NEW) {
                                // Initialize flash animation
                                clashTimer = FLASH_SETTLING_TIME;
                                flashAnimationBrightness_f = loudness * 255.0;
                                flashAnimationBrightness = (int)(flashAnimationBrightness_f);
                                // Pick random hue, avoiding similar colors
                                flashAnimationHue = wrap(flashAnimationHue + random(63, 191), 0, 255);
                            }    
                            if (STATE == CLASH_ONGOING) {
                                clashTimer -= UPDATE_RATE_LED;
                                
                                if (clashTimer <= 0) {
                                    // End of flash animation
                                    fill_solid(leds, NUM_LEDS, CRGB::Black);
                                    FastLED.show();
                                    isActiveClashAnimation = false;
                                } else {
                                    // Calculate fading brightness
                                    flashAnimationBrightness = (int)(
                                        flashAnimationBrightness_f * 
                                        expScale((float)clashTimer, 0.0, (float)FLASH_SETTLING_TIME, 
                                                0.0, 1.0, SCALE_BRIGHTNESS_DECAY_EXPONENT)
                                    );
                                }
                                ledFlashAnimation(flashAnimationHue, flashAnimationBrightness);
                            }
                        }
                        
                        if (CLASH_ANIMATION == VMETER) {
                            // V-meter returns false when animation complete
                            isActiveClashAnimation = ledVmeterAnimation(loudness, currentMillis);
                            FastLED.show();    
                        }
                        
                    }
                    
                    lastUpdateLed = currentMillis;
                    // Store current loudness for next peak detection
                    loudnessHis = loudness;
                }
                
            }
            
          

            lastUpdate = currentMillis;
        }
    }
  
}