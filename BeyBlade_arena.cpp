#include <FastLED.h>

// Pin definitions
#define LED_PIN 5          // NeoPixel data pin (can use D2-D12)
#define MIC_PIN A1         // Piezo mic on analog pin A1
#define NUM_LEDS 60
#define ONBOARD_LED 13     // Onboard LED pin

// Test modes
#define NO_TEST = -1
#define TEST_PIEZO = 0   // Set to true to enable piezo testing mode
#define TEST_LED = 1     // Set to true to enable LED strip testing mode
int TEST_MODE = NO_TEST; // TEST_PIEZO TEST_LED TEST_LOUDNESS

// Clash animation
#define NO_ANIMATION = -1
#define VMETER = 0
#define FLASH = 1;
int CLASH_ANIMATION = 0; 
// led states
#define OFF = 0;
#define CLASH_NEW = 1;
#define CLASH_ONGOING = 2;
#define PATTERN = 3;
int STATE = OFF;
// Audio parameters
#define NOISE_GATE 50      // Threshold value (0-1023 for Arduino 10-bit ADC)
#define SIGNAL_MAX 0.9 // loudness will be rescaled to this max level
#define SAMPLE_WINDOW 30   // Sample window in ms for peak detection
#define UPDATE_RATE_MAIN 25     

//Intensity parameters
#define UPDATE_RATE_INTENSITY 50 
#define FLASH_SETTLING_TIME 500 // time for flash led animation to settle from peak to off 
#define VMETER_SETTLING_TIME 1500 // time for flash led animation to settle from peak to off 

// Scaling control
float SCALE_PIEZO_INPUT_EXPONENT = 2.0;  // Controls logarithmic curve (1.0 = linear, >1 = compressed, <1 = expanded)
float SCALE_INTENSITY_EXPONENT = 2.0;
// Pattern parameters
#define PATTERN_SPEED 100  // Movement speed in ms

// LED array
CRGB leds[NUM_LEDS];

// Variables
float loudness = 0.0; // as detected by the piezoelectric mic
float intensity = 0.0; // this climb up to loudness peak when loudness> brightness else it slowly fades down

unsigned long lastUpdateMain = 0;
unsigned long lastUpdateIntensity = 0;
unsigned long lastPattern = 0;
int patternOffset = 0;


// UTILITIES

/**
 * Basic exponential scaling function
 * exponent: 
 *   1.0 = linear mapping (same as map() function)
 *   0.5 = square root (compress high values)
 *   2.0 = square (expand high values)
 */
float expScale(float val, float minIn, float maxIn, float minOut, float maxOut, float exponent) {
  // Constrain input
  val = constrain(val, minIn, maxIn);
  
  // Normalize to 0-1 range
  float normalized = (val - minIn) / (maxIn - minIn);
  
  // Apply exponential curve
  float scaled = pow(normalized, exponent);
  
  // Map to output range
  return minOut + (scaled * (maxOut - minOut));
}

int wrap(int val, int minVal, int maxVal) {
  return ((val - minVal) % (maxVal - minVal + 1) + (maxVal - minVal + 1)) % (maxVal - minVal + 1) + minVal;
}

// BESPOKEN SCALING FOR SIGNAL COMING FROM PIEZOELECTRIC
float scalePiezoInput(int rawValue) {
  if (rawValue <= NOISE_GATE) return 0.0;
  
  // Normalize above threshold
  float normalized = (float)(rawValue - NOISE_GATE) / (1023.0 - NOISE_GATE);
  
  // Apply power curve controlled by SCALE_PIEZO_INPUT_EXPONENT
  // SCALE_PIEZO_INPUT_EXPONENT = 1.0: linear
  // SCALE_PIEZO_INPUT_EXPONENT > 1.0: compressed (more gradual, like log)
  // SCALE_PIEZO_INPUT_EXPONENT < 1.0: expanded (more sensitive to low levels)
  float scaledValue = expScale(normalized, 0.0, SIGNAL_MAX, 0.0, SIGNAL_MAX, SCALE_PIEZO_INPUT_EXPONENT);
  
  return constrain(scaledValue, 0.0, 1.0);
}

// TEST FUNCTIONS

// Test function for piezoelectric microphone
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

// Read microphone with peak detection
float readMicLevel() {
  unsigned long startMillis = millis();
  int peakToPeak = 0;
  int signalMax = 0;
  int signalMin = 1023;
  
  // Collect data for sample window
  while (millis() - startMillis < SAMPLE_WINDOW) {
    int sample = analogRead(MIC_PIN);
    if (sample > signalMax) signalMax = sample;
    if (sample < signalMin) signalMin = sample;
  }
  
  peakToPeak = signalMax - signalMin;
  return scalePiezoInput(peakToPeak);
}



// LED ANIMATION FUNCTIONS
//Make clash animation
void ledClashAnimation(hue,val) {
  static int hue = 0;
  static int sat = 255;
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CHSV(hue, sat, val);
  }
}

// VU meter color calculation (green -> yellow -> orange -> red)
void ledVmeterAnimation(float signal) { //signal range 0.0 to 1.0
  int greenLedsIdx = 24;
  int yellowLedsIdx = 40;
  int orangeLedsIdx = 52;
  int nLedsUsed = int(float(NUM_LEDS)*signal);

  for (i = 0; i < NUM_LEDS; i++) {
    if (i < nLedsUsed) {
      if (i < greenLedsIdx) {
        leds[i] = CRGB::Green;
      } else if ( i < yellowLedsIdx) {
        leds[i] = CRGB::Yellow;
      } else if ( i < orangeLedsIdx) {
        leds[i] = CRGB::Orange;
      } else {
        leds[i] = CRGB::Red;
      }
    } else {
      leds[i] = CRGB::Black;
    }
  } 
  FastLED.show();
}

void setup() {
  Serial.begin(9600);
  
  // Initialize onboard LED
  pinMode(ONBOARD_LED, OUTPUT);
  digitalWrite(ONBOARD_LED, LOW);
  
  // Initialize FastLED
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(255);
  FastLED.clear();
  FastLED.show();
  
  // Set analog reference to default (5V)
  analogReference(DEFAULT);
  // TEST_MODE = NO_TEST; // TEST_PIEZO TEST_LED TEST_LOUDNESS
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
  } else if (TEST_MODE == TEST_LOUDNESS) {
    Serial.println("=== LOUDNESS TEST MODE ENABLED ===");
    Serial.println("LED V-Meter will react to signal coming from piezo");
    Serial.println("====================================");
  }
}

void loop() {
  // Run test mode if enabled
  if (TEST_MODE == TEST_PIEZO) {
    testPiezo();
    return;
  }
  
  if (TEST_MODE == TEST_LED) {
    testLEDStrip();
    return;
  }

  
  unsigned long currentMillis = millis();
  static int clashTimer = 0;
  static float flashAnimationBrightness_f = 0.0; // range 0.0 to 255.0
  static int flashAnimationHue = 0; // range 0 to 255

  if (currentMillis - lastUpdateMain >= UPDATE_RATE_MAIN) {
    lastUpdateMain = currentMillis;
    
    // Read and process audio
    loudness = readMicLevel();

    // Debug output
    Serial.print("Intensity: ");
    Serial.println(loudness);
  }
  
  // Update intensity
  if (currentMillis - lastUpdateIntensity >= UPDATE_RATE_INTENSITY) {
    lastUpdateIntensity = currentMillis;

    // CLASH ANIMATIONS
    if (CLASH_ANIMATION != NO_ANIMATION) {
      // NEW PEAK => NEW ANIMATION
      // check if new loudness peak has been detected
      if (loudness > intensity) { // new clash
        STATE = CLASH_NEW;
        intensity = loudness;
        if (CLASH_ANIMATION == FLASH) {
          clashTimer = FLASH_SETTLING_TIME;
          flashAnimationBrightness_f = intensity * 255.0;
          flashAnimationHue = wrap(flashAnimationHue + random(63,191), 0, 255); // pick random colour
        }
        if (CLASH_ANIMATION == VMETER) {
          clashTimer = VMETER_SETTLING_TIME;
        }

      } else { //no new flash
        STATE = CLASH_ONGOING;
        clashTimer -= UPDATE_RATE_INTENSITY;
        if (clashTimer <= 0) { // end of clash animation        
          flashAnimationBrightness = 0;
          STATE = OFF;
        // RELEASE
        if (STATE == CLASH) { 

                                 
            } else {   
              int flashAnimationBrightness = int ( flashAnimationBrightness_f * expScale(float(clashTimer), 0.0, float(FLASH_SETTLING_TIME), 0.0, 1.0, SCALE_INTENSITY_EXPONENT) );
            }
            ledClashAnimation(flashAnimationHue, flashAnimationBrightness);  
          }

      }
    
          
        }
      }

      if (CLASH_ANIMATION == VMETER) { //FINO A QUI
        ledVmeterAnimation(loudness)
      }


  }
}