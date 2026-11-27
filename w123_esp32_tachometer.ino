// ================================================================
// Mercedes W123 250 (6-Cylinder) ESP32-C3 Super Mini Tachometer
// Hardware: 4N35 Optocoupler on GPIO 3, external 5.1K pull-up to 3.3V
// Arduino IDE: set "USB CDC On Boot: Enabled" so Serial works over USB.
// Display: 4-digit 7-segment HT16K33 on I2C, SDA = GPIO 6, SCL = GPIO 7.
// Library: "HT16K33" by Rob Tillaart (Arduino Library Manager).
//
// Method (useTrimmedMean = true): trimmed mean of the last numSamples pulse
// intervals. Sort them, throw away 1/trimDivisor at the low end and the same
// at the high end, average the middle. Outliers (stray noise pulses) land at
// the ends and are discarded; the average of the middle smooths out
// cylinder-to-cylinder differences. No thresholds to tune.
//
// Method (useTrimmedMean = false): no filtering, only the newest interval.
// ================================================================

#include <Wire.h>
#include <HT16K33.h>

// ======================== FILTER SETTINGS ========================
// true  = trimmed mean (settings below)
// false = raw reading from the newest pulse interval only
const bool useTrimmedMean = true;

// Number of recent intervals in the window. More = steadier but slower.
const int numSamples = 12;

// Fraction dropped from EACH end: 3 = a third, 4 = a fourth.
// 0 = drop nothing (plain average). 1 and 2 are not allowed.
const int trimDivisor = 4;

// =================================================================

// Intervals needed before showing RPM (trimmed mean only): a full window,
// so the first reading is filtered as well as every later one.
const int minSamples = numSamples;

const int sdaPin = 6;
const int sclPin = 7;

HT16K33 display(0x70);   // I2C address with A0, A1, A2 pads open

const int tachPin = 3;            // Connected to 4N35 Pin 5 (Collector)
const float pulsesPerRev = 3.0;   // 6-cylinder, 4-stroke = 3 sparks per crank revolution

// Ignore pulses closer together than this (glitches).
// 3,000 us = 333 Hz = ~6,666 RPM, above the W123 redline.
const unsigned long debounceUs = 3000;

// Engine counts as stopped if no pulse for this long (0.5 s).
const unsigned long timeoutUs = 500000;

// Round the displayed RPM to this step (hides 583 vs 584 flicker).
const int displayStep = 5;

volatile unsigned long intervals[numSamples] = {0};
volatile int sampleIndex = 0;
volatile unsigned long lastPulseTime = 0;   // 0 = no baseline yet

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTachPulse() {
  unsigned long now = micros();

  portENTER_CRITICAL_ISR(&mux);
  if (lastPulseTime == 0) {
    // First pulse after start/reset: baseline only, no interval yet
    lastPulseTime = (now != 0) ? now : 1;
  } else {
    unsigned long interval = now - lastPulseTime;
    if (interval > debounceUs) {
      intervals[sampleIndex] = interval;
      sampleIndex = (sampleIndex + 1) % numSamples;
      lastPulseTime = now;
    }
  }
  portEXIT_CRITICAL_ISR(&mux);
}

void showRpm(int rpm) {
  display.displayInt(rpm);     // right-aligned, no leading zeros
}

void showDashes() {
  uint8_t dashes[4] = {HT16K33_MINUS, HT16K33_MINUS, HT16K33_MINUS, HT16K33_MINUS};
  display.display(dashes);
}

void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);

  Wire.begin(sdaPin, sclPin);  // must come before display.begin()
  if (!display.begin()) Serial.println("Display not found");
  display.setBrightness(15);   // 0 (dim) to 15 (bright)
  display.displayColon(false); // make sure the colon is off

  // Internal pull-up stays on, in parallel with the external 5.1K.
  pinMode(tachPin, INPUT_PULLUP);

  // Trigger when the 4N35 pulls GPIO 3 to GND
  attachInterrupt(digitalPinToInterrupt(tachPin), onTachPulse, FALLING);

  Serial.println("Mercedes W123 250 Tachometer Ready...");
}

void loop() {
  delay(250);  // Refresh 4 times per second

  unsigned long buf[numSamples];
  unsigned long newest = 0;
  int valid = 0;
  bool stopped;

  portENTER_CRITICAL(&mux);
  stopped = (lastPulseTime == 0) || ((micros() - lastPulseTime) > timeoutUs);
  if (stopped) {
    // Reset so the next start begins fresh
    for (int i = 0; i < numSamples; i++) intervals[i] = 0;
    sampleIndex = 0;
    lastPulseTime = 0;
  } else {
    for (int i = 0; i < numSamples; i++) {
      if (intervals[i] > 0) buf[valid++] = intervals[i];
    }
    // The newest interval sits just before the next write position
    newest = intervals[(sampleIndex + numSamples - 1) % numSamples];
  }
  portEXIT_CRITICAL(&mux);

  static bool blinkOn = false;
  blinkOn = !blinkOn;                        // flips every pass (250 ms)
  display.displayColon(stopped && blinkOn);  // blink = waiting, off = pulses

  if (stopped) {
    Serial.println("Frequency: 0.0 Hz | RPM: 0");
    showRpm(0);
    return;
  }
  const int needed = useTrimmedMean ? minSamples : 1;
  if (valid < needed) {
    Serial.println("Frequency: -- | RPM: -- (collecting)");
    showDashes();
    return;
  }

  float avgInterval;
  if (useTrimmedMean) {
    // Sort the window (insertion sort)
    for (int i = 1; i < valid; i++) {
      unsigned long key = buf[i];
      int j = i - 1;
      while (j >= 0 && buf[j] > key) {
        buf[j + 1] = buf[j];
        j--;
      }
      buf[j + 1] = key;
    }

    // Trimmed mean: drop the lowest and highest part, average the middle
    int trim = (trimDivisor > 0) ? valid / trimDivisor : 0;
    unsigned long sum = 0;
    for (int i = trim; i < valid - trim; i++) sum += buf[i];
    avgInterval = (float)sum / (float)(valid - 2 * trim);
  } else {
    avgInterval = (float)newest;
  }

  float hz = 1000000.0f / avgInterval;
  float rpmExact = hz * 60.0f / pulsesPerRev;
  int rpm = (int)roundf(rpmExact / displayStep) * displayStep;

  Serial.printf("Frequency: %.1f Hz | RPM: %d\n", hz, rpm);
  showRpm(rpm);
}