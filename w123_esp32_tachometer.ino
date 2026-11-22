// ================================================================
// Mercedes W123 250 (6-Cylinder) ESP32-C3 Super Mini Tachometer
// Hardware: 4N35 Optocoupler on GPIO 3, external 5.1K pull-up to 3.3V
// Arduino IDE: set "USB CDC On Boot: Enabled" so Serial works over USB.
//
// Method: trimmed mean of the last 14 pulse intervals. Sort them, throw away
// the lowest and highest third, average the middle. Outliers (stray noise
// pulses) land at the ends and are discarded; the average of the middle
// smooths out cylinder-to-cylinder differences. No thresholds to tune.
// ================================================================

const int tachPin = 3;            // Connected to 4N35 Pin 5 (Collector)
const float pulsesPerRev = 3.0;   // 6-cylinder, 4-stroke = 3 sparks per crank revolution

// Ignore pulses closer together than this (glitches).
// 3,000 us = 333 Hz = ~6,666 RPM, above the W123 redline.
const unsigned long debounceUs = 3000;

// Engine counts as stopped if no pulse for this long (0.5 s).
const unsigned long timeoutUs = 500000;

// Window of recent intervals. Trimming a third from each end discards up to
// 4 bad intervals of 14 on the low side, which covers about two stray pulses
// (each stray pulse splits a real interval into two short ones).
const int numSamples = 14;
const int minSamples = 6;          // need this many intervals before showing RPM

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

void setup() {
  Serial.begin(115200);

  // Internal pull-up stays on, in parallel with the external 5.1K.
  pinMode(tachPin, INPUT_PULLUP);

  // Trigger when the 4N35 pulls GPIO 3 to GND
  attachInterrupt(digitalPinToInterrupt(tachPin), onTachPulse, FALLING);

  Serial.println("Mercedes W123 250 Tachometer Ready...");
}

void loop() {
  delay(250);  // Refresh 4 times per second

  unsigned long buf[numSamples];
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
  }
  portEXIT_CRITICAL(&mux);

  if (stopped) {
    Serial.println("Frequency: 0.0 Hz | RPM: 0");
    return;
  }
  if (valid < minSamples) {
    Serial.println("Frequency: -- | RPM: -- (collecting)");
    return;
  }

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

  // Trimmed mean: drop the lowest and highest third, average the middle
  int trim = valid / 3;
  unsigned long sum = 0;
  for (int i = trim; i < valid - trim; i++) sum += buf[i];
  float avgInterval = (float)sum / (float)(valid - 2 * trim);

  float hz = 1000000.0f / avgInterval;
  float rpmExact = hz * 60.0f / pulsesPerRev;
  int rpm = (int)roundf(rpmExact / displayStep) * displayStep;

  Serial.printf("Frequency: %.1f Hz | RPM: %d\n", hz, rpm);
}