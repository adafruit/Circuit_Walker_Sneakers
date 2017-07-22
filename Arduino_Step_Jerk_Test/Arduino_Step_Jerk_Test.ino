// Circuit Playground Circuit Walker Sneakers - Step Jerk Detection Test
// Author: Tony DiCola
// License: MIT License
//
// This sketch uses ideas from an adaptive step detection algorithm to
// turn Circuit Playground into a pedometer/step detector.  When the board
// detects a step with the accelerometer it will turn the LEDs on red for
// a brief period of time (~half a second).  The sketch uses the non-adaptive
// step jerk theshold ('sjt') approach to detect steps as mentioned in:
//   https://github.com/danielmurray/adaptiv
//
// You might need to tweak the THRESHOLD define below to your walking
// style.  If it's too sensitive and flashing red too often try increasing
// the threshold slightly above 0.15, or if it isn't sensitive enough try
// decreasing the threshold below 0.15
#include <Adafruit_CircuitPlayground.h>

#include "IIRFilter.h"


// Configuration values:
#define  THRESHOLD           0.15    // Threshold for a 'dip' or 'jerk' in movement
                                     // to be considered a step.  Increase this value
                                     // to make the detection less sensitive and decrease
                                     // to make more sensitive.

#define  GRAVITY             1.0     // Baseline accelerometer magnitude to detect zero
                                     // crossings in the filtered data.  Since the units
                                     // are gravities and 1.0G is expected when between
                                     // peak/trough keep this at 1.0 or very close to it.  

#define  SAMPLE_RATE_HZ      50.0    // How quickly to sample the accelerometer.
                                     // Note that the filter is generated using
                                     // an assumed 50hz sample rate so if you change
                                     // this value you also need to regenerate and
                                     // change the filter coefficients below.

#define  STEP_FLASH_MS       500     // Amount of time in milliseconds to turn on
                                     // the NeoPixels red when a step is detected.

// Coefficients for IIR Butterworth low pass filter with cutoff of 3.6667 hz.
// These were generated from SciPy's Butterworth filter generation function.
// You can see a Jupyter notebook that generated them here:
//   https://gist.github.com/tdicola/a7809bfa839b587f64e46e9280741f1a
#define FILTER_A       { 1.0, -2.08554010137, 1.54484341043, -0.394448994245 }
#define FILTER_B       { 0.00810678935123, 0.0243203680537, 0.0243203680537, 0.00810678935123 }



// Global state (don't change this):
const uint32_t periodMS = int(1.0/SAMPLE_RATE_HZ*1000.0);     // Period of time (in milliseconds) between accelerometer samples.
IIRFilter<4,4> filter((float[])FILTER_A, (float[])FILTER_B);  // The IIR filter that will use the coefficients defined above.
uint32_t lastMS = 0;           // Time of last loop iteration.
uint32_t lastSampleMS = 0;     // Time of last accelerometer sample.
int pixelClearMS = 0;      // Amount of time left before clearing the NeoPixels and turning them off.
float lastTrough = GRAVITY;
float lastPeak = GRAVITY;
enum JerkState { ZERO, PEAK, TROUGH };
JerkState currentState = ZERO;


void setup() {
  // Initialize serial port and turn off all the NeoPixels.
  Serial.begin(115200);
  CircuitPlayground.begin(255);
  CircuitPlayground.clearPixels();
  // Bootstrap main loop elapsed time.
  lastMS = millis();
}

void loop() {
  // Compute how much time ellapsed since the last loop iteration.
  uint32_t currentMS = millis();
  uint32_t deltaMS = currentMS - lastMS;

  // Check if enough time has elapsed to turn off the NeoPixels if they were on.
  if (pixelClearMS > 0) {
    pixelClearMS -= deltaMS;
    if (pixelClearMS <= 0) {
      pixelClearMS = 0;
      CircuitPlayground.clearPixels();
    }
  }

  // Check if enough time has elapsed to take a new accelerometer reading.
  if (currentMS >= (lastSampleMS + periodMS)) {
    // Grab a new accelerometer reading and compute the magnitude of the
    // acceleration vector defined by the X, Y, Z axis.  This is just computing:
    // sqrt(x^2 + y^2 + z^2)
    CircuitPlayground.lis.read();
    float mag = sqrt(pow(CircuitPlayground.lis.x_g, 2.0) +
                     pow(CircuitPlayground.lis.y_g, 2.0) +
                     pow(CircuitPlayground.lis.z_g, 2.0));

    // Apply the low pass filter to the magnitude reading.
    float filtered = filter.filter(mag);

    // Print the raw magnitude and filtered magnitude if the slide switch is in
    // the +/right position.  This is handy to temporarily stop the plotter output
    // and examine the accelerometer data.
    if (CircuitPlayground.slideSwitch()) {
      Serial.print(mag);
      Serial.print(',');
      Serial.print(filtered);
      Serial.println();
    }

    // Compute step 'jerk', or the strength of a movement from up to down acceleration
    // and vice-versa.  Once the jerk exceeds a threshold then a step is detected.
    // First find the new jerk state, either a peak (acceleration above baseline gravity)
    // or a trough (acceleration below baseline gravity).
    JerkState newState = ZERO;
    if (mag < GRAVITY) {
      newState = TROUGH;
    }
    else if (mag > GRAVITY) {
      newState = PEAK;
    }
    if (newState == currentState) {
      // No change in peak/trough state so just keep searching for the min trough value
      // and maximum peak value.
      if (newState == TROUGH) {
        lastTrough = min(lastTrough, mag);
      }
      else if (newState == PEAK) {
        lastPeak = max(lastPeak, mag);
      }
    }
    else {
      // Changed state from peak to trough or vice/versa.  Compute the 'jerk' or distance
      // between trough and peak when we're coming out of a trough and into a peak.
      if (newState == PEAK) {
        float jerk = lastPeak - lastTrough;
        //Serial.println(jerk, 6);
        if (jerk > THRESHOLD) {
          // Jerk exceeded the threshold, a step was detected!
          // Turn on the lights.
          pixelClearMS = STEP_FLASH_MS;
          for (int i=0; i<10; ++i) {
            CircuitPlayground.strip.setPixelColor(i, 255, 0, 0);
          }
          CircuitPlayground.strip.show();
        }
        // Be sure to reset the peak and trough min/max values for next state change.
        lastPeak = mag;
        lastTrough = GRAVITY;
      }
    }
    // Update the current state to the new one so the next loop iteration detects changes.
    currentState = newState;
    
    // Update time of last sample.
    lastSampleMS = currentMS;
  }
  
  // Update time of last frame.
  lastMS = currentMS;
}
