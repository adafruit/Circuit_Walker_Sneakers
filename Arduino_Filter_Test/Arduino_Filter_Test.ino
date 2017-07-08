// Circuit Playground Circuit Walker Sneakers - Filtering Test
// Author: Tony DiCola
// License: MIT License
//
// This sketch shows how to apply a filter to the accelerometer data from
// Circuit Playground as part of testing step detection algorithms.
// The filter to be applied is an IIR (infinite impulse response)
// Butterworth low pass filter with a cutoff frequency of about 3.667 hz.
// This is the same filter used in the adaptive step detection algorithm
// from here:
//   https://github.com/danielmurray/adaptiv
// After loading the sketch on Circuit Playground (either classic or express)
// open the serial plotter at 115200 baud.  The raw magnitude of the accelerometer
// will be displayed as one graph, and the low-pass filtered magnitude will be
// displayed as another series on the graph.  Notice as you move the board around
// the filtered data doesn't jump around as much, and that the filtered data is
// slightly 'behind' the raw data in time (i.e. it's out of phase).
#include <Adafruit_CircuitPlayground.h>

#include "IIRFilter.h"


// Configuration values:
#define  SAMPLE_RATE_HZ      50.0    // How quickly to sample the accelerometer.
                                     // Note that the filter is generated using
                                     // an assumed 50hz sample rate so if you change
                                     // this value you also need to regenerate and
                                     // change the filter coefficients below.

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
  }
}
