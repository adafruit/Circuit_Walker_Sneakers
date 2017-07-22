// Circuit Playground Circuit Walker Sneakers
// Author: Tony DiCola
// License: MIT License
//
// This sketch uses ideas from an adaptive step detection algorithm to
// animate the board's NeoPixels when it detects a step is taken.  Load
// this sketch on a Circuit Playground classic or express and then attach
// it to your shoes (or just hold it in your hand, attach it to your clothes,
// etc.).  As you take steps the board will flicker and animate the pixels
// in rainbow colors.  The algorithm adapts to your step gait so take a few
// steps to help calibrate and improve the step detection. The sketch uses 
// the adaptive step jerk theshold ('asjt') approach to detect steps as mentioned in:
//   https://github.com/danielmurray/adaptiv
#include <Adafruit_CircuitPlayground.h>

#include "IIRFilter.h"


// Configuration values (you don't need to change these, but can!):
#define  ANIMATION_FREQ_HZ   1.0     // The frequency or speed of the rainbow animation.
                                     // A value of 1 means the rainbow will spin once
                                     // a second.  Higher values increase how many times
                                     // it spins per second, and lower values decrease it.

#define  STEP_FLASH_MS       500     // Amount of time in milliseconds to turn on
                                     // the NeoPixels red when a step is detected.
                                     // This controls how long the animation runs after
                                     // a step is detected.

#define  INITIAL_JERK_AVG    0.15    // Initial average 'dip' or 'jerk' in movement
                                     // to be considered a step.  The algorithm will adapt
                                     // this value over time based on how strong or soft
                                     // jerks are detected.  Increase this value to make the
                                     // detection less sensitive at the start and decrease
                                     // to make more sensitive.

#define  ALPHA               0.125   // Weight for a weighted average of the jerk value.
                                     // This blends together a percent of a new jerk value
                                     // with the current average jerk value.

#define  BETA                0.25    // Weight for a weighted standard deviation of the jerk value.
                                     // This blends together a percent of a new jerk deviation
                                     // with the current standard deviation value.

#define  GRAVITY             1.0     // Baseline accelerometer magnitude to detect zero
                                     // crossings in the filtered data.  Since the units
                                     // are gravities and 1.0G is expected when between
                                     // peak/trough keep this at 1.0 or very close to it.  

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

// Color gamma correction table, this makes the hues of the NeoPixels
// a little more accurate by accounting for our eye's non-linear color
// sensitivity.  See this great guide for more details:
//   https://learn.adafruit.com/led-tricks-gamma-correction/the-issue
const uint8_t PROGMEM gamma8[] = {
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,
    1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,
    2,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  4,  4,  5,  5,  5,
    5,  6,  6,  6,  6,  7,  7,  7,  7,  8,  8,  8,  9,  9,  9, 10,
   10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16,
   17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25,
   25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36,
   37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50,
   51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68,
   69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89,
   90, 92, 93, 95, 96, 98, 99,101,102,104,105,107,109,110,112,114,
  115,117,119,120,122,124,126,127,129,131,133,135,137,138,140,142,
  144,146,148,150,152,154,156,158,160,162,164,167,169,171,173,175,
  177,180,182,184,186,189,191,193,196,198,200,203,205,208,210,213,
  215,218,220,223,225,228,231,233,236,239,241,244,247,249,252,255 };


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
float jerkMean = INITIAL_JERK_AVG;
float jerkDev = jerkMean / 2.0;


void animateRainbow(uint32_t current, float elapsed) {
  float t = current / 1000.0;
  for (int i=0; i<10; ++i) {
    float phase = 2.0*PI*(i/10.0);
    float x = sin(2.0*PI*ANIMATION_FREQ_HZ*t + phase);
    float h = lerp(x, -1.0, 1.0, 0.0, 360.0);
    setPixelHSV(i, h, 1.0, 1.0-elapsed);
  }
  CircuitPlayground.strip.show();
}

// Helper to change the color of a NeoPixel on the Circuit Playground board.
// Will automatically convert from HSV color space to RGB and apply gamma
// correction.
float setPixelHSV(int i, float h, float s, float v) {
  // Convert HSV to RGB
  float r, g, b = 0.0;
  HSVtoRGB(&r, &g, &b, h, s, v);
  // Lookup gamma correct RGB colors (also convert from 0...1.0 RGB range to 0...255 byte range).
  uint8_t r1 = pgm_read_byte(&gamma8[int(r*255.0)]);
  uint8_t g1 = pgm_read_byte(&gamma8[int(g*255.0)]);
  uint8_t b1 = pgm_read_byte(&gamma8[int(b*255.0)]);
  // Set the color of the pixel.
  CircuitPlayground.strip.setPixelColor(i, r1, g1, b1);
}

// HSV to RGB color space conversion function taken directly from:
//  https://www.cs.rit.edu/~ncs/color/t_convert.html
void HSVtoRGB( float *r, float *g, float *b, float h, float s, float v )
{
  int i;
  float f, p, q, t;
  if( s == 0 ) {
    // achromatic (grey)
    *r = *g = *b = v;
    return;
  }
  h /= 60;      // sector 0 to 5
  i = floor( h );
  f = h - i;      // factorial part of h
  p = v * ( 1 - s );
  q = v * ( 1 - s * f );
  t = v * ( 1 - s * ( 1 - f ) );
  switch( i ) {
    case 0:
      *r = v;
      *g = t;
      *b = p;
      break;
    case 1:
      *r = q;
      *g = v;
      *b = p;
      break;
    case 2:
      *r = p;
      *g = v;
      *b = t;
      break;
    case 3:
      *r = p;
      *g = q;
      *b = v;
      break;
    case 4:
      *r = t;
      *g = p;
      *b = v;
      break;
    default:    // case 5:
      *r = v;
      *g = p;
      *b = q;
      break;
  }
}

// Linear interpolation of value y within y0...y1 given x and x0...x1.
float lerp(float x, float x0, float x1, float y0, float y1) {
  return y0 + (x-x0)*((y1-y0)/(x1-x0));
}


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

  // Drive the NeoPixel animation if they should currently be animating.
  if (pixelClearMS > 0) {
    pixelClearMS -= deltaMS;
    if (pixelClearMS <= 0) {
      // Done animating, turn off the pixels.
      pixelClearMS = 0;
      CircuitPlayground.clearPixels();
    }
    else {
      // Still animating, computing how long the animation has elapsed
      // as a value from 0...1.0 (0 = just started and 1.0 = done) and
      // animate the pixels.  The elapsed time value is used to fade out
      // the pixels over time (i.e. as elapsed gets closer to 1 the pixels
      // get darker).
      float elapsed = (STEP_FLASH_MS - pixelClearMS)/float(STEP_FLASH_MS);
      animateRainbow(currentMS, elapsed);
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
        // Check if the jerk value is within 4 standard deviations of the current jerk average.
        if (jerk > (jerkMean - 4.0*jerkDev)) {
          // Strong jerk detected! (i.e. a step)
          // Update the jerk standard deviation by blending in the current deviation with this jerk value.
          jerkDev = abs(jerkMean-jerk)*BETA + jerkDev*(1.0-BETA);
          // Likewise update the jerk average by blending in the current average with this jerk value.
          jerkMean = jerk*ALPHA + jerkMean*(1.0-ALPHA);
          // Turn on the lights.
          pixelClearMS = STEP_FLASH_MS;
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
