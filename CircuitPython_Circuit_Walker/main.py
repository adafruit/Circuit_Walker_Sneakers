# Circuit Playground Express Circuit Walker Sneakers
# Author: Tony DiCola
# License: MIT License
#
# This sketch animates the NeoPixels on Circuit Playground Express in a flashy
# rainbow when it detects a spike of acceleration from the board's
# accelerometer.  Attach it to your sneakers for a quick and easy Python-powered
# 'circuit walker' sneaker project.
#
# This requires the neopixel.mpy be loaded on your board from:
#   https://github.com/adafruit/Adafruit_CircuitPython_NeoPixel/releases
# Or from the CircuitPython bundle:
#   https://github.com/adafruit/Adafruit_CircuitPython_Bundle
import array
import math
import time

import board
import busio
import digitalio

import adafruit_lis3dh
import neopixel


# Configuration:
TAP_THRESHOLD_G     = 1.5      # Threshold for detecting a step and starting the
                               # animation.  This is a value in gravities (G's)
                               # so 1G = the normal force of gravity.  Typically
                               # you want a slightly higher value like 1.5 as
                               # the threshold so your foot lifting up against
                               # gravity triggers a step.

TAP_TIME_LIMIT_S    = 0.25     # Time window for step detection (in seconds).
                               # This configures the tap detection TIME_LIMIT
                               # register and tunes the tap detection.  If
                               # a spike of acceleration on any axis goes above
                               # TAP_THRESHOLD_G for _less_ than this time then
                               # a step is detected.  Decreasing this value
                               # makes the detection more sensitive to fast
                               # spikes.

STEP_FLASH_S        = 0.5      # Amount of time in seconds to flash/animates
                               # when a step is detected.

ANIMATION_FREQ_HZ   = 1.0      # Frequency for the sine wave of colors that
                               # occur during an animation.  Increase this to
                               # speed up the rainbow animation and decrease
                               # to slow down.

# Gamma correction lookup table.  This is adapted from the table at:
#   https://learn.adafruit.com/led-tricks-gamma-correction/the-quick-fix
GAMMA8 = array.array('B', (
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
  215,218,220,223,225,228,231,233,236,239,241,244,247,249,252,255
))


# Helper functions:
def lerp(x, x0, x1, y0, y1):
    """Linearly interpolate a value y given range y0...y1 that is proportional
    to x in range x0...x1 .
    """
    return y0 + (x-x0)*((y1-y0)/(x1-x0))

def HSV_to_RGB(h, s, v):
    """HSV color space to RGB color space conversion. Hue (h) should be a
    degree value from 0.0 to 360.0, saturation (s) and value (v) should be a
    value from 0.0 to 1.0.  Returns a 3-tuple of gamma-corrected RGB color
    bytes (0-255).
    """
    # This is adapted from C/C++ code here:
    #  https://www.cs.rit.edu/~ncs/color/t_convert.html
    r = 0
    g = 0
    b = 0
    if s == 0.0:
        r = v
        g = v
        b = v
    else:
        h /= 60.0       # sector 0 to 5
        i = int(math.floor(h))
        f = h - i       # factorial part of h
        p = v * ( 1.0 - s )
        q = v * ( 1.0 - s * f )
        t = v * ( 1.0 - s * ( 1.0 - f ) )
        if i == 0:
            r = v
            g = t
            b = p
        elif i == 1:
            r = q
            g = v
            b = p
        elif i == 2:
            r = p
            g = v
            b = t
        elif i == 3:
            r = p
            g = q
            b = v
        elif i == 4:
            r = t
            g = p
            b = v
        else:
            r = v
            g = p
            b = q
    r = GAMMA8[int(255.0*r)]
    g = GAMMA8[int(255.0*g)]
    b = GAMMA8[int(255.0*b)]
    return (r, g, b)

def animate_rainbow(pixels, current, remaining):
    """Update pixels with a frame of cyling rainbow animation.  Current should
    be a time value (in seconds) and remaining should be an indicator of how
    much time remains as a value 0...1.0 with 0=no time left and 1.0=all time
    left.  The remaining time is used to fade out the rainbow over time.
    """
    # Precompute some constants to speed up math inside the loop.
    a = 2.0*math.pi*ANIMATION_FREQ_HZ*current
    phase_offset = 2.0*math.pi/pixels.n
    # Go through each pixel and compute its color.
    for i in range(pixels.n):
        # Compute phase of each pixel so the rainbow color wraps around pixels.
        phase = i*phase_offset
        # Use sine wave to smoothly 'move' the rainbow.
        x = math.sin(a + phase)
        # Interpolate a hue from the sine wave output.
        h = lerp(x, -1.0, 1.0, 0.0, 360.0)
        # Set the pixel to a color based on the computed hue and remaining time.
        pixels[i] = HSV_to_RGB(h, 1.0, remaining)

def error_sos(pixels, message):
    """Flash a red S.O.S (three short, then three long blinks) and print the
    indicated error message.
    """
    while True:
        print(message)
        # Three short blinks.
        for i in range(3):
            pixels.fill((255,0,0))
            pixels.write()
            time.sleep(0.25)
            pixels.fill((0,0,0))
            pixels.write()
            time.sleep(0.25)
        # Three long blinks.
        for i in range(3):
            pixels.fill((255,0,0))
            pixels.write()
            time.sleep(0.50)
            pixels.fill((0,0,0))
            pixels.write()
            time.sleep(0.50)


# Initialization / setup that happens once at start:
# Define NeoPixels on the board (and turn them off).
pixels = neopixel.NeoPixel(board.NEOPIXEL, 10)
pixels.fill((0,0,0))
pixels.write()

# Initialize the LIS3DH accelerometer.
# Note that this is specific to Circuit Playground Express boards.  For other
# uses change the SCL and SDA pins below, and optionally the address of the
# device if needed.
i2c = busio.I2C(board.ACCELEROMETER_SCL, board.ACCELEROMETER_SDA)
lis3dh = adafruit_lis3dh.LIS3DH_I2C(i2c, address=25)

# Set accelerometer range and data rate to good values for step detection.
# Range is +/-4G so a small ~1.5G threshold can be detected with tap detection.
# Data rate is reduced to 50hz as faster samplining is not needed.
# Note if you change these you must also recalculate the divisor for tap
# threshold and time limit below!
lis3dh.range = adafruit_lis3dh.RANGE_4_G
lis3dh.data_rate = adafruit_lis3dh.DATARATE_50_HZ

# Compute tap detection threshold and time limit values given desired targets.
# The actual threshold and time limit values can only be set to a 7-bit range
# of values that depend on the current range and data rate.  Find the closest
# one that is at least the target.  If a value is too high then stop and flash
# an error S.O.S. in red.
tap_threshold = int(math.ceil(TAP_THRESHOLD_G/(4.0/128.0)))
if tap_threshold > 127:
    error_sos(pixels, 'Tap threshold is too high to represent!')
print('Using tap threshold value: {}'.format(tap_threshold))

time_limit = int(math.ceil(TAP_TIME_LIMIT_S/(1.0/50.0)))
if time_limit > 127:
    error_sos(pixels, 'Time limit is too high to represent!')
print('Using time limit value: {}'.format(time_limit))

# Enable single click detection with the provided tap threshold and time limits.
lis3dh.set_click(1, tap_threshold, time_limit=time_limit)

# Main loop will run forever animating pixels and checking for click/taps from
# accelerometer which signal a step.
last = time.monotonic() # Keep track of the last time the loop ran.
pixels_animate = 0      # Also track how long the pixels remain to be animated.
while True:
    # Update and calculate the amount of time since the last loop iteration.
    current = time.monotonic()
    delta = current - last
    last = current
    # Update amount of time remaining for pixels animate.  Make sure this
    # never goes below 0.
    pixels_animate = max(0, pixels_animate - delta)
    if pixels_animate > 0:
        # Still some time left for the pixels to animate!
        # Calculate remaining time as a 0...1.0 value (0=no time, 1=max time).
        remaining = pixels_animate/STEP_FLASH_S
        # Draw a frame of rainbow animation on the pixels.
        animate_rainbow(pixels, current, remaining)
    else:
        # No more time left to animate pixels, just turn them off/black.
        pixels.fill((0,0,0))
    # Make sure to write/update the pixels after changing them above.
    pixels.write()
    # Check for a tap/click from the accelerometer.
    # A single tap will occur when acceleration on any axis exceeds the
    # specified threshold for less than the time window.
    single, double = lis3dh.read_click()
    if single:
        # Got a single tap, set the time for the pixels to animate.
        pixels_animate = STEP_FLASH_S
    # Small delay to give time for any other processing.
    time.sleep(0.01)
