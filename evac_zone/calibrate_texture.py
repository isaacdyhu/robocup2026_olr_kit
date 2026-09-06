"""
evac_zone/calibrate_texture.py -- pick TEXTURE_MIN (and sanity-check
TEXTURE_WINDOW) for zone_scan.py's texture-based silver-ball detection.

There's no built-in tool for this the way OpenMV IDE's Threshold Editor
handles colour thresholds, since this is a custom blur-then-subtract
pipeline, not a stock colour/grayscale range. So: show the texture map
itself instead of the normal camera view, with the value under a centre
crosshair printed on-screen and to the console every frame.

How to use:
  1. Run this in OpenMV IDE.
  2. Point the camera so the *background* (white floor, nothing else)
     fills the centre crosshair. Note the printed value -- average a few
     frames, it'll jitter a little.
  3. Point it so the *ball* fills the centre crosshair. Note that value.
  4. TEXTURE_MIN in zone_scan.py should sit somewhere between the two --
     comfortably above the background reading, comfortably below the
     ball's, with margin on both sides for frame-to-frame jitter.

If the two numbers end up close together (little separation between
background and ball), that's a sign TEXTURE_WINDOW needs adjusting
before TEXTURE_MIN can be picked well -- try a couple of different
window sizes here and see which gives the cleanest separation.
"""

import sensor
import time

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_vflip(True)    # match zone_scan.py's mount orientation
sensor.set_hmirror(True)
clock = time.clock()

TEXTURE_WINDOW = 2  # keep this in sync with whatever zone_scan.py uses

while True:
    clock.tick()
    img = sensor.snapshot()

    texture = img.copy()
    texture = texture.to_grayscale()
    blurred = texture.copy()
    blurred.mean(TEXTURE_WINDOW)
    texture.difference(blurred)

    cx = texture.width() // 2
    cy = texture.height() // 2
    value = texture.get_pixel((cx, cy))

    texture.draw_cross((cx, cy), color=255, size=8, thickness=1)
    texture.draw_string((4, 4), "centre=%d  window=%d" % (value, TEXTURE_WINDOW), color=255)
    texture.draw_string((4, texture.height() - 12), "fps=%.1f" % clock.fps(), color=255)

    print("texture value at centre:", value)
