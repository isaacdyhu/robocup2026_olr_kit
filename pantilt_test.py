"""
pantilt_test.py -- confirm the OpenMV H7 Plus can drive the SingTown
pan/tilt bracket's two servos, before anything else depends on it.

Hardware: github.com/SingTown/OpenMV-Pan-Tilt. That repo is mechanical
design only (STL files for the bracket, eagle files for the mounting
plate) -- no example code or wiring notes, so nothing here is copied
from it. The servos themselves plug into the OpenMV H7 Plus's own two
dedicated PWM headers.

Confirmed against OpenMV's own docs, forums, and tutorials (checked
directly rather than guessed, given how many sensor/image API
assumptions turned out wrong earlier in this project):
  pyb.Servo(1) -> P7 -> pan
  pyb.Servo(2) -> P8 -> tilt
  .angle(degrees[, time_ms]) -- 0 = centre, roughly -90..+90 is the full
  travel of a standard 180 deg hobby servo. TIM4 is shared by pyb.Servo,
  so don't mix machine.PWM on P7/P8 into the same script later.

SIGN CONVENTIONS -- confirmed by running this, not assumed:
  Pan's native direction is inverted from "positive = right" (raw +30
  went left). set_pan() below corrects for that, so positive = right
  everywhere this is used from now on -- matching estimate_bearing_deg()
  in camera.py, which already documents positive = right of centre. Call
  set_pan()/set_tilt(), never pan.angle()/tilt.angle() directly, so that
  correction can't get missed somewhere downstream.
  Tilt's native direction already matches design.md's mount convention
  (negative = up toward horizontal, positive = down) -- no correction
  needed, set_tilt() just passes the angle straight through.
  Still unconfirmed: what real-world angle 0 (centre) actually points at
  on either axis -- that depends on how the bracket is physically bolted
  on, and needs checking with a level against the camera, not something
  this script alone can tell you.

Run this directly in OpenMV IDE (not through camera.py) and watch the
physical bracket move through both tests.
"""

from pyb import Servo
import time

pan = Servo(1)   # P7
tilt = Servo(2)  # P8


def set_pan(angle_deg, time_ms=0):
    """Positive = right, negative = left -- corrects this servo's inverted
    native direction (see SIGN CONVENTIONS above)."""
    pan.angle(-angle_deg, time_ms)


def set_tilt(angle_deg, time_ms=0):
    """Positive = down, negative = up -- native direction, no correction
    needed (see SIGN CONVENTIONS above)."""
    tilt.angle(angle_deg, time_ms)

# --- tunables ----------------------------------------------------------

CENTER_DEG = 0

# Kept conservative rather than the servo's full +-90 range -- this is a
# wiring/direction test, not a mechanical-limits test, and cheap hobby
# servos can grind against the bracket's physical stop if driven too far
# before that limit is known. Widen once the bracket's actual safe range
# has been confirmed by hand.
PAN_TEST_DEG = 30
TILT_TEST_DEG = 30

MOVE_TIME_MS = 500  # time given to the servo to actually get there
HOLD_MS = 800       # how long to sit at each position, so it's easy to see


def move(name, setter, angle):
    print(name, "->", angle, "deg")
    setter(angle, MOVE_TIME_MS)
    time.sleep_ms(MOVE_TIME_MS + HOLD_MS)


print("centring both servos")
move("pan", set_pan, CENTER_DEG)
move("tilt", set_tilt, CENTER_DEG)

print("pan test (positive should now be right, negative left)")
move("pan", set_pan, PAN_TEST_DEG)
move("pan", set_pan, -PAN_TEST_DEG)
move("pan", set_pan, CENTER_DEG)

print("tilt test (positive is down, negative is up)")
move("tilt", set_tilt, TILT_TEST_DEG)
move("tilt", set_tilt, -TILT_TEST_DEG)
move("tilt", set_tilt, CENTER_DEG)

print("done -- confirm pan now goes right for +%d and left for -%d "
      "(if not, the sign correction in set_pan() needs revisiting)" %
      (PAN_TEST_DEG, PAN_TEST_DEG))
