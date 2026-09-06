#!/usr/bin/env pybricks-micropython
"""
pybricks/line_follow.py -- basic two-sensor bang-bang line follower.

Both wheels drive forward by default. Each wheel's own-side sensor
independently controls only that wheel: if a sensor sees black, its
wheel reverses instead of driving forward, which pivots the robot away
from the line on that side. No PD control, no shared error term between
the two sides -- deliberately simple, as a first working version to get
the robot moving before anything more sophisticated.

Ports (confirmed against design.md's Hub 1 table):
  A -- left inner colour sensor
  E -- right inner colour sensor
  B -- left wheel motor
  F -- right wheel motor
"""

from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Port, Direction, Color
from pybricks.tools import wait

left_sensor = ColorSensor(Port.A)
right_sensor = ColorSensor(Port.E)

# Left/right wheel motors are normally mounted mirror-imaged on a
# differential drive, so identical positive speeds can end up spinning
# them in physically opposite directions. Untested which way is correct
# here -- if the robot doesn't drive straight forward, flip one of these
# two Direction values.
left_motor = Motor(Port.B, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.F, Direction.CLOCKWISE)

# --- tunables ----------------------------------------------------------

DRIVE_SPEED = 50      # deg/s -- forward speed while a side is on white
BACKTRACK_SPEED = 100  # deg/s -- reverse speed while a side sees black

# reflection() is 0 (darkest) to 100 (brightest). Untested placeholder --
# calibrate against the real line and background.
LINE_THRESHOLD = 50

# Blind pivot on a green marker: both wheels move, but the wheel on the
# side the marker was seen on (the turn side) runs slower than the other
# -- an arc toward that side, not a point-turn (equal-and-opposite
# speeds) and not a single-wheel turn (one side stopped). All three
# untested placeholders -- PIVOT_DURATION_MS in particular needs
# calibrating against how far the robot actually needs to turn to find
# the branch.
PIVOT_INNER_SPEED = 100    # deg/s -- wheel on the side turning toward
PIVOT_OUTER_SPEED = 300    # deg/s -- wheel on the opposite side
PIVOT_DURATION_MS = 500    # how long the blind turn runs

# Blind straight-through move when BOTH sensors have read black for
# several consecutive loop ticks -- reversing both wheels (the normal
# per-side reaction to seeing black) doesn't actually get the robot
# anywhere on a real perpendicular crossing, since the band is still
# there when it drives forward again. So: let the normal reversal happen
# first, and only give up on it and drive straight through once it's
# clearly not resolving anything. EXPERIMENTAL -- easy to remove if it
# doesn't work out: this block, both_black_streak and the branch that
# uses it in the main loop, and CROSSING_SPEED/CROSSING_DURATION_MS/
# BOTH_BLACK_STREAK_LIMIT are the only things touched.
#
# All three untested placeholders. CROSSING_DURATION_MS needs sizing
# against the crossing's actual physical width plus margin for it not
# being perfectly perpendicular to the robot's approach (design.md: a
# 15.8mm crossing becomes ~18.2mm effective width at 30 deg of yaw).
# BOTH_BLACK_STREAK_LIMIT trades off two risks: too low and a crossing
# gets blasted through before the reversal even had a chance to matter;
# too high and the robot sits there reversing (going nowhere useful) for
# longer than necessary on a real crossing.
CROSSING_SPEED = 200          # deg/s, both wheels equally
CROSSING_DURATION_MS = 300    # how long the blind forward move runs
BOTH_BLACK_STREAK_LIMIT = 3   # consecutive both-black ticks before giving
                              # up on reversing and driving forward instead


def is_black(sensor):
    return sensor.reflection() < LINE_THRESHOLD


def cross_black_band():
    """
    Drive straight through a crossing: equal speed, both wheels, for a
    fixed duration, no reaction to either sensor while it runs. Same
    run_time(..., wait=False) + shared wait() pattern as pivot(), so both
    wheels start together instead of running one after the other.
    """
    left_motor.run_time(CROSSING_SPEED, CROSSING_DURATION_MS, wait=False)
    right_motor.run_time(CROSSING_SPEED, CROSSING_DURATION_MS, wait=False)
    wait(CROSSING_DURATION_MS)


def pivot(direction):
    """
    Blind arc turn toward `direction` ("left" or "right"). Uses
    run_time(..., wait=False) on both motors so they start together, then
    a single wait() for the shared duration -- run_time(..., wait=True)
    on one motor and then the other would run them sequentially, not
    simultaneously.
    """
    if direction == "left":
        inner_motor, outer_motor = left_motor, right_motor
    else:
        inner_motor, outer_motor = right_motor, left_motor

    inner_motor.run_time(PIVOT_INNER_SPEED, PIVOT_DURATION_MS, wait=False)
    outer_motor.run_time(PIVOT_OUTER_SPEED, PIVOT_DURATION_MS, wait=False)
    wait(PIVOT_DURATION_MS)


both_black_streak = 0

while True:
    # Green outranks the black/white check -- a marker sits immediately
    # before the junction it marks, so missing it means driving straight
    # through a turn the course just instructed (same reasoning as
    # design.md Sec6's guard ordering, though this uses Pybricks' built-in
    # colour classifier rather than design.md's HSV hue-window approach,
    # a simplification worth revisiting if it misfires in real lighting).
    if left_sensor.color() == Color.GREEN:
        pivot("left")
        both_black_streak = 0
    elif right_sensor.color() == Color.GREEN:
        pivot("right")
        both_black_streak = 0
    else:
        left_black = is_black(left_sensor)
        right_black = is_black(right_sensor)

        if left_black and right_black:
            both_black_streak += 1
        else:
            both_black_streak = 0

        if both_black_streak >= BOTH_BLACK_STREAK_LIMIT:
            cross_black_band()
            both_black_streak = 0
        else:
            left_motor.run(-BACKTRACK_SPEED if left_black else DRIVE_SPEED)
            right_motor.run(-BACKTRACK_SPEED if right_black else DRIVE_SPEED)

    wait(10)
