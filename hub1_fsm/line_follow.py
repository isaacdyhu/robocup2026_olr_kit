#!/usr/bin/env pybricks-micropython
"""
hub1_fsm/line_follow.py -- full state-machine framework, debug-stepped.

Restarted from scratch: every state design.md defines (Sec4/Sec5/Sec7)
is represented here, with the correct transitions between them, but only
state F's actual behaviour (continuous black-line following) is
implemented. Every other state -- including F's own outgoing guards --
is a stub: the function exists, is called every tick while active, and
is where that state's real logic goes later. Nothing here is a
simplification of design.md's state names; L and R are separate states
(not merged, unlike an earlier draft of this file), and T correctly
means DEPOSIT (a zone state, design.md Sec7), not "green turn".

DEBUG MODE: hub.display.char(state) shows which state is active. Once
a state is ARMED (see below), transitions between states happen fully
automatically -- the instant a state function's own guard condition
fires, the state variable updates and the new state starts running.
The right button does NOT gate transitions; it gates ACTION. Every
freshly-entered state starts unarmed (paused, robot.stop()'d, its
function not even called) until the right button is pressed once --
edge-triggered, so holding it down doesn't repeat -- at which point it
arms and its real behaviour (and guard-checking) begins immediately and
keeps running, unattended, until that state's own guard hands control
to the next state, which again starts unarmed and waits for its own
press. This is one button press per state entered, not one press per
transition. IMPORTANT CONSEQUENCE OF THE ABOVE: since no state's guard
logic is implemented yet -- not even F's, beyond the green guard --
arming a stub state will just sit there with nothing visibly happening
until real guards are filled in, state by state. That's expected, not
a bug.

Ports (confirmed against design.md's Hub 1 table, same hardware as
pybricks/line_follow.py):
  A -- left inner colour sensor
  E -- right inner colour sensor
  C -- ultrasonic distance sensor
  B -- left wheel motor
  F -- right wheel motor
"""

import math

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Direction, Button
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from pybricks.iodevices import PUPDevice

# --- hub 2 link (BLE broadcast) ---------------------------------------------
#
# Master switch, and the reason this is a switch at all: with hub 2 off,
# an unconditional hub2_goal() blocks for its whole timeout and then
# reports failure, which is indistinguishable from a jammed claw and
# leaves state A looking like it did nothing. Off means the claw steps
# are skipped outright and the states that use them still run, so the
# driving can be tested without hub 2 present. Turn it on once hub 2 is
# actually running pybricks/hub2_code.py.
HUB2_ENABLED = True

# Channel numbers and goal letters are COPIES of pybricks/hub2_code.py's
# own constants, checked against that file directly rather than taken
# from design.md -- the doc's Sec3 describes a higher-level goal set
# (STOW/CAPTURE/RELEASE) that hub 2 does not actually implement. What it
# really accepts is these four primitives. Nothing enforces the match,
# and pybricks/hub1_test_hub2.py's own header spells out how a mismatch
# fails: hub 2 doesn't recognise the goal and reports FAULT, which looks
# exactly like a jammed motor.
COMMAND_CHANNEL = 1     # hub 1 -> hub 2
STATE_CHANNEL = 2       # hub 2 -> hub 1

GOAL_LOWER = "L"        # lifter down to grab height
GOAL_CLOSE = "C"        # claw closed until it stalls, plus a grip verdict
GOAL_RAISE = "R"        # lifter up to carry height
GOAL_OPEN = "O"         # claw opened

PHASE_DONE = "D"
PHASE_FAULT = "F"

# BLE channels can only be declared at construction, never added to an
# existing PrimeHub -- hence the branch rather than a later call. With
# HUB2_ENABLED off this is byte-for-byte the plain PrimeHub() this file
# used before hub 2 existed, so everything already working (line
# following, the scan, the obstacle bypass) is untouched by any of this.
if HUB2_ENABLED:
    hub = PrimeHub(broadcast_channel=COMMAND_CHANNEL,
                   observe_channels=[STATE_CHANNEL])
else:
    hub = PrimeHub()

left_sensor = ColorSensor(Port.A)
right_sensor = ColorSensor(Port.E)
ultrasonic_sensor = UltrasonicSensor(Port.C)  # obstacle detection (design.md
                                              # Sec4 guard 2 / Sec6 state O)

# Confirmed direction for this hardware in pybricks/line_follow.py's
# latest tested state -- reused here rather than re-guessed, since it's
# the same robot. If it doesn't drive straight, that file's own note
# still applies: left/right motors are usually mirror-mounted, so flip
# one of these two.
left_motor = Motor(Port.B, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.F, Direction.CLOCKWISE)

# --- drivebase --------------------------------------------------------------
#
# Continuous drive(speed, turn_rate) every tick, not blocking turn()+
# straight() steps -- design.md Sec4's own critique of the old approach.
# WHEEL_DIAMETER_MM/AXLE_TRACK_MM: real measured values, carried over
# from this file's previous version -- not placeholders.
WHEEL_DIAMETER_MM = 85
AXLE_TRACK_MM = 140
robot = DriveBase(left_motor, right_motor, WHEEL_DIAMETER_MM, AXLE_TRACK_MM)

# --- tunables: line-following control law (design.md Sec4) -----------------
#
# Reused directly from design.md's own response-curve table rather than
# re-derived. hsv().v is already 0-100 on Pybricks, so no separate
# normalise() step is needed the way design.md's real calibrated version
# has one; that 30s white/black calibration window (Sec6.1.2) isn't
# implemented here yet.
V_MAX = 60        # mm/s, speed at error ~= 0
V_MIN = 10         # mm/s, speed at or beyond ERROR_FULL
ERROR_FULL = 60    # error magnitude at which speed reaches V_MIN
STEER_GAIN = 0.09  # deg/mm of curvature per unit of error
TURN_MAX = 300     # deg/s, clamp

LINE_LOST_MM = 300  # mm of continuous both-white travel before guard 4
                    # (-> B) fires. design.md's own figure is ~200mm;
                    # set to 300 here per this project's own spec.
                    # Untested placeholder.

# --- tunables: green marker detection (design.md Sec6) ----------------------
#
# Hue window plus saturation/value floors -- rejects black, white and
# grey regardless of brightness, rather than a colour() classification.
# All untested placeholders; Pybricks hsv() is h:0-359, s/v:0-100.
GREEN_HUE_MIN = 100
GREEN_HUE_MAX = 180
GREEN_SAT_MIN = 40
GREEN_VAL_MIN = 15

# Three consecutive samples before committing to a turn -- rejects
# sensor noise, not a brief spatial false positive (design.md Sec6's own
# accepted trade-off for this same debounce).
GREEN_CONFIRM_SAMPLES = 1


def read_sensors():
    return left_sensor.hsv(), right_sensor.hsv()


def is_green(hsv):
    return (
        GREEN_HUE_MIN <= hsv.h <= GREEN_HUE_MAX
        and hsv.s >= GREEN_SAT_MIN
        and hsv.v >= GREEN_VAL_MIN
    )


def follow_line(l_hsv, r_hsv):
    """
    State F's actual driving behaviour: continuous curvature-based
    steering (design.md Sec4). Speed scheduling (fast when centred, slow
    on error) plus a curvature term independent of speed, so raising
    V_MAX doesn't silently invalidate STEER_GAIN.

    Takes the already-read hsv values rather than reading the sensors
    itself: design.md Sec4's own reasoning is one hsv() per sensor per
    tick, cached and reused for the control law AND the green gate below,
    not read twice.
    """
    error = l_hsv.v - r_hsv.v  # no polarity term -- state W (inverted
                               # sign) isn't implemented yet either
    speed = V_MAX - (V_MAX - V_MIN) * min(1, abs(error) / ERROR_FULL)
    turn_rate = STEER_GAIN * error * speed
    turn_rate = max(-TURN_MAX, min(TURN_MAX, turn_rate))
    robot.drive(speed, turn_rate)


# --- blind pivot turn (design.md Sec6) ---------------------------------------
#
# The two tunables that fully describe one pivot. Written right-turn-
# positive, and consumed that way directly by state_R() below; state_L()
# is this maneuver's mirror image, not a separately-tuned pivot, so it
# negates both rather than getting its own pair -- tune these against
# the right-hand green turn and the left one follows automatically.
PIVOT_OFFSET_MM = 50    # signed distance from the axle's centre to the pivot
                       # point, measured along the axle line itself.
                       # Negative = pivot to the left, positive = right,
                       # 0 = in-place spin (both wheels move, equal and
                       # opposite). Untested placeholder -- 0 until real
                       # hardware says otherwise.
PIVOT_ANGLE_DEG = 60   # signed heading change to sweep through. Positive =
                       # turn right (clockwise from above), matching
                       # DriveBase's own turn-angle sign convention, so this
                       # composes the same way turn_rate/error do elsewhere
                       # in this file. Untested placeholder.

# Not one of the two describing variables above -- a necessary third
# knob for actually driving the motors, since motor.run_angle() needs a
# speed as well as an angle. Only sets the speed of whichever wheel
# travels the LONGER arc; run_pivot() slows the other wheel down so
# both finish together (see its own docstring for why that matters).
PIVOT_SPEED_DPS = 100  # deg/s, untested placeholder


def pivot_wheel_angles(pivot_offset_mm, turn_angle_deg):
    """
    Turn PIVOT_OFFSET_MM/PIVOT_ANGLE_DEG's two signed numbers into
    (left_deg, right_deg): exactly how many degrees each wheel motor
    must rotate to sweep that turn about that pivot.

    Why this is exact, not approximate: the pivot point sits ON the
    axle line -- the same line both wheels sit on -- so as the body
    rotates about it, each wheel's rolling direction (forward/backward)
    is always exactly tangent to its own circular arc around that
    point. That means each wheel's total travel is simply
    (signed radius) * (angle in radians), with no separate sideways/sine
    term to worry about -- unlike pivoting about a point ahead of or
    behind the robot, which these wheels physically can't do at all.

    Signed radius is measured wheel-to-pivot (not pivot-to-wheel) --
    that's what makes the signs land right; checked by hand against
    three cases:
      * offset=0 (in-place spin), angle>0 (turn right): left wheel
        comes out positive (forward), right comes out negative
        (backward) -- a standard in-place turn-right spin.
      * offset=+AXLE_TRACK_MM/2 (pivot sitting exactly on the right
        wheel): right wheel's radius is 0, so it doesn't move at all;
        left sweeps forward around it, like pivoting on one track.
      * abs(offset) > AXLE_TRACK_MM/2 (pivot beyond the track): both
        radii come out the same sign, so both wheels move forward
        (different amounts) -- an ordinary wide curve, not a pivot.
    """
    half_track = AXLE_TRACK_MM / 2
    r_left = pivot_offset_mm + half_track   # wheel-to-pivot signed radius
    r_right = pivot_offset_mm - half_track
    theta_rad = math.radians(turn_angle_deg)

    arc_left_mm = r_left * theta_rad
    arc_right_mm = r_right * theta_rad

    circumference_mm = math.pi * WHEEL_DIAMETER_MM
    left_deg = arc_left_mm / circumference_mm * 360
    right_deg = arc_right_mm / circumference_mm * 360
    return left_deg, right_deg


def run_pivot(pivot_offset_mm=PIVOT_OFFSET_MM, turn_angle_deg=PIVOT_ANGLE_DEG,
              speed_dps=PIVOT_SPEED_DPS):
    """
    Execute one blind (open-loop, no sensor feedback mid-turn) pivot.

    Both wheels are commanded at once, not one after the other -- run
    sequentially, they would NOT trace the same combined arc this was
    derived for; pivot_wheel_angles() assumes both rotations happen
    simultaneously, at a constant ratio, over the same span of time.
    speed_dps is given to whichever wheel has the larger |angle|; the
    other is scaled down proportionally so both finish together, rather
    than running at full speed and simply stopping early once its own
    (shorter) angle is reached -- which would race ahead of the other
    wheel and sweep the wrong radius for the back half of the turn.

    UNTESTED: run_angle()'s speed parameter is passed here as a
    magnitude, with rotation_angle carrying the direction sign --
    confirmed against Pybricks' own docs (run_target()'s docs state
    speed's sign doesn't affect direction; run_angle() isn't explicit
    about it, so this follows the same convention). Not yet run on real
    hardware.
    """
    left_deg, right_deg = pivot_wheel_angles(pivot_offset_mm, turn_angle_deg)

    max_abs_deg = max(abs(left_deg), abs(right_deg))
    if max_abs_deg == 0:
        return  # zero-length pivot -- nothing to do

    left_speed = speed_dps * abs(left_deg) / max_abs_deg
    right_speed = speed_dps * abs(right_deg) / max_abs_deg

    # wait=False on the first call so both motors start together; the
    # second call's wait=True (the default) blocks until IT finishes,
    # which -- since speeds were scaled so both take the same time --
    # is also when the first one finishes.
    left_motor.run_angle(left_speed, left_deg, wait=False)
    right_motor.run_angle(right_speed, right_deg, wait=True)


# --- branch hunt: continue the same pivot, now sensor-gated -----------------
#
# run_pivot() above is blind -- a fixed angle, committed to up front. This
# picks up from exactly where it left off and keeps rotating about that
# SAME pivot point and direction, but open-ended: it watches the far
# sensor (the one opposite whichever saw green) and stops the moment
# that sensor finds the black line, rather than committing to one preset
# angle. HUNT_MAX_ANGLE_DEG is the fallback so a missed/faded branch
# doesn't spin the robot forever.
HUNT_MAX_ANGLE_DEG = 35  # extra degrees, ON TOP OF run_pivot()'s own
                          # PIVOT_ANGLE_DEG, before giving up on finding
                          # the branch. Untested placeholder.
HUNT_SPEED_DPS = 50       # slower than PIVOT_SPEED_DPS -- this phase is
                          # sensor-gated rather than blind, so it can
                          # afford to go slower in exchange for stopping
                          # more precisely on the line. Untested placeholder.
HUNT_POLL_MS = 10         # sensor-check interval while hunting

BLACK_VAL_MAX = 20  # hsv().v at/below this reads as "on the black line".
                    # Untested placeholder -- same normalise()-free
                    # shortcut follow_line() uses, no calibrated
                    # white/black window yet (design.md Sec6.1.2).

WHITE_VAL_MIN = 70  # hsv().v at/above this reads as "on white/background".
                    # A genuine positive test for white, not just "not
                    # black" -- a grey in-between reading counts as
                    # neither, same untested-placeholder caveat as
                    # BLACK_VAL_MAX.


def is_black(hsv):
    return hsv.v <= BLACK_VAL_MAX


def is_white(hsv):
    return hsv.v >= WHITE_VAL_MIN


def hunt_for_branch(pivot_offset_mm, turn_sign, far_sensor):
    """
    Keep rotating about pivot_offset_mm (same signed offset run_pivot()
    was just given) in the same direction (turn_sign: +1 = right/
    clockwise, -1 = left) until far_sensor reads black or
    HUNT_MAX_ANGLE_DEG of additional rotation is swept, whichever comes
    first. far_sensor is the sensor OPPOSITE the one that triggered the
    green guard in state_F() -- that's the one now sweeping across
    toward the branch as the turn continues.

    Runs the motors continuously (run(), not run_angle()) rather than
    stepping in fixed increments, since the stopping condition here is a
    sensor reading, not a preset angle -- the same reasoning run_pivot()
    itself doesn't apply, because THAT phase has no sensor to gate on.
    """
    half_track = AXLE_TRACK_MM / 2
    r_left = pivot_offset_mm + half_track   # same wheel-to-pivot radii
    r_right = pivot_offset_mm - half_track  # pivot_wheel_angles() derives

    max_abs_r = max(abs(r_left), abs(r_right))
    if max_abs_r == 0:
        return  # degenerate pivot (offset cancels both wheels) -- bail

    left_speed = turn_sign * HUNT_SPEED_DPS * r_left / max_abs_r
    right_speed = turn_sign * HUNT_SPEED_DPS * r_right / max_abs_r

    # Track total body rotation from whichever wheel has the LARGER
    # |radius| (i.e. moves more per degree of body rotation) -- better
    # resolution than tracking the near-stationary inner wheel, and
    # avoids dividing by a near-zero radius below.
    if abs(r_left) >= abs(r_right):
        track_motor, track_r = left_motor, r_left
    else:
        track_motor, track_r = right_motor, r_right
    track_motor.reset_angle(0)

    circumference_mm = math.pi * WHEEL_DIAMETER_MM

    left_motor.run(left_speed)
    right_motor.run(right_speed)

    while True:
        wheel_arc_mm = track_motor.angle() / 360 * circumference_mm
        swept_deg = abs(math.degrees(wheel_arc_mm / track_r))
        if swept_deg >= HUNT_MAX_ANGLE_DEG:
            break
        if is_black(far_sensor.hsv()):
            break
        wait(HUNT_POLL_MS)

    left_motor.stop()
    right_motor.stop()


# --- obstacle bypass (design.md Sec6 "Obstacle bypass (state O)") -----------
#
# Supersedes design.md's own written spec (a fixed 70deg/-200mm/-100deg
# blind detour) with a different, explicitly-requested maneuver that
# circles the obstacle -- the course's "water tower" -- using it as the
# pivot itself, gyro-gated for accuracy rather than trusting blind wheel
# rotation over what is a much longer turn than the green-turn pivot:
#
#   1. Record the current heading (hub.imu.heading()), then spin
#      OBSTACLE_SPIN_DEG clockwise on the spot. Since the spin doesn't
#      translate the robot, this points it tangentially past the tower,
#      which is now assumed to be directly to its LEFT at
#      OBSTACLE_PIVOT_RADIUS_MM -- a fixed assumed distance rather than
#      the ultrasonic's live reading at trigger time, so the loop's
#      radius doesn't depend on exactly where inside OBSTACLE_TRIGGER_MM
#      the guard happened to fire.
#   2. Pivot OBSTACLE_LOOP_DEG (half circle) with the assumed tower
#      position as the pivot point -- pivot_offset =
#      -OBSTACLE_PIVOT_RADIUS_MM. Because that offset is far beyond half
#      the axle track, both wheels drive forward (at different rates),
#      tracing a loop around the tower rather than reversing a wheel,
#      and bring the robot out the other side, near the original line
#      of travel, roughly facing back along it.
#   3. NOT a blind return to the original heading -- drive straight
#      ahead and hunt for the line with the inner-RIGHT sensor (the one
#      now sweeping toward it after the loop), stopping the instant it
#      reads black, or OBSTACLE_REJOIN_MAX_MM if it never does. This is
#      the sensor-gated close-out moves 1-2's blind geometry can't
#      guarantee by itself.
#
# Moves 1-2 share rotate_to_heading() below rather than run_pivot():
# that one is open-loop (a fixed motor-angle command), fine for the
# short green-turn pivot but not for a maneuver whose middle step is up
# to a full 180-degree loop, where wheel slip would accumulate into a
# real heading error. rotate_to_heading() runs continuously and stops
# only once the gyro itself confirms the target heading.
OBSTACLE_TRIGGER_MM = 220         # design.md's exact figure -- ultrasonic
                                  # threshold for the F -> O guard
OBSTACLE_SPIN_DEG = 70           # move 1: degrees to turn tangential to
                                  # the tower
OBSTACLE_LOOP_DEG = 120          # move 2: half-circle around the tower
OBSTACLE_PIVOT_RADIUS_MM = 200   # move 2's pivot distance -- a fixed
                                  # assumed clearance rather than the
                                  # ultrasonic's live reading at trigger
                                  # time, so the loop's radius doesn't
                                  # depend on exactly where inside
                                  # OBSTACLE_TRIGGER_MM the guard happened
                                  # to fire.
OBSTACLE_ROTATE_SPEED_DPS = 150  # deg/s for moves 1-2
OBSTACLE_ROTATE_POLL_MS = 10     # gyro poll interval while rotating

ROTATE_SLOWDOWN_MARGIN_DEG = 15  # once rotate_to_heading() is this close
                                  # to its target, it drops to
                                  # ROTATE_CREEP_SPEED_DPS rather than
                                  # stopping abruptly from full speed --
                                  # motor momentum plus this loop's own
                                  # OBSTACLE_ROTATE_POLL_MS polling gap
                                  # both add real overshoot past the
                                  # target otherwise, worse the longer/
                                  # faster the rotation was (exactly what
                                  # overshot the -90 scan step). Untested
                                  # placeholder.
ROTATE_CREEP_SPEED_DPS = 30      # deg/s during that final approach

OBSTACLE_REJOIN_MAX_MM = 300     # move 3: give up and -> H if the line
                                  # still hasn't been found after this
                                  # much straight-line travel
OBSTACLE_REJOIN_SPEED_MM_S = 60  # mm/s, straight-line speed for move 3
OBSTACLE_REJOIN_POLL_MS = 5     # sensor-check interval while driving straight
# All untested placeholders, same as every other numeric constant here.


def drive_until_black(sensor, max_distance_mm, speed_mm_s):
    """
    Move 3's straight-line hunt: drive dead ahead (no curvature) until
    `sensor` reads black or `max_distance_mm` of travel is used up,
    whichever comes first. Returns True if the line was found, False if
    the distance cap was hit first.

    Distance is tracked as a difference of two robot.distance() readings,
    never via robot.reset() -- design.md Sec10 forbids that, since other
    states (and rotate_to_heading()'s own gyro baseline) store absolute
    readings a reset would silently invalidate.
    """
    start_mm = robot.distance()
    robot.drive(speed_mm_s, 0)
    while True:
        if is_black(sensor.hsv()):
            robot.stop()
            return True
        if robot.distance() - start_mm >= max_distance_mm:
            robot.stop()
            return False
        wait(OBSTACLE_REJOIN_POLL_MS)


def _wrap180(deg):
    """Fold an angle into -180..180.

    Needed because hub.imu.heading() accumulates without wrapping -- after
    enough turns around the zone it can read several hundred degrees, and
    a raw subtraction against a fixed angle would then pick the wrong
    alignment entirely, or unwind those whole turns to reach it.
    """
    while deg > 180:
        deg -= 360
    while deg < -180:
        deg += 360
    return deg


def rotate_to_relative(base, rel_deg, speed_dps):
    """Rotate to `rel_deg` off `base`, always taking the short way round.

    rotate_to_heading() takes an ABSOLUTE heading, and hub.imu.heading()
    accumulates without wrapping, so `base + rel` can sit a whole
    revolution away from where the robot actually is even though it names
    the same direction. Handing that straight over would have the robot
    unwind the difference. Turning by the wrapped delta instead never
    moves more than 180 degrees.
    """
    current_rel = _wrap180(hub.imu.heading() - base)
    rotate_to_heading(0, hub.imu.heading() + _wrap180(rel_deg - current_rel),
                      speed_dps)


def rotate_to_heading(pivot_offset_mm, target_heading_deg, speed_dps):
    """
    Rotate about pivot_offset_mm (same signed convention as
    pivot_wheel_angles()/run_pivot(): negative = left, positive = right,
    0 = in-place spin) until hub.imu.heading() reaches target_heading_deg,
    then stop -- closed-loop via the gyro rather than a blind run_angle()
    command, since state O's rotations (up to a full 180-degree loop
    around the tower) are long enough for wheel slip to accumulate real
    heading error if left open-loop.

    Never calls hub.imu.reset_heading() -- like robot.distance() in
    hunt_for_branch(), this only ever reads the accumulated heading and
    compares it against an externally-supplied absolute target, the same
    "record and subtract, never reset" rule
    design.md Sec10 states for odometry, extended here to the gyro.

    Slows to ROTATE_CREEP_SPEED_DPS for the final ROTATE_SLOWDOWN_MARGIN_DEG
    of the turn rather than braking abruptly from full speed the instant
    the target is reached -- motor momentum plus this loop's own
    OBSTACLE_ROTATE_POLL_MS polling gap both add real overshoot past the
    target otherwise, worse the longer/faster the rotation was.
    """
    half_track = AXLE_TRACK_MM / 2
    r_left = pivot_offset_mm + half_track
    r_right = pivot_offset_mm - half_track
    max_abs_r = max(abs(r_left), abs(r_right))
    if max_abs_r == 0:
        return  # degenerate pivot -- nothing to do

    turn_sign = 1 if target_heading_deg > hub.imu.heading() else -1

    def _set_speed(magnitude):
        left_motor.run(turn_sign * magnitude * r_left / max_abs_r)
        right_motor.run(turn_sign * magnitude * r_right / max_abs_r)

    _set_speed(speed_dps)
    slowed = False

    while True:
        current = hub.imu.heading()
        if (turn_sign > 0 and current >= target_heading_deg) or \
           (turn_sign < 0 and current <= target_heading_deg):
            break

        if not slowed and abs(target_heading_deg - current) <= ROTATE_SLOWDOWN_MARGIN_DEG:
            _set_speed(ROTATE_CREEP_SPEED_DPS)
            slowed = True

        wait(OBSTACLE_ROTATE_POLL_MS)

    left_motor.stop()
    right_motor.stop()


# --- state functions ---------------------------------------------------------
#
# One function per state design.md defines. Each runs every tick while
# its state is active, and returns either a state letter (what to move
# to on the next button press) or None ("no transition available yet").
# Only follow_line() above is real; every state below is a stub except
# for calling it out.


def state_S():
    """Armed -- wait for the button; do not move (design.md Sec4)."""
    robot.stop()
    # TODO: -> F, once a real start-button wait (distinct from the debug
    # right-button step) is implemented.
    return None


def state_F():
    """Follow black line (design.md Sec4)."""
    global _green_streak, _line_lost_start_mm

    l_hsv, r_hsv = read_sensors()  # one hsv() per sensor per tick,
                                   # reused below for the green guard
    print("L h=%d s=%d v=%d | R h=%d s=%d v=%d" %
          (l_hsv.h, l_hsv.s, l_hsv.v, r_hsv.h, r_hsv.s, r_hsv.v))
    follow_line(l_hsv, r_hsv)

    # Guard 1, highest priority: green -> L / R (design.md Sec4/Sec6).
    # Detection and the state switch only, for now -- state_L()/state_R()
    # don't perform the actual turn yet (see their own TODOs), this just
    # switches into whichever one so the debug-stepped framework can be
    # exercised guard by guard.
    left_green = is_green(l_hsv)
    right_green = is_green(r_hsv)

    if left_green or right_green:
        _green_streak += 1
    else:
        _green_streak = 0

    if _green_streak >= GREEN_CONFIRM_SAMPLES:
        _green_streak = 0
        return "L" if left_green else "R"

    # Guard 2: obstacle -> O (design.md Sec4 "Guard ordering within the
    # loop"). state_O() re-reads the ultrasonic itself on entry rather
    # than trusting this exact reading, so nothing needs to be stashed
    # here beyond the transition itself.
    if ultrasonic_sensor.distance() < OBSTACLE_TRIGGER_MM:
        print("obstacle: detected within %d mm" % OBSTACLE_TRIGGER_MM)
        return "O"

    # Guard 3: both inner sensors dark -> X (design.md Sec4 "Guard
    # ordering within the loop"). Reuses is_black()/BLACK_VAL_MAX --
    # already the same "on the black line" test the obstacle/green-turn
    # rejoin hunts use -- rather than a separate threshold.
    if is_black(l_hsv) and is_black(r_hsv):
        return "X"

    # Guard 4, lowest priority: both inner sensors white, continuously,
    # for LINE_LOST_MM of travel -> B (design.md Sec4 "Guard ordering
    # within the loop"). Distance-anchored the same way state O's own
    # trigger-adjacent tracking works elsewhere in this file: record
    # where the continuous white streak began, only fire once travel
    # since then reaches the threshold, and reset the anchor the instant
    # either sensor stops reading white -- a brief gap (a corner, a
    # crossing) never accumulates toward it.
    both_white = is_white(l_hsv) and is_white(r_hsv)
    now_mm = robot.distance()
    if both_white:
        if _line_lost_start_mm is None:
            _line_lost_start_mm = now_mm
        elif now_mm - _line_lost_start_mm >= LINE_LOST_MM:
            _line_lost_start_mm = None
            print("line lost: both white for %d mm" % LINE_LOST_MM)
            return "B"
    else:
        _line_lost_start_mm = None

    return None


def state_W():
    """Follow white line -- same law as F, inverted error sign (design.md Sec4)."""
    # TODO: implement; guard -> M once both inner sensors read white.
    return None


INTERSECTION_CROSS_MM = 20      # move forward this far to clear the
                                 # black band before resuming F. Untested
                                 # placeholder.
INTERSECTION_CROSS_SPEED_MM_S = 60  # mm/s for the crossing move --
                                     # applied via robot.settings() below,
                                     # since straight() itself takes no
                                     # speed argument (confirmed against
                                     # Pybricks' own docs: straight()/
                                     # turn()/arc() use whatever
                                     # straight_speed/turn_rate was last
                                     # configured with settings(), unlike
                                     # drive(), which takes its own speed
                                     # directly every call).


def state_X():
    """Junction classifier (design.md Sec5) -- simplified: rather than
    design.md's fuller classifier (read the outer sensors, ask the
    camera if ambiguous, then dispatch to F/W/H), this always treats the
    band the same way -- drive straight through it for a fixed distance,
    then resume F."""
    # Cancel state_F()'s continuous drive() before taking the blocking
    # straight() move below -- same reasoning as state_L()/state_R()/
    # state_O().
    robot.stop()

    robot.settings(straight_speed=INTERSECTION_CROSS_SPEED_MM_S)
    robot.straight(INTERSECTION_CROSS_MM)  # DriveBase's own built-in
                                            # blocking straight-line move
                                            # -- no hand-rolled loop
                                            # needed, since this isn't
                                            # sensor-gated like the
                                            # obstacle/green-turn hunts

    print("intersection: crossed %d mm, resuming F" % INTERSECTION_CROSS_MM)
    # TODO: design.md's fuller Sec5 spec can also dispatch to W (case c,
    # black background ahead -- an inverted-tile seam) or H (cases f, g,
    # camera/hub-2 link unusable) instead of always resuming F; not
    # implemented here, this always assumes cases a/b/d/e (corner,
    # crossing, or recovery).
    return "F"


def state_M():
    """Seam classifier, W's counterpart to X (design.md Sec5)."""
    # TODO -> F (case a, d) / W (cases b, c) / H (cases e, f, g)
    return None


def state_L():
    """Green turn, left (design.md Sec6)."""
    # Cancel state_F()'s continuous drive() call before taking direct
    # control of the two motors inside run_pivot() -- otherwise
    # DriveBase's own control loop and the pivot's direct motor commands
    # would fight each other. Same reasoning as state_S()/state_H()'s
    # own robot.stop(), just immediately followed by the real action
    # here instead of staying parked.
    robot.stop()

    # PIVOT_OFFSET_MM/PIVOT_ANGLE_DEG are written right-turn-positive
    # (see their own comments). Left is that maneuver's mirror image,
    # not a separately-tuned one -- negating both turns a right pivot
    # into the equivalent left one, so only one side ever needs tuning
    # by hand and the other follows automatically.
    run_pivot(-PIVOT_OFFSET_MM, -PIVOT_ANGLE_DEG, PIVOT_SPEED_DPS)

    # Left saw the green, so the branch is being hunted for with the
    # RIGHT sensor (the far one) -- same pivot offset and direction
    # (both negated, matching run_pivot() just above) continued past
    # the initial blind angle until that sensor finds black or
    # HUNT_MAX_ANGLE_DEG runs out.
    hunt_for_branch(-PIVOT_OFFSET_MM, -1, right_sensor)

    # TODO: design.md Sec6's fuller maneuver also backs up before
    # pivoting -- not implemented yet. Always -> F regardless, whether
    # or not the branch was actually found (hunt_for_branch() gives up
    # silently past its angle cap rather than reporting failure).
    return "F"


def state_R():
    """Green turn, right -- mirror of L (design.md Sec6)."""
    robot.stop()  # see state_L()'s note -- cancels F's drive() first
    run_pivot(PIVOT_OFFSET_MM, PIVOT_ANGLE_DEG, PIVOT_SPEED_DPS)
    # Right saw the green -- hunt with the LEFT (far) sensor instead.
    hunt_for_branch(PIVOT_OFFSET_MM, 1, left_sensor)
    # TODO -- see state_L()'s TODO (backing up before the pivot).
    return "F"


def state_O():
    """Obstacle bypass -- circle the water tower using it as the pivot,
    gyro-gated (see this file's "obstacle bypass" section header for the
    full derivation of why each step is signed the way it is)."""
    # Cancel state_F()'s continuous drive() before taking direct motor
    # control inside rotate_to_heading() -- same reasoning as
    # state_L()/state_R().
    robot.stop()

    heading0 = hub.imu.heading()  # the orientation to return to at the end

    # Move 1: spin clockwise on the spot. The robot hasn't translated, so
    # this points it tangentially past the tower, which is now assumed to
    # be directly to its LEFT at OBSTACLE_PIVOT_RADIUS_MM -- a fixed
    # distance, not the ultrasonic's live reading at trigger time (see
    # that constant's own comment for why).
    rotate_to_heading(0, heading0 + OBSTACLE_SPIN_DEG, OBSTACLE_ROTATE_SPEED_DPS)

    # Move 2: half-circle around the assumed tower position as the pivot
    # (negative = left, at OBSTACLE_PIVOT_RADIUS_MM). The target heading
    # SUBTRACTS OBSTACLE_LOOP_DEG rather than adding it: pivoting around
    # a point on the robot's left, moving forward, curves the heading
    # back the OTHER way (counterclockwise) relative to move 1's
    # clockwise spin -- see this section's header comment for the full
    # derivation. The net effect is the robot loops around the tower and
    # comes out the other side, back on the original line of travel.
    rotate_to_heading(-OBSTACLE_PIVOT_RADIUS_MM,
                       heading0 + OBSTACLE_SPIN_DEG - OBSTACLE_LOOP_DEG,
                       OBSTACLE_ROTATE_SPEED_DPS)

    # Move 3: NOT a blind spin back to heading0 -- moves 1-2 are a fixed
    # blind pivot, but closing out the maneuver is sensor-gated instead
    # of trusting that geometry to have landed exactly back on the line.
    # Drive straight and hunt with the inner-RIGHT sensor (the one now
    # sweeping toward the line after the loop) until it reads black, or
    # give up -> H if OBSTACLE_REJOIN_MAX_MM passes without finding it.
    if drive_until_black(right_sensor, OBSTACLE_REJOIN_MAX_MM, OBSTACLE_REJOIN_SPEED_MM_S):
        print("obstacle: line reacquired")
        return "F"
    print("obstacle: rejoin distance exceeded")
    return "H"


ZONE_BACKUP_MM = LINE_LOST_MM-100           # move backward this far once B decides
                               # the line is genuinely gone, before
                               # flagging zone mode active. Untested
                               # placeholder.
ZONE_BACKUP_SPEED_MM_S = 60    # mm/s for the backup move -- applied via
                               # robot.settings() before straight(), same
                               # reasoning as INTERSECTION_CROSS_SPEED_MM_S


def state_B():
    """Zone check (design.md Sec6) -- simplified: recognise that the line
    is genuinely gone (guard 4 already confirmed both-white for
    LINE_LOST_MM), back up a fixed distance, flag zone mode active, then
    hand off to V. Design.md's fuller spec -- raise the camera, spin 360
    scanning for zone targets, dispatch to F/A/V/H depending on what's
    found -- is not implemented; this always assumes case d (zone
    confirmed, survey next) and goes straight to V."""
    global _zone_mode_active

    # Cancel state_F()'s continuous drive() before the blocking
    # straight() move below -- same reasoning as every other state that
    # takes direct/blocking control. No one-shot re-entry guard needed
    # here (unlike state_O()'s multi-tick equivalents once did) -- B now
    # always finishes by transitioning straight to V in the same call,
    # so it's never invoked again afterward to repeat the backup.
    robot.stop()
    robot.settings(straight_speed=ZONE_BACKUP_SPEED_MM_S)
    robot.straight(-ZONE_BACKUP_MM)  # negative = backward

    _zone_mode_active = True
    print("zone: mode activated, reversing complete -> V")
    return "V"


def state_H():
    """Give up -- full reset, wait for the button (design.md Sec4)."""
    robot.stop()
    # TODO: reset polarity/camera aim/hub-2 goal/lockouts/counters, then
    # -> F only, after a button press -- never to W.
    return None


# --- hub <-> camera link (PUPRemote), for state V's zone survey -------------
#
# Not needed by anything before this point in the file -- F/O/X/B never
# talk to the camera -- so it's introduced here rather than up top with
# the other hardware, right before the one state that actually uses it.
#
# Vendored from github.com/antonvh/PUPRemote (pupremote_hub.py, GPL),
# same class pybricks/hub_camera_test.py already carries and already
# confirmed end-to-end against real hardware -- copied in again here
# rather than imported, for the same reason that file gives: stable
# code.pybricks.com doesn't support Pybricks Code Beta's cross-file
# auto-bundling, so a plain `from pupremote_hub import PUPRemoteHub`
# would fail. Includes the self.port fix already applied to that copy
# (confirmed as a genuine upstream bug, not a vendoring mistake) -- see
# pybricks/hub_camera_test.py's own comment on it for the full story.
import ustruct as struct
from pybricks.tools import run_task
from micropython import const

_MAX_PKT = const(16)

_NAME = const(0)
_SIZE = const(1)
_TO_HUB_FORMAT = const(2)
_FROM_HUB_FORMAT = const(3)
_ARGS_TO_HUB = const(5)
_ARGS_FROM_HUB = const(6)
_CALLBACK = const(0)


class _PUPRemote:
    def __init__(self, max_packet_size=_MAX_PKT):
        self.commands = []
        self.modes = {}
        self.max_packet_size = max_packet_size

    def add_command(self, mode_name, to_hub_fmt="", from_hub_fmt="", command_type=_CALLBACK):
        if to_hub_fmt == "repr" or from_hub_fmt == "repr":
            msg_size = self.max_packet_size
            num_args_from_hub = -1
            num_args_to_hub = -1
        else:
            size_to_hub_fmt = struct.calcsize(to_hub_fmt)
            size_from_hub_fmt = struct.calcsize(from_hub_fmt)
            msg_size = max(size_to_hub_fmt, size_from_hub_fmt)
            num_args_to_hub = len(
                struct.unpack(to_hub_fmt, bytearray(struct.calcsize(to_hub_fmt)))
            )
            num_args_from_hub = len(
                struct.unpack(from_hub_fmt, bytearray(struct.calcsize(from_hub_fmt)))
            )

        assert msg_size <= self.max_packet_size, "Payload exceeds maximum packet size"
        self.commands.append({
            _NAME: mode_name,
            _TO_HUB_FORMAT: to_hub_fmt,
            _SIZE: msg_size,
            _ARGS_TO_HUB: num_args_to_hub,
        })
        if command_type == _CALLBACK:
            self.commands[-1][_FROM_HUB_FORMAT] = from_hub_fmt
            self.commands[-1][_ARGS_FROM_HUB] = num_args_from_hub

        self.modes[mode_name] = len(self.commands) - 1

    def decode(self, fmt, data):
        if fmt == "repr":
            clean = data.rstrip(b"\x00")
            return (eval(clean),) if clean else ("",)
        else:
            size = struct.calcsize(fmt)
            data = struct.unpack(fmt, data[:size])
        return data

    def encode(self, size, format, *argv):
        if format == "repr":
            s = bytes(repr(*argv), "UTF-8")
        else:
            s = struct.pack(format, *argv)
        assert len(s) <= size, "Payload exceeds maximum packet size"
        return s


class _PUPRemoteHub(_PUPRemote):
    def __init__(self, port, max_packet_size=_MAX_PKT):
        super().__init__(max_packet_size)
        if isinstance(port, str):
            port = eval("Port." + port)
        elif isinstance(port, int):
            port = eval("Port." + chr(64 + port))
        self.port = port
        try:
            self.pup_device = PUPDevice(port)
        except OSError:
            self.pup_device = None
            print("Check wiring and remote script. Unable to connect on ", self.port)
            raise

    def add_command(self, mode_name, to_hub_fmt="", from_hub_fmt="", command_type=_CALLBACK):
        super().add_command(mode_name, to_hub_fmt, from_hub_fmt, command_type)
        modes = self.pup_device.info()["modes"]
        n = len(self.commands) - 1
        assert len(self.commands) <= len(modes), "More commands than on remote side"
        assert mode_name == modes[n][0].rstrip(), (
            "Expected '{}' as mode {}, but got '{}'".format(modes[n][0].rstrip(), n, mode_name)
        )
        assert self.commands[-1][_SIZE] == modes[n][1], (
            "Different parameter size than on remote side. Check formats."
        )

    def call(self, mode_name, *argv, wait_ms=0):
        assert not run_task(), "Use 'call_multitask' instead of 'call', with multiple start blocks or multitask blocks"

        mode = self.modes[mode_name]
        size = self.commands[mode][_SIZE]

        if _FROM_HUB_FORMAT in self.commands[mode]:
            num_args = self.commands[mode][_ARGS_FROM_HUB]
            if num_args >= 0:
                assert len(argv) == num_args, (
                    "Expected {} argument(s) in call '{}'".format(num_args, mode_name)
                )
            self.pup_device.read(mode)
            payl = self.encode(size, self.commands[mode][_FROM_HUB_FORMAT], *argv)
            self.pup_device.write(
                mode,
                [((i + 128) & 0xFF) - 128 for i in tuple(payl + b"\x00" * (size - len(payl)))],
            )
            wait(wait_ms)

        data = self.pup_device.read(mode)
        raw_data = bytes([b if b >= 0 else b + 256 for b in data])
        result = self.decode(self.commands[mode][_TO_HUB_FORMAT], raw_data)
        return result[0] if len(result) == 1 else result


# --- the 'look' protocol ----------------------------------------------------
#
# Formats must match camera.py's own add_command exactly, and the whole
# question/echo dance below mirrors pybricks/hub_camera_test.py's ask(),
# which is the reference implementation this was checked against.
#
# REQUEST, 4 bytes: (question, seq, kind_filter, 0)
#   question    one of the QUESTION_* letters below; it also decides where
#               the camera aims, so there is no separate mode command any
#               more -- asking a zone question IS what tilts the camera up
#               for the zone, and asking a line question tilts it back down.
#   seq         1..255, bumped for every fresh question
#   kind_filter KIND_* to hunt one target type, or KIND_NONE for "nearest
#               of anything". Only 'Z' reads it.
#
# REPLY, 4 shorts: (echo, a, b, c)
#   'Z'         (echo, kind, bearing_deg, range_mm)
#   'J'/'M'/'A' (echo, line_ahead 0/1, angle_deg, coverage_percent)
#
# The echo is the point of the whole design. The camera adopts a new seq
# only once it is actually aimed where that question wants AND the servo
# has stopped, so a reply whose echo matches is guaranteed to have been
# computed from a settled frame taken after the request. Polling until
# the echo matches is therefore how the hub waits for the camera, and it
# replaces the old scheme of guessing a fixed hub-side delay.
camera = _PUPRemoteHub(Port.D)
camera.add_command("look", to_hub_fmt="hhhh", from_hub_fmt="4s")

QUESTION_JUNCTION = ord('J')    # black line continuing ahead? how much BLACK?
QUESTION_SEAM = ord('M')        # same, but measuring WHITE
QUESTION_AHEAD = ord('A')       # line anywhere ahead? raised aim
QUESTION_ZONE = ord('Z')        # nearest evacuation-zone target

KIND_NONE = 0
KIND_SPHERE_DARK = 1            # the black victim
KIND_SPHERE_LIGHT = 2           # a silver victim
KIND_POINT_GREEN = 3            # evacuation point for the living
KIND_POINT_RED = 4              # evacuation point for the dead

# A ball is either sphere kind. Points are NOT balls -- checking merely
# "kind != KIND_NONE" would have the survey charge at an evacuation
# corner, which is the one mistake this split exists to prevent.
SPHERE_KINDS = (KIND_SPHERE_DARK, KIND_SPHERE_LIGHT)

CAMERA_WAIT_MS = 6              # PUPRemote call timeout
CAMERA_POLL_GAP_MS = 30         # between polls while waiting for the echo
CAMERA_POLL_TRIES = 10          # enough when the aim is already correct

# An aim change costs the camera's TILT_MOVE_MS + TILT_SETTLE_MS before it
# will echo at all, so any question that re-points the servo needs a much
# bigger budget than one that doesn't. Same figures hub_camera_test.py
# uses, for the same reason.
CAMERA_AIM_SETTLE_MS = 1000
CAMERA_AIM_POLL_TRIES = CAMERA_AIM_SETTLE_MS // CAMERA_POLL_GAP_MS + 10

_camera_seq = 0


def ask(question, tries=CAMERA_POLL_TRIES, kind_filter=KIND_NONE):
    """
    Put one question to the camera and poll until it answers THAT question.

    Returns the raw (echo, a, b, c), or None if the camera never echoed
    the sequence byte within `tries` polls. Pass CAMERA_AIM_POLL_TRIES for
    any question that moves the aim ('A' and 'Z'), or the answer will be
    given up on while the servo is still travelling.
    """
    global _camera_seq
    _camera_seq = (_camera_seq + 1) & 0xFF
    request = bytes((question, _camera_seq, kind_filter, 0))

    for _ in range(tries):
        try:
            reply = camera.call("look", request, wait_ms=CAMERA_WAIT_MS)
        except Exception as error:
            print("camera: %s query failed: %s" % (chr(question), error))
            return None

        if reply[0] == _camera_seq:
            return reply
        wait(CAMERA_POLL_GAP_MS)

    print("camera: %s never echoed seq %d in %d polls"
          % (chr(question), _camera_seq, tries))
    return None


# --- zone survey (design.md Sec7 "SURVEY (state V)") -------------------------
#
# A staggered (step-and-check, not continuous) rotation scan: pivot in
# SCAN_ANGLES_DEG's fixed 30-degree increments from -90 to +90 relative
# to the heading the robot had on entering V, checking the camera for a
# sphere after each step, and stopping the instant one is found --
# dead or alive, either counts (design.md's own kind distinction is for
# a later state to act on, not for V to filter here).
SCAN_ANGLES_DEG = [-70, -60, -30, 0, 30, 60, 70]
SCAN_ROTATE_SPEED_DPS = 100     # deg/s for each pivot step and the final
                                # bearing-centring correction

SCAN_SETTLE_MS = 500           # pause after each pivot, before querying.
                                # Much of what this used to cover is now
                                # the protocol's job -- the echo is only
                                # given for a frame taken after the
                                # request, with the servo settled -- but it
                                # still buys the CHASSIS time to stop
                                # rocking after a pivot, which the camera
                                # has no way to know about. Probably
                                # reducible a long way from 1500 now; worth
                                # retuning once the new link is proven.
# All untested placeholders, same as every other numeric constant here.


def query_zone_camera(kind_filter=KIND_NONE):
    """
    Ask for the nearest zone target and return (kind, bearing_deg,
    range_mm), or None if the camera never answered.

    Uses CAMERA_AIM_POLL_TRIES because 'Z' re-points the camera.

    kind_filter defaults to KIND_NONE, "nearest of anything", which is
    what the ball search wants: the filter only takes ONE kind, so
    covering both sphere kinds would mean asking twice. Pass a specific
    KIND_* when exactly one target type will do -- the evacuation-point
    search does, and it matters there, because filtering in the camera
    means a wrong-coloured point standing nearer cannot mask the right
    one. Checking the kind after the fact could not recover that.
    """
    reply = ask(QUESTION_ZONE, CAMERA_AIM_POLL_TRIES, kind_filter)
    if reply is None:
        return None
    _echo, kind, bearing, range_mm = reply
    return kind, bearing, range_mm


# --- closing the loop on bearing ---------------------------------------------
#
# One continuous slow rotation toward the ball, watching the bearing as it
# goes and stopping when it reaches zero -- rather than the scan's old
# one-shot "pivot by whatever bearing was measured once" correction, which
# was only ever as good as ONE stale reading times ONE imperfect pivot.
#
# Closing the loop is insensitive to error in both, and -- the part that
# really matters here -- to SCALE error in the bearing itself.
# CAMERA_HFOV_DEG (camera.py) is still an unmeasured placeholder, so if
# it's off by 20%, every bearing read here is off by 20%. A one-shot
# correction inherits that directly; this still stops in the right place,
# because only the bearing's SIGN has to be right for it to work.
CENTRE_TOLERANCE_DEG = 7      # |bearing| at or below this counts as centred

CENTRE_ROTATE_SPEED_DPS = 40  # can be brisk: the robot is STOPPED every
                              # time it measures, so rotation speed no
                              # longer blurs or stales the reading the way
                              # it did when this turned and polled at once.
CENTRE_MAX_STEP_DEG = 45      # cap on any single correction, so one wild
                              # bearing reading can't swing the robot
                              # straight past the ball it was aiming at
CENTRE_MAX_STEPS = 12         # give up rather than shuffle forever if the
                              # bearing never converges (ball rolled out
                              # of frame, bearing sign inverted, camera
                              # stuck on a stale frame). Each step costs a
                              # SCAN_SETTLE_MS pause, so this is a real
                              # time budget as well as a safety net -- 12
                              # of them is ~18s worst case. Back to 12 to
                              # match the version that was tracking the
                              # ball reliably; drop it again if a genuine
                              # competition run can't spare that long on a
                              # ball it is never going to converge on.


def centre_on_target(kinds, kind_filter=KIND_NONE):
    """
    Turn toward a zone target in steps, STOPPING to measure between each,
    until the camera reports it within CENTRE_TOLERANCE_DEG of dead ahead.

    `kinds` is which KIND_* values count as the thing being centred on,
    and `kind_filter` is passed to the camera so it can narrow its own
    search -- see query_zone_camera() for why filtering there beats
    filtering here.

    Returns the final (kind, bearing, range_mm) reading once centred, or
    None if the target was lost, the camera stopped answering, or the
    step budget ran out. A reading is truthy and None is not, so callers
    can treat this as a plain success test and still get the range.

    Stop-and-measure, not measure-while-turning, and that distinction is
    the whole point. SCAN_SETTLE_MS in this same file is 1500ms -- that is
    how long the scan was found to need, standing still, before the camera
    returns a detection worth trusting. An earlier version of this rotated
    continuously and polled as it went, which gave the camera no still
    time at all: it reported no ball and this bailed out with
    "ball lost mid-turn" every time. Rotation speed was never the problem
    and slowing it down did not fix it -- the camera needs stillness, not
    gentleness.

    Each correction is the full measured bearing (clamped to
    CENTRE_MAX_STEP_DEG) rather than a small fixed nudge, because every
    measurement costs that 1500ms settle: converging in one or two big
    steps is far quicker than creeping there in ten small ones.

    Returns True once centred, False if the ball was lost, the camera
    stopped answering, or the step budget ran out.
    """
    for step in range(CENTRE_MAX_STEPS):
        # Always measured stationary -- on the first pass the robot is
        # still settled from the scan's own pause, and on every later pass
        # from this loop's own settle after rotating.
        result = query_zone_camera(kind_filter)
        if result is None:
            print("centre: camera not responding")
            return None

        kind, bearing, dist = result

        if kind not in kinds:
            print("centre: no target visible at step %d (kind=%d)" % (step, kind))
            return None

        if abs(bearing) <= CENTRE_TOLERANCE_DEG:
            print("centre: centred at bearing %d after %d step(s)" % (bearing, step))
            return result

        # Positive bearing = ball right of centre -> turn right
        # (clockwise), which is +1 in this file's convention throughout.
        correction = max(-CENTRE_MAX_STEP_DEG, min(CENTRE_MAX_STEP_DEG, bearing))
        print("centre: bearing %d, correcting by %d" % (bearing, correction))
        rotate_to_heading(0, hub.imu.heading() + correction, CENTRE_ROTATE_SPEED_DPS)

        # The settle that makes the NEXT measurement trustworthy -- the
        # entire reason this loop stops instead of turning continuously.
        wait(SCAN_SETTLE_MS)

    print("centre: gave up after %d steps" % CENTRE_MAX_STEPS)
    return None


def centre_on_ball():
    """Centre on a sphere -- the ball search's entry point, unchanged."""
    return centre_on_target(SPHERE_KINDS)


def state_V():
    """SURVEY (design.md Sec7) -- staggered rotation scan for a sphere,
    then centre on its reported bearing before handing off to A."""
    # Cancel state_F()'s continuous drive() before taking direct motor
    # control inside rotate_to_heading() -- same reasoning as every
    # other state that takes over the motors directly.
    robot.stop()

    # No mode to switch into any more: asking QUESTION_ZONE is itself
    # what re-aims the camera, and its echo is withheld until that aim
    # has settled. This first ask is still worth making before the scan
    # starts, though -- it pays the one-off aim cost here rather than
    # inside the first scan step, where it would look like that angle
    # being slow to answer.
    if query_zone_camera() is None:
        print("survey: camera did not answer the first zone question, "
              "scanning anyway")

    global _zone_heading0

    # Anchored ONCE per zone visit, not per survey. state_D() measures its
    # sweep off this, and the post-delivery scan below measures a back
    # bearing off it too -- re-reading it on a later survey would reset
    # the reference to wherever the last delivery left the robot standing
    # and quietly invalidate both.
    if _zone_heading0 is None:
        _zone_heading0 = hub.imu.heading()
        print("survey: zone entry heading %d" % _zone_heading0)
    base = _zone_heading0

    # Where the sweep is centred. The first survey of a visit looks
    # straight out from the entry heading. Every survey AFTER a delivery
    # looks back out of the corner the ball was just placed in: state_T()
    # squared up to one of the zone's edges to release, so facing 180
    # from that points back across the zone, where any remaining victims
    # are. Scanning +/-70 about the old entry heading instead would spend
    # half the sweep pointed into the wall just delivered to.
    if _deposit_align_deg is None:
        centre = 0
    else:
        centre = _wrap180(_deposit_align_deg + 180)
        print("survey: last delivery squared to %d, so sweeping about %d"
              % (_deposit_align_deg, centre))

    for angle in SCAN_ANGLES_DEG:
        rotate_to_relative(base, centre + angle, SCAN_ROTATE_SPEED_DPS)

        # Diagnostic: requested vs actually-reached heading (relative to
        # base), so a mismatch between the two is visible directly
        # rather than inferred from robot behaviour alone. actual should
        # match angle closely (within ROTATE_SLOWDOWN_MARGIN_DEG-ish); a
        # large or systematic gap here points at rotate_to_heading()
        # itself (or the gyro/motors), not at this loop or SCAN_ANGLES_DEG.
        actual = _wrap180(hub.imu.heading() - base)
        print("survey: requested %d deg, actual %d deg"
              % (_wrap180(centre + angle), actual))

        wait(SCAN_SETTLE_MS)  # let the camera grab a fresh frame at this heading

        result = query_zone_camera()
        if result is None:
            print("survey: camera not responding at %d deg" % angle)
            continue

        kind, bearing, dist = result
        print("survey: camera says kind=%d bearing=%d range=%d" %
              (kind, bearing, dist))

        # Spheres only. kind 3/4 are the evacuation points, which are
        # zone targets but not victims -- driving at one would be a bug,
        # not a rescue.
        if kind in SPHERE_KINDS:  # dark or light, either counts
            print("survey: sphere found at scan angle %d (kind=%d bearing=%d dist=%d)" %
                  (angle, kind, bearing, dist))

            if centre_on_ball():
                print("survey: centred, heading %d deg relative to entry" %
                      _wrap180(hub.imu.heading() - base))
                return "A"

            # Couldn't lock on -- ball lost mid-correction, or the bearing
            # never converged. Carry on scanning from the next angle
            # rather than handing A a target that isn't there; the scan's
            # own targets are absolute (base + centre + angle), so whatever
            # rotation centring already did doesn't throw the rest off.
            print("survey: centring failed, resuming scan")

    print("survey: no sphere found after full scan")
    # TODO -> K (none left, or time short) / Q (scan failed) per
    # design.md Sec7 -- neither implemented yet, so this just stays in V.
    return None


# --- approach (design.md Sec7 "APPROACH (state A)") --------------------------

HUB2_GOAL_TIMEOUT_MS = 5000   # give up on a hub-2 goal after this long.
                              # Only reachable with HUB2_ENABLED on; a hub
                              # that never answers and one stuck MOVING
                              # forever look identical from here, since
                              # both simply fail to ever say DONE.
HUB2_POLL_MS = 10             # matches hub 2's own broadcast cadence

APPROACH_SPEED_MM_S = 80      # mm/s for the drive-up

APPROACH_CORRECTION_MM = -60  # SIGNED adjustment added to the camera's
                              # reported range to get the distance actually
                              # driven. Negative because the camera reports
                              # range from its own mounting point, but the
                              # robot wants to stop with the ball inside the
                              # claw -- which sits ahead of the wheel centre
                              # that robot.straight() moves. Tune against
                              # real behaviour: stops short -> make this LESS
                              # negative; drives into/past the ball -> MORE
                              # negative. Untested placeholder.

_hub2_seq = 0  # incremented per goal, so hub 2 can tell a fresh request
               # from a resend of the one it is already working on


def hub2_goal(goal, timeout_ms=HUB2_GOAL_TIMEOUT_MS):
    """
    Send one goal to hub 2 and block until it echoes that goal DONE.
    Returns True on DONE, False on FAULT or timeout.

    Returns True immediately when HUB2_ENABLED is off -- "nothing is
    blocking you", so callers carry on with the rest of their sequence
    rather than treating a deliberately-absent hub 2 as a failure.

    One broadcast is enough, not a resend loop: Pybricks' broadcast()
    sets what this hub continuously advertises and keeps advertising it
    until changed, so the goal stays on the air as a level for the whole
    wait -- the same "levels, not verbs" reasoning design.md Sec3 gives
    for the link as a whole.

    Matching on seq is load-bearing. Hub 2 rebroadcasts its state every
    10ms regardless, so without the seq check this would immediately read
    a DONE left over from the PREVIOUS goal and return before the new one
    had moved anything at all.
    """
    if not HUB2_ENABLED:
        print("hub2: disabled, skipping goal %s" % goal)
        return True

    global _hub2_seq
    _hub2_seq += 1
    seq = _hub2_seq
    hub.ble.broadcast((seq, goal))

    timer = StopWatch()
    while timer.time() < timeout_ms:
        state = hub.ble.observe(STATE_CHANNEL)
        # observe() returns None when nothing has been heard recently;
        # anything carrying a stale seq is left over from a previous goal.
        if state is not None and len(state) >= 2 and state[0] == seq:
            phase = state[1]
            if phase == PHASE_DONE:
                return True
            if phase == PHASE_FAULT:
                print("hub2: goal %s reported FAULT" % goal)
                return False
        wait(HUB2_POLL_MS)

    print("hub2: goal %s timed out after %d ms" % (goal, timeout_ms))
    return False


def _usable_range(result):
    """True if a query_zone_camera() reply carries both a ball and a range.

    result is (kind, bearing, range_mm) -- a ball means a SPHERE kind,
    not merely "something was seen", since an evacuation point reports a
    perfectly good range too and is not a thing to drive into.
    """
    return result is not None and result[0] in SPHERE_KINDS and result[2] > 0


def state_A():
    """APPROACH (design.md Sec7) -- lower the claw, then drive the
    camera-reported range to the ball and hand off to C."""
    robot.stop()

    # ONE range measurement, taken here, while the robot is still sitting
    # where state_V() centred it and before anything else has happened.
    # This is the only look the approach takes.
    #
    # It deliberately does NOT re-confirm after the claw is lowered. That
    # re-confirmation is what made this state loop: a single empty read
    # after the claw step sent it to V, V re-found the same ball and
    # handed it straight back, and round it went. The ball has not moved
    # between the two reads -- only the robot's ability to see it can
    # have changed -- so the earlier measurement is the better number to
    # trust anyway, not merely the safer one.
    result = query_zone_camera()

    if result is None:
        print("approach: camera not responding")
        return "Q"

    kind, bearing, dist = result
    if kind not in SPHERE_KINDS:
        print("approach: no ball to approach (kind=%d)" % kind)
        return "V"  # design.md Sec7: A -> V when the target is lost
    if dist <= 0:
        print("approach: no usable range to the ball")
        return "V"

    # Remember WHICH victim this is, because after the capture nothing
    # can tell any more -- the ball is inside the claw and out of the
    # camera's view, and a dark sphere and a light one look identical
    # from the outside of a closed claw. state_D() needs it to pick the
    # matching evacuation point, and this reading is the last confident
    # look anything gets at it.
    global _captured_kind, _captured_heading
    _captured_kind = kind

    # And WHERE it was, as a bearing off the zone entry heading. Nothing
    # rotates between here and the capture -- the drive below is
    # straight, and state_C() only works the claw -- so this is the
    # orientation the ball is captured at. state_D() reads its sign to
    # work out which side of the zone the robot has ended up on.
    if _zone_heading0 is None:
        _captured_heading = None
    else:
        _captured_heading = hub.imu.heading() - _zone_heading0

    print("approach: carrying kind=%d (%s) at %s deg off entry" %
          (kind, "light/living" if kind == KIND_SPHERE_LIGHT else "dark/dead",
           "?" if _captured_heading is None else "%d" % _captured_heading))

    # Claw down before a wheel turns, but AFTER the range is known --
    # lowering it mid-drive would swing the lifter through the space the
    # ball occupies instead of arriving with it already open around it.
    # Whatever the claw does to the camera's view from here on no longer
    # matters, because nothing looks again.
    if not hub2_goal(GOAL_LOWER):
        # design.md Sec7 says A -> Q when blocked, and that is what this
        # should become once state Q actually does something. While Q is
        # still a blank stub, dead-ending there strands the robot with no
        # diagnosis and looks exactly like "state A did nothing" -- so
        # press on and let the approach itself be observed instead.
        print("approach: WARNING claw would not lower, approaching anyway")

    travel = dist + APPROACH_CORRECTION_MM
    print("approach: range %d mm, driving %d mm (correction %d)" %
          (dist, travel, APPROACH_CORRECTION_MM))

    if travel > 0:
        robot.settings(straight_speed=APPROACH_SPEED_MM_S)
        robot.straight(travel)
    else:
        # The correction alone already covers the whole measured range,
        # so the ball should be in the claw already. Driving a negative
        # distance here would reverse away from it.
        print("approach: already within the correction distance, not driving")

    return "C"


def state_C():
    """CAPTURE (design.md Sec7) -- close the claw on the ball, then raise
    the lifter to carry height."""
    # Nothing here drives, but state_A()'s straight() leaves the drive
    # base holding position -- stop it rather than leaving the wheels
    # energised through two mechanical moves.
    robot.stop()

    # Close first, raise second, and never overlapped: hub2_goal() blocks
    # until each is DONE. Raising a claw that is still closing would lift
    # past the ball while the jaws are still open around it.
    if not hub2_goal(GOAL_CLOSE):
        print("capture: claw would not close")
        return "Q"  # design.md Sec7: C -> Q on hub 2 FAULT or timeout

    if not hub2_goal(GOAL_RAISE):
        print("capture: lifter would not raise")
        return "Q"

    print("capture: closed and raised")
    # TODO: this reports success on "both moves completed", which is not
    # the same as "a ball is actually held". Hub 2 already sends a grip
    # verdict -- GRIP_EMPTY / GRIP_HOLDING / GRIP_BLOCKED -- in field 2
    # of the very packet hub2_goal() reads the DONE phase from, and it
    # currently throws that field away. Reading it is what would let this
    # take design.md Sec7's other two exits: A when the grip failed but
    # the ball is still visible, V when it failed and the ball is gone.
    # Until then a closed-on-nothing claw still reports captured.
    return "D"


# --- deliver (design.md Sec7 "DELIVER (state D)") ----------------------------

# A much wider sweep than the ball search: the evacuation points sit on
# the zone walls rather than out on the floor, so after a capture has
# dragged the robot off to one side they can easily be behind it. 45 deg
# steps against the camera's ~58 deg horizontal field leave an overlap at
# every step, and the list ends exactly on +135 -- which is where the
# reposition below assumes the robot is left standing after a failure.
DELIVER_SCAN_ANGLES_DEG = [-135, -90, -45, 0, 45, 90, 135]

# Sweeps before giving up: the first from wherever the capture left the
# robot, then one more from the middle of the zone after repositioning.
DELIVER_SWEEPS = 3

DELIVER_MIDDLE_TURN_DEG = 90    # face straight across the zone to cross it
DELIVER_MIDDLE_DRIVE_MM = 300   # 30 cm toward the middle
DELIVER_MIDDLE_SPEED_MM_S = 300

# Kinds 3 and 4 -- the evacuation points themselves, as opposed to the
# spheres SPHERE_KINDS covers.
POINT_KINDS = (KIND_POINT_GREEN, KIND_POINT_RED)

# Which point each victim goes to, straight off camera.py's own
# definitions: KIND_SPHERE_LIGHT is "a silver victim" and
# KIND_POINT_GREEN is "evacuation point for the living"; KIND_SPHERE_DARK
# is "the black victim" and KIND_POINT_RED is "for the dead". Delivering
# to the wrong one is not a crash, it is a silently lost score, which is
# why the pairing lives in one named place rather than in an if.
POINT_FOR_SPHERE = {
    KIND_SPHERE_LIGHT: KIND_POINT_GREEN,   # silver -> living -> green
    KIND_SPHERE_DARK: KIND_POINT_RED,      # black  -> dead   -> red
}


def sweep_for_point(base, kind_filter, acceptable):
    """
    One full DELIVER_SCAN_ANGLES_DEG sweep, measured off `base`.

    Returns (angle, kind, bearing, range_mm) for the first acceptable
    target seen, or None having finished the sweep. Either way the robot
    is left pointing at the last angle in the list, which the caller
    relies on when deciding where to reposition from.
    """
    for angle in DELIVER_SCAN_ANGLES_DEG:
        rotate_to_heading(0, base + angle, SCAN_ROTATE_SPEED_DPS)
        wait(SCAN_SETTLE_MS)

        result = query_zone_camera(kind_filter)
        if result is None:
            print("deliver: camera not responding at %d deg" % angle)
            continue

        kind, bearing, dist = result
        print("deliver: %d deg -- kind=%d bearing=%d range=%d"
              % (angle, kind, bearing, dist))

        # Still checked even when the camera was filtering, because the
        # unknown-kind path asks with no filter at all.
        if kind in acceptable:
            return angle, kind, bearing, dist

    return None


def state_D():
    """DELIVER (design.md Sec7) -- find the evacuation point matching the
    captured victim, crossing to the middle of the zone and looking again
    if the first sweep cannot see it."""
    robot.stop()

    # Why not sweep from the current heading: by now the robot has been
    # turned to face a ball, driven at it and captured it, so where it
    # points is "wherever that ball happened to be". state_V() recorded
    # the heading the zone was ENTERED on, and every angle below is
    # measured off that instead.
    if _zone_heading0 is None:
        # Only reachable if D is entered without V having run -- debug
        # stepping straight into it, say. Fall back rather than crash.
        print("deliver: no zone entry heading recorded, sweeping from here")
        base = hub.imu.heading()
    else:
        base = _zone_heading0

    # Which point this victim belongs at, from the kind state_A() recorded
    # before capturing it. Asking the camera to filter on that kind beats
    # sifting the replies here: the camera reports only the NEAREST target
    # it considers, so without the filter a wrong-coloured point standing
    # closer would mask the right one entirely, and no amount of checking
    # afterwards could recover it.
    wanted = POINT_FOR_SPHERE.get(_captured_kind)
    if wanted is None:
        # D entered without A having recorded a kind -- debug stepping,
        # or a capture that never went through the approach. Take either
        # point rather than refusing to deliver at all.
        print("deliver: captured kind unknown, accepting either point")
        kind_filter = KIND_NONE
        acceptable = POINT_KINDS
    else:
        kind_filter = wanted
        acceptable = (wanted,)
        print("deliver: carrying kind=%d, looking for point kind=%d (%s)"
              % (_captured_kind, wanted,
                 "green/living" if wanted == KIND_POINT_GREEN else "red/dead"))

    for sweep in range(DELIVER_SWEEPS):
        print("deliver: sweep %d of %d, %d to %d deg off entry heading %d"
              % (sweep + 1, DELIVER_SWEEPS, DELIVER_SCAN_ANGLES_DEG[0],
                 DELIVER_SCAN_ANGLES_DEG[-1], base))

        found = sweep_for_point(base, kind_filter, acceptable)
        if found is not None:
            angle, kind, bearing, dist = found
            print("deliver: evacuation point found at %d deg "
                  "(kind=%d bearing=%d range=%d)"
                  % (angle, kind, bearing, dist))
            # TODO: the delivery itself is still blank. design.md Sec7
            # has D arriving at the triangle before T deposits, which
            # needs the centre-and-approach pair state_V()/state_A()
            # already do for a ball. Going straight to T deposits at
            # whatever range the point was spotted from.
            return "T"

        if sweep == DELIVER_SWEEPS - 1:
            break

        # --- nothing found: cross to the middle and look again ---------
        #
        # Which way the middle is follows from where the capture left the
        # robot. A ball found to the LEFT of the entry heading was
        # chased leftward, so the robot is now on the zone's left side
        # and the middle lies to its right, and vice versa. Turning to
        # base +/- 90 points it straight ACROSS the zone, so driving
        # forward from there translates it sideways rather than deeper
        # in.
        if _captured_heading is None:
            # No recorded capture bearing (D entered without A). Pick a
            # side rather than stalling, and say that it is a guess.
            on_left = False
            print("deliver: no capture bearing recorded, guessing right side")
        else:
            # Exactly 0 counts as the right side -- an arbitrary
            # tie-break for a robot that is already centred, where
            # either direction is as good as the other.
            on_left = _captured_heading < 0

        across = DELIVER_MIDDLE_TURN_DEG if on_left else -DELIVER_MIDDLE_TURN_DEG
        print("deliver: nothing found; on the %s side, crossing %d deg "
              "and driving %d mm toward the middle"
              % ("left" if on_left else "right", across, DELIVER_MIDDLE_DRIVE_MM))

        rotate_to_heading(0, base + across, SCAN_ROTATE_SPEED_DPS)
        robot.settings(straight_speed=DELIVER_MIDDLE_SPEED_MM_S)
        robot.straight(DELIVER_MIDDLE_DRIVE_MM)
        robot.stop()

        # The sweep angles below are still measured off `base`. Driving
        # moved the robot but not its heading reference, so the same
        # absolute targets remain correct from the new position.

    print("deliver: no evacuation point found after %d sweep(s)" % DELIVER_SWEEPS)
    return "V"  # design.md Sec7: D -> V when the triangle is missing


# --- deposit (design.md Sec7 "DEPOSIT (state T)") ----------------------------

DEPOSIT_SPEED_MM_S = 150       # mm/s for the drive up to the zone

DEPOSIT_CORRECTION_MM = -80   # SIGNED, added to the camera's reported range
                              # exactly as APPROACH_CORRECTION_MM is. More
                              # negative than that one because the robot
                              # must stop SHORT of the zone wall with the
                              # ball overhanging it, not drive its own
                              # wheels up to where the target was seen.
                              # Untested placeholder.

DEPOSIT_BACKOFF_MM = 50      # reversed after releasing, BEFORE anything
                              # rotates. The drive-up deliberately parks
                              # the robot overhanging the zone wall, which
                              # is exactly the wrong place to start
                              # spinning: a survey pivot from there sweeps
                              # the chassis straight into it. Untested
                              # placeholder.

DELIVERIES_TO_EGRESS = 3      # victims to deliver before giving up on the
                              # zone and heading for the exit

# The zone is a triangle in a corner, so its two straight edges run at
# 45 degrees to the entry heading. Squaring up to one of them before
# opening the claw is what puts the ball over the zone rather than over
# the wall's lip, where it can roll back out.
DEPOSIT_ALIGN_ANGLES_DEG = (-135, -45, 45, 135)


def state_T():
    """DEPOSIT (design.md Sec7) -- NOT "green turn"; that's L/R above.

    Drive up to the evacuation point, square up to the nearest of the
    zone's straight edges, and only then open the claw.
    """
    robot.stop()

    base = _zone_heading0 if _zone_heading0 is not None else hub.imu.heading()

    # The same point this victim belongs at that state_D() went looking for.
    wanted = POINT_FOR_SPHERE.get(_captured_kind)
    if wanted is None:
        kind_filter = KIND_NONE
        acceptable = POINT_KINDS
    else:
        kind_filter = wanted
        acceptable = (wanted,)

    # state_D() stopped at the scan angle it spotted the point from, with
    # the point still off to one side of that. Centre properly before
    # driving, or the drive-up runs at an angle and misses.
    reading = centre_on_target(acceptable, kind_filter)
    if reading is None:
        print("deposit: lost the evacuation point while centring")
        return "D"  # back to the sweep that found it in the first place

    kind, bearing, dist = reading
    travel = dist + DEPOSIT_CORRECTION_MM
    print("deposit: point kind=%d at %d mm, driving %d mm (correction %d)"
          % (kind, dist, travel, DEPOSIT_CORRECTION_MM))

    if travel > 0:
        robot.settings(straight_speed=DEPOSIT_SPEED_MM_S)
        robot.straight(travel)
        robot.stop()
    else:
        print("deposit: already within the correction distance, not driving")

    # --- square up to the nearest zone edge ---------------------------
    #
    # Driving at the point leaves the robot pointing at wherever in the
    # triangle the camera found it, which is not square to anything. The
    # ball is released over the front of the robot, so an off-square
    # release drops it over a corner or over the wall's lip. Turning to
    # whichever edge angle is nearest costs at most 45 degrees.
    rel = _wrap180(hub.imu.heading() - base)

    # Explicit loop rather than min(key=...) -- the comparison is on the
    # WRAPPED difference, not the raw one, and spelling that out avoids
    # relying on keyword support in this MicroPython build.
    best = DEPOSIT_ALIGN_ANGLES_DEG[0]
    best_gap = abs(_wrap180(best - rel))
    for candidate in DEPOSIT_ALIGN_ANGLES_DEG[1:]:
        gap = abs(_wrap180(candidate - rel))
        if gap < best_gap:
            best, best_gap = candidate, gap

    # Turn by the WRAPPED delta rather than to an absolute base + best:
    # with an accumulated heading those can differ by whole revolutions,
    # and the absolute form would spin the robot round to unwind them.
    turn = _wrap180(best - rel)
    print("deposit: at %d deg off entry, squaring to %d (turning %+d)"
          % (rel, best, turn))
    rotate_to_heading(0, hub.imu.heading() + turn, SCAN_ROTATE_SPEED_DPS)

    # Remembered so the NEXT survey knows which way to look: the back
    # bearing of this edge is where any remaining victims are.
    global _deposit_align_deg
    _deposit_align_deg = best

    # --- release ------------------------------------------------------
    #
    # Open only, with the lifter left at the carry height state_C() put
    # it at -- design.md Sec10's note that the release position has to
    # clear the 60mm wall is exactly why the ball is dropped in from
    # above rather than lowered first.
    if not hub2_goal(GOAL_OPEN):
        print("deposit: claw would not open")
        return "Q"  # design.md Sec7: T -> Q on hub 2 FAULT or timeout

    # Back off BEFORE returning, not at the start of the next state: the
    # robot is parked overhanging the zone wall, and whatever comes next
    # begins by rotating. Reversing here means every exit from this state
    # leaves the robot somewhere it can safely turn.
    print("deposit: released, reversing %d mm clear of the zone"
          % DEPOSIT_BACKOFF_MM)
    robot.settings(straight_speed=DEPOSIT_SPEED_MM_S)
    robot.straight(-DEPOSIT_BACKOFF_MM)
    robot.stop()

    global _delivered_count
    _delivered_count += 1
    print("deposit: %d of %d delivered" % (_delivered_count, DELIVERIES_TO_EGRESS))

    # TODO: design.md Sec7 also has T -> D when the release failed and the
    # ball is still held. Telling that apart needs hub 2's grip verdict,
    # which hub2_goal() still discards -- the same gap state_C() has. Note
    # that gap feeds the count above too: a release that silently failed
    # still counts as a delivery here.
    if _delivered_count >= DELIVERIES_TO_EGRESS:
        print("deposit: all %d delivered, heading for the exit"
              % DELIVERIES_TO_EGRESS)
        return "K"

    return "V"  # released -- go find the next victim


def state_K():
    """EGRESS (design.md Sec7)."""
    # TODO -> F (out of the zone, line reacquired, +20) / Q (cannot find
    #         the exit)
    return None


def state_Q():
    """RECOVER (design.md Sec7)."""
    # TODO -> V (resume the search) / K (give up rescuing, try to leave)
    return None


STATE_FUNCTIONS = {
    "S": state_S, "F": state_F, "W": state_W, "X": state_X, "M": state_M,
    "L": state_L, "R": state_R, "O": state_O, "B": state_B, "H": state_H,
    "V": state_V, "A": state_A, "C": state_C, "D": state_D, "T": state_T,
    "K": state_K, "Q": state_Q,
}

# design.md Sec7's zone states -- everything else (line-following and its
# own guards/turns/bypasses) is a "line" state, where the camera should
# be tilted down in LINE mode (design.md Sec4/5), not left in whatever
# mode state_V()'s own scan last put it in.
ZONE_STATES = {"V", "A", "C", "D", "T", "K", "Q"}


def sync_camera_line_mode():
    """
    Point the camera back down for line work, by asking it a line
    question -- there is no mode command any more, so the question IS
    the aim. The answer is discarded; only the re-aiming matters here.

    Called once at the moment the FSM crosses from a zone state back
    into a line state, never per tick: this blocks for as long as the
    aim takes to settle, which state_F()'s 10ms loop could not absorb.
    """
    ask(QUESTION_JUNCTION, CAMERA_AIM_POLL_TRIES)

# --- debug-stepped main loop -------------------------------------------------

# Master switch for the whole arm-gate mechanism this file's header
# describes. True (bench/testing): every freshly-entered state starts
# unarmed and needs one right-button press before its function runs, as
# described below. False (normal/competition running): that gate is
# skipped entirely -- every state runs immediately and unattended the
# instant it's entered, and transitions chain straight into each other
# with no button presses at all. Flip this and reflash; it isn't wired
# to a live button, so it can't be changed without stopping the program.
DEBUG_MODE = True

state = "F"  # starts directly in F rather than S, since S's real
             # start-button wait isn't implemented yet either
was_pressed = False
_green_streak = 0  # state_F()'s green-guard debounce counter
_line_lost_start_mm = None  # state_F()'s guard-4 distance anchor (None
                            # while not currently both-white)
_zone_heading0 = None      # heading the zone survey started from, recorded
                           # by state_V() and reused by state_D()
_captured_kind = KIND_NONE  # which sphere kind state_A() drove at, so
                            # state_D() knows which point to deliver to
_delivered_count = 0        # victims released so far; at
                            # DELIVERIES_TO_EGRESS the zone work is done
_deposit_align_deg = None   # zone edge state_T() last squared up to, so the
                            # next survey can sweep about its back bearing
_captured_heading = None    # bearing off the zone entry heading the ball
                            # was captured at; its SIGN tells state_D()
                            # which side of the zone the robot is on
_zone_mode_active = False  # state_B()'s one-shot entry guard -- True
                           # once the backup move has run once

# Per-state arm gate, not a per-transition one. Only meaningful when
# DEBUG_MODE is True: every freshly-entered state (including F on the
# very first tick) starts unarmed -- paused, not even ticked -- until
# one right-button press arms it. Once armed, the state function runs
# every tick unattended and transitions happen automatically the instant
# it returns a state letter; no further button presses are needed for
# that. The new state then starts unarmed again, same as any other
# entry. When DEBUG_MODE is False this is forced True every tick below,
# so it never gates anything.
armed = False

# F is the very first state, with no transition into it for the
# ZONE_STATES check below to hook -- synced explicitly here instead,
# once, so the camera starts in LINE mode regardless of whatever mode it
# was left in by a previous run or test.
sync_camera_line_mode()

while True:
    hub.display.char(state)

    if DEBUG_MODE:
        is_pressed = Button.RIGHT in hub.buttons.pressed()
        if is_pressed and not was_pressed and not armed:
            armed = True
            print("armed: %s" % state)
        was_pressed = is_pressed
    else:
        armed = True  # no arm gate outside debug mode -- always run

    if armed:
        next_state = STATE_FUNCTIONS[state]()
        if next_state is not None:
            print("-> %s" % next_state)
            if state in ZONE_STATES and next_state not in ZONE_STATES:
                sync_camera_line_mode()  # crossing back from a zone
                                          # state into a line state
            state = next_state
            armed = False  # debug mode: new state waits for its own
                            # press; normal mode: overridden straight
                            # back to True above on the very next tick
    else:
        robot.stop()  # paused: safe default, same reasoning as state_S()/state_H()

    wait(10)