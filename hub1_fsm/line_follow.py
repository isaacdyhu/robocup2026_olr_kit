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
from pybricks.tools import wait
from pybricks.iodevices import PUPDevice

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


# to_hub_fmt/from_hub_fmt must match camera.py's add_command("mode", ...)
# exactly, and this project's own hub_camera_test.py already confirmed
# them end-to-end. ZONE (echoed_mode=1) reply fields, per camera.py's own
# comment above its "mode" registration:
#   count, kind, bearing_deg, dist_mm, green_found, red_found
# kind: -1 = no sphere, 0 = dead, 1 = live.
camera = _PUPRemoteHub(Port.D)
camera.add_command("mode", to_hub_fmt="hhhhhhhh", from_hub_fmt="b")


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

# The camera doesn't switch into ZONE mode instantly -- it needs a frame
# or two after the "mode" command lands (pybricks/hub_camera_test.py's
# own "(switching...)" comment describes the same lag) -- so both of the
# retry loops below resend/re-check rather than trusting a single call.
SCAN_MODE_SWITCH_RETRIES = 20   # attempts to confirm ZONE mode before
                                # the scan itself starts
SCAN_MODE_SWITCH_POLL_MS = 50
SCAN_SETTLE_MS = 1500            # pause after each pivot, before querying,
                                # so the camera grabs a frame at the new heading
SCAN_QUERY_RETRIES = 5          # per-angle attempts to get a fresh ZONE reply
SCAN_QUERY_POLL_MS = 50
# All untested placeholders, same as every other numeric constant here.


def query_zone_camera():
    """
    Ask the camera for its current ZONE-mode reading, retrying up to
    SCAN_QUERY_RETRIES times if echoed_mode hasn't caught up to ZONE yet
    (see this section's header comment on why that lag is expected, not
    a bug). Returns (count, kind, bearing_deg, dist_mm, green_found,
    red_found), or None if the camera never confirmed ZONE mode within
    the retry budget.
    """
    for _ in range(SCAN_QUERY_RETRIES):
        echoed_mode, heartbeat, f2, f3, f4, f5, f6, f7 = camera.call("mode", 1, wait_ms=5)
        if echoed_mode == 1:
            return f2, f3, f4, f5, f6, f7
        wait(SCAN_QUERY_POLL_MS)
    return None


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

CENTRE_ROTATE_SPEED_DPS = 10  # deliberately SLOW, and the single most
                              # important constant here. Unlike a
                              # stop-and-measure loop, this one measures
                              # while moving, so every reading is stale by
                              # one camera frame plus one link round-trip
                              # -- call it 100-200ms in ZONE mode, where
                              # the blob work is heavy. The robot keeps
                              # turning through that whole delay, so
                              # overshoot is roughly
                              #     speed x latency
                              # (20 deg/s x 0.15 s = 3 deg, inside the
                              # tolerance band above). IF IT OVERSHOOTS,
                              # LOWER THIS -- do not tighten
                              # CENTRE_TOLERANCE_DEG, which makes it worse
                              # by narrowing the band the robot has to
                              # catch while sailing past at the same rate.
CENTRE_MAX_SWEEP_DEG = 90     # safety cap on total rotation: give up
                              # rather than spin forever if the bearing
                              # never reaches zero (ball rolled out of
                              # frame, bearing sign convention inverted,
                              # camera stuck on a stale frame)


def centre_on_ball():
    """
    Turn slowly toward the ball in one continuous motion, polling the
    camera throughout, and stop once it reports the ball within
    CENTRE_TOLERANCE_DEG of dead ahead.

    Two stop conditions, not one. The obvious one is the bearing landing
    inside the tolerance band. The second is the bearing CHANGING SIGN
    between two polls -- that means the ball crossed dead-ahead somewhere
    in the gap between those readings, and without catching it the robot
    would carry on turning away from a ball it has already passed. With
    stale readings and a finite poll rate, that case is not an edge case;
    it is what happens whenever the band is crossed faster than it is
    sampled.

    Returns True once centred (either way), False if the ball was lost,
    the camera stopped answering, or the sweep cap was hit.
    """
    result = query_zone_camera()
    if result is None:
        print("centre: camera not responding")
        return False

    count, kind, bearing, dist, green_found, red_found = result
    if kind == -1:
        print("centre: no ball to centre on")
        return False

    if abs(bearing) <= CENTRE_TOLERANCE_DEG:
        print("centre: already centred at bearing %d" % bearing)
        return True

    # Positive bearing = ball right of centre -> turn right (clockwise),
    # which is +1 in this file's convention throughout. The wheel speeds
    # below are just rotate_to_heading()'s own in-place-spin case
    # (pivot_offset 0) written out directly: left forward, right back.
    turn_sign = 1 if bearing > 0 else -1
    start_heading = hub.imu.heading()
    print("centre: bearing %d, turning %s" %
          (bearing, "right" if turn_sign > 0 else "left"))

    left_motor.run(turn_sign * CENTRE_ROTATE_SPEED_DPS)
    right_motor.run(-turn_sign * CENTRE_ROTATE_SPEED_DPS)

    while True:
        if abs(hub.imu.heading() - start_heading) >= CENTRE_MAX_SWEEP_DEG:
            left_motor.stop()
            right_motor.stop()
            print("centre: swept %d deg without centring, giving up" %
                  CENTRE_MAX_SWEEP_DEG)
            return False

        # No explicit wait between polls: query_zone_camera() already
        # costs a link round-trip, and every millisecond added here is
        # another millisecond of rotation the reading doesn't know about.
        result = query_zone_camera()
        if result is None:
            left_motor.stop()
            right_motor.stop()
            print("centre: camera stopped responding mid-turn")
            return False

        count, kind, bearing, dist, green_found, red_found = result

        if kind == -1:
            left_motor.stop()
            right_motor.stop()
            print("centre: ball lost mid-turn")
            return False

        if abs(bearing) <= CENTRE_TOLERANCE_DEG:
            left_motor.stop()
            right_motor.stop()
            print("centre: centred at bearing %d" % bearing)
            return True

        if (bearing > 0) != (turn_sign > 0):
            # Sign flipped: the ball crossed dead-ahead between polls.
            # Stop now -- still turning would walk away from it. The
            # printed bearing is how far past centre it got, which is the
            # number to watch if CENTRE_ROTATE_SPEED_DPS needs lowering.
            left_motor.stop()
            right_motor.stop()
            print("centre: passed centre, stopped at bearing %d" % bearing)
            return True


def state_V():
    """SURVEY (design.md Sec7) -- staggered rotation scan for a sphere,
    then centre on its reported bearing before handing off to A."""
    # Cancel state_F()'s continuous drive() before taking direct motor
    # control inside rotate_to_heading() -- same reasoning as every
    # other state that takes over the motors directly.
    robot.stop()

    # Make sure the camera is actually in ZONE mode before scanning --
    # state_B() only flags zone mode locally, it doesn't yet tell the
    # camera itself to switch (out of scope to add there right now), so
    # it's confirmed here instead, once, before the scan begins.
    for _ in range(SCAN_MODE_SWITCH_RETRIES):
        echoed_mode = camera.call("mode", 1, wait_ms=5)[0]
        if echoed_mode == 1:
            wait(SCAN_SETTLE_MS)
            break
        wait(SCAN_MODE_SWITCH_POLL_MS)
    else:
        print("survey: camera never confirmed ZONE mode, scanning anyway")

    heading0 = hub.imu.heading()  # scan angles below are relative to this

    for angle in SCAN_ANGLES_DEG:
        rotate_to_heading(0, heading0 + angle, SCAN_ROTATE_SPEED_DPS)

        # Diagnostic: requested vs actually-reached heading (relative to
        # heading0), so a mismatch between the two is visible directly
        # rather than inferred from robot behaviour alone. actual should
        # match angle closely (within ROTATE_SLOWDOWN_MARGIN_DEG-ish); a
        # large or systematic gap here points at rotate_to_heading()
        # itself (or the gyro/motors), not at this loop or SCAN_ANGLES_DEG.
        actual = hub.imu.heading() - heading0
        print("survey: requested %d deg, actual %d deg" % (angle, actual))

        wait(SCAN_SETTLE_MS)  # let the camera grab a fresh frame at this heading

        result = query_zone_camera()
        if result is None:
            print("survey: camera not responding at %d deg" % angle)
            continue

        count, kind, bearing, dist, green_found, red_found = result
        print("survey: camera says count=%d kind=%d bearing=%d dist=%d green=%s red=%s" %
              (count, kind, bearing, dist,
               "y" if green_found else "n", "y" if red_found else "n"))

        if kind != -1:  # a sphere was found -- dead or alive both count
            print("survey: sphere found at scan angle %d (kind=%d bearing=%d dist=%d)" %
                  (angle, kind, bearing, dist))

            if centre_on_ball():
                print("survey: centred, heading %d deg relative to entry" %
                      (hub.imu.heading() - heading0))
                return "A"

            # Couldn't lock on -- ball lost mid-correction, or the bearing
            # never converged. Carry on scanning from the next angle
            # rather than handing A a target that isn't there; the scan's
            # own targets are absolute (heading0 + angle), so whatever
            # rotation centring already did doesn't throw the rest off.
            print("survey: centring failed, resuming scan")

    print("survey: no sphere found after full scan")
    # TODO -> K (none left, or time short) / Q (scan failed) per
    # design.md Sec7 -- neither implemented yet, so this just stays in V.
    return None


def state_A():
    """APPROACH (design.md Sec7)."""
    # TODO -> C (within capture range) / V (target lost) / Q (blocked)
    return None


def state_C():
    """CAPTURE (design.md Sec7)."""
    # TODO -> D (grip confirmed) / A (grip failed, ball visible) /
    #         V (grip failed, ball gone) / Q (hub 2 FAULT or timeout)
    return None


def state_D():
    """DELIVER (design.md Sec7)."""
    # TODO -> T (arrived at the correct triangle) / A (dropped, still
    #         visible) / V (dropped and gone, or triangle missing) /
    #         Q (blocked, or hub 2 FAULT)
    return None


def state_T():
    """DEPOSIT (design.md Sec7) -- NOT "green turn"; that's L/R above."""
    # TODO -> V (released -- go find the next) / D (release failed,
    #         still holding) / Q (hub 2 FAULT or timeout)
    return None


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
    Tell the camera to switch to LINE mode (tilts it back down). Called
    once at the moment the FSM crosses from a zone state back into a
    line state, not every tick -- state_F()'s own loop runs far too
    often to afford a blocking camera.call() on every 10ms tick, unlike
    state_V()'s own zone-mode entry, which can afford to retry-until-
    confirmed because it only happens once per zone attempt. This is
    just restoring the default/safe camera orientation for line-
    following, so a single best-effort send is accepted here instead.
    """
    camera.call("mode", 0, wait_ms=5)

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