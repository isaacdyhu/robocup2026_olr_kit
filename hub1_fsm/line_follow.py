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
    """
    half_track = AXLE_TRACK_MM / 2
    r_left = pivot_offset_mm + half_track
    r_right = pivot_offset_mm - half_track
    max_abs_r = max(abs(r_left), abs(r_right))
    if max_abs_r == 0:
        return  # degenerate pivot -- nothing to do

    turn_sign = 1 if target_heading_deg > hub.imu.heading() else -1
    left_speed = turn_sign * speed_dps * r_left / max_abs_r
    right_speed = turn_sign * speed_dps * r_right / max_abs_r

    left_motor.run(left_speed)
    right_motor.run(right_speed)

    while True:
        current = hub.imu.heading()
        if (turn_sign > 0 and current >= target_heading_deg) or \
           (turn_sign < 0 and current <= target_heading_deg):
            break
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


ZONE_BACKUP_MM = 300           # move backward this far once B decides
                               # the line is genuinely gone, before
                               # flagging zone mode active. Untested
                               # placeholder.
ZONE_BACKUP_SPEED_MM_S = 60    # mm/s for the backup move -- applied via
                               # robot.settings() before straight(), same
                               # reasoning as INTERSECTION_CROSS_SPEED_MM_S


def state_B():
    """Zone check (design.md Sec6) -- simplified: just recognise that the
    line is genuinely gone (guard 4 already confirmed both-white for
    LINE_LOST_MM) and flag zone mode active, without doing anything with
    that yet. Design.md's fuller spec -- raise the camera, spin 360
    scanning for zone targets, dispatch to F/A/V/H -- is not implemented;
    there's nowhere further to go yet, so this only ever stays in B."""
    global _zone_mode_active

    if _zone_mode_active:
        robot.stop()  # already activated -- idle, nothing further
                       # implemented yet (see docstring)
        return None

    # Cancel state_F()'s continuous drive() before the blocking
    # straight() move below -- same reasoning as every other state that
    # takes direct/blocking control.
    robot.stop()
    robot.settings(straight_speed=ZONE_BACKUP_SPEED_MM_S)
    robot.straight(-ZONE_BACKUP_MM)  # negative = backward

    _zone_mode_active = True
    print("zone: mode activated")
    return None


def state_H():
    """Give up -- full reset, wait for the button (design.md Sec4)."""
    robot.stop()
    # TODO: reset polarity/camera aim/hub-2 goal/lockouts/counters, then
    # -> F only, after a button press -- never to W.
    return None


def state_V():
    """SURVEY (design.md Sec7)."""
    # TODO -> A (sphere known) / K (none left, or time short) / Q (scan failed)
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

# --- debug-stepped main loop -------------------------------------------------

# Master switch for the whole arm-gate mechanism this file's header
# describes. True (bench/testing): every freshly-entered state starts
# unarmed and needs one right-button press before its function runs, as
# described below. False (normal/competition running): that gate is
# skipped entirely -- every state runs immediately and unattended the
# instant it's entered, and transitions chain straight into each other
# with no button presses at all. Flip this and reflash; it isn't wired
# to a live button, so it can't be changed without stopping the program.
DEBUG_MODE = False

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
            state = next_state
            armed = False  # debug mode: new state waits for its own
                            # press; normal mode: overridden straight
                            # back to True above on the very next tick
    else:
        robot.stop()  # paused: safe default, same reasoning as state_S()/state_H()

    wait(10)