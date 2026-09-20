"""
RoboCup Open Line Rescue - Hub 2 (SPIKE Prime / Pybricks)

Hub 2 is a peripheral. It never decides anything and it never waits for hub 1:
it runs one loop that does two independent jobs, and hub 1 reads the results
whenever it likes.

  1. publish the two OUTER colour sensor readings, every cycle, unconditionally
  2. advance whatever claw/lifter goal hub 1 last asked for, one step per cycle

EVERY GOAL IS ONE MOTION, and hub 1 owns the sequencing.

  L  lower    lifter down to grab height
  C  close    claw closed until it stalls, plus the grip verdict
  R  raise    lifter up to carry height
  O  open     claw opened

A capture is L, C, R issued in turn; a deposit is O, and confirming the deposit
is C again - open jaws prove nothing about what was in them, only a re-close
does. Hub 2 previously owned those sequences as compound STOW, CAPTURE and
RELEASE goals. Sequencing here meant hub 2 BRANCHING ON ITS OWN GRIP VERDICT,
which put half the recovery policy on the peripheral and left hub 1 unable to
change its mind partway or to exercise one joint at a time.

What could NOT move is the grip verdict itself. The claw's stall angle is a
tight loop on a motor hub 1 cannot see, so `close` still measures it and
publishes it; what hub 1 does about an EMPTY or BLOCKED result is hub 1's
decision alone.

WHY IT LOOKS LIKE THIS
----------------------
Pybricks hub-to-hub messaging is connectionless advertising: broadcast() sets
what this hub advertises, observe() returns the last thing heard, and there is
no addressing and no acknowledgement. There is no way to "reply to a request".

So both directions carry continuously-repeated STATE rather than commands, and a
dropped packet costs one cycle instead of desynchronising the two hubs.

PROTOCOL
--------
  channel 1   hub 1 -> hub 2   (seq, goal)
  channel 2   hub 2 -> hub 1   (seq_echo, phase, grip, left_raw, right_raw)

`goal` is a LEVEL, not a verb - re-reading the same value does nothing. `seq` is
what makes retries expressible: without it, asking for CLOSE again after a
failed grab would be indistinguishable from the command already being obeyed.

Hub 1's "wait for completion" is `seq_echo == seq and phase == DONE`.

FOUR RULES THE LOOP MUST OBEY
-----------------------------
1. Sensors and broadcast run unconditionally, outside the executor. Whatever the
   claw is doing, hub 2 reads both sensors and publishes. That is what lets
   hub 1 tell "busy" from "crashed" - a hub 2 mid-motion keeps saying MOVING.
2. Every step is non-blocking. One step advances per cycle. A blocking call
   silences the broadcast for its duration, and silence is indistinguishable
   from a dead hub.
3. Every step has its own deadline. Exceeding it means FAULT, so a claw that
   jams on closing reports in a second rather than after the whole sequence.
4. observe() returning None means HOLD. Never open, never move. Dropping a
   victim mid-transit costs the full 40 points; holding one a few seconds too
   long costs nothing.

   Rule 4 is now the ONLY protection left here. The compound goals used to
   refuse to do damage on their own - CAPTURE stopped rather than lift an empty
   claw, RELEASE re-closed to confirm the victim had left. Neither check
   survives: OPEN at carry height drops whatever is in the jaws, and hub 2 will
   not second-guess it. Rule 4 still holds because it is about a LOST LINK, not
   about an instruction that arrived.

DONE and FAULT LATCH until the next seq arrives. That is what makes the protocol
survive a lossy link: the result stays readable indefinitely, so hub 1 can miss
any number of broadcasts and still learn the outcome.

RAW READINGS, NOT CLASSIFIED
----------------------------
The outer sensors are published as raw hsv() VALUE readings. Hub 1 owns the
polarity and will own the calibration, so it owns the thresholds too - if hub 2
classified with its own constants the two hubs could disagree about what "dark"
means.

The value channel specifically, because that is the scale hub 1 measures all of
its own thresholds on. Sending reflection() instead would be numerically
different for the same surface, and hub 1 would compare it against constants
tuned for something else.

STARTUP
-------
The claw and lifter are placed at their home positions BY HAND before starting,
and reset_angle(0) records that as the reference. Every angle below is relative
to it, including the grip verdict - so a run started with the claw somewhere
else will misreport HOLDING and EMPTY.
"""

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import ColorSensor, Motor
from pybricks.parameters import Port
from pybricks.tools import wait, StopWatch

# ============================================================================
# CONFIGURATION
# ============================================================================

COMMAND_CHANNEL = 1     # hub 1 -> hub 2
STATE_CHANNEL = 2       # hub 2 -> hub 1

LEFT_OUTER_PORT = Port.B
CLAW_PORT = Port.C
LIFTER_PORT = Port.D
RIGHT_OUTER_PORT = Port.F

LOOP_MS = 10

# Per-STEP deadline, not per goal. Long enough for the slowest single motion,
# short enough that a jam is reported while hub 1 can still do something.
STEP_TIMEOUT_MS = 3000

# ---- Claw -------------------------------------------------------------------
# Angles are relative to the hand-placed home, which is the CLOSED position, so
# opening is positive. If the claw opens on a negative angle instead, flip the
# sign of CLAW_OPEN and of the run() speed in the closing steps.
CLAW_SPEED = 300        # deg/s
CLAW_OPEN = 100          # deg from closed

# Closing stalls somewhere; where it stalls is the whole grip verdict.
#   at or below EMPTY_MAX  -> the jaws met, nothing in them
#   inside the ball window -> a 40-50 mm sphere
#   above BALL_MAX         -> something larger, or a wall
# MEASURE THESE with a real victim in the claw before trusting a run.
CLAW_EMPTY_MAX = 10     # deg
CLAW_BALL_MIN = 15
CLAW_BALL_MAX = 55

# Torque ceiling while gripping. A victim is an 80 g plastic sphere; a claw
# closing at full torque can crush it or squirt it out sideways.
CLAW_GRIP_TORQUE = 200  # mNm - MEASURE, this is a starting guess

# ---- Lifter -----------------------------------------------------------------
# Also relative to the hand-placed home.
LIFTER_SPEED = 700      # deg/s
LIFTER_GRAB = 1400      # down at floor level to take a victim
LIFTER_CARRY = -1400    # clear of the floor and of the camera's view

# TWO POSITIONS ARE GONE AND NOTHING REPLACES THEM YET.
#
#   LIFTER_PARK = 0     stowed for line following, reached only by STOW
#   LIFTER_RELEASE = 90 above the 60 mm evacuation-point wall, only by RELEASE
#
# So nothing parks the mechanism before line following, and nothing lifts high
# enough to deposit a victim. Both need a goal of their own, or CARRY has to
# serve for one of them. See DESIGN.md open question 19.

# ============================================================================
# PROTOCOL VOCABULARY
# ============================================================================
# Single characters: compact on the wire, and readable on the hub display.

GOAL_LOWER = "L"        # lifter down to grab height
GOAL_CLOSE = "C"        # claw closed until it stalls, plus the grip verdict
GOAL_RAISE = "R"        # lifter up to carry height
GOAL_OPEN = "O"         # claw opened

PHASE_IDLE = "I"
PHASE_MOVING = "M"
PHASE_DONE = "D"
PHASE_FAULT = "F"

GRIP_UNKNOWN = "?"
GRIP_EMPTY = "E"
GRIP_HOLDING = "H"
GRIP_BLOCKED = "B"

# ============================================================================
# HARDWARE
# ============================================================================

hub = PrimeHub(broadcast_channel=STATE_CHANNEL,
               observe_channels=[COMMAND_CHANNEL])

left_outer = ColorSensor(LEFT_OUTER_PORT)
right_outer = ColorSensor(RIGHT_OUTER_PORT)

claw = Motor(CLAW_PORT)
lifter = Motor(LIFTER_PORT)

# Record the hand-placed home. Without this every angle below is measured from
# wherever the encoder happened to be, and the grip verdict is meaningless.
#claw.reset_angle(0)
#.reset_angle(0)

# Keep the grip gentle enough not to crush or eject a victim.
claw.control.limits(torque=CLAW_GRIP_TORQUE)

# Latest grip verdict. Set when CLOSE finds its stall, cleared when a new goal
# starts, and published every cycle in between.
grip = GRIP_UNKNOWN


# ============================================================================
# HELPERS
# ============================================================================

def classify_grip(angle):
    """Turn the angle the claw stalled at into a verdict.

    This is the reason hub 2 owns the mechanics rather than hub 1: the stall is
    a tight loop on a motor hub 1 cannot see, and streaming raw angles over BLE
    fast enough to close that loop remotely is not realistic.
    """
    if angle <= CLAW_EMPTY_MAX:
        return GRIP_EMPTY
    if CLAW_BALL_MIN <= angle <= CLAW_BALL_MAX:
        return GRIP_HOLDING
    return GRIP_BLOCKED


def closing_stalled():
    """Advance a close-until-stall step. True once the jaws have stopped.

    run_until_stalled() would be the obvious call but it BLOCKS, which breaks
    rule 2 - so the close is issued as a plain run() and stall is polled instead.
    """
    if claw.stalled():
        claw.hold()
        return True
    return False


# ============================================================================
# GOALS
# ============================================================================
# Each runner is called once per cycle with the current step, and `entering`
# True on the first cycle of that step - which is when the motor command is
# issued. It returns (next_step, phase):
#
#   (step,     PHASE_MOVING)   still working on this step
#   (step + 1, PHASE_MOVING)   this step finished, move on
#   (step,     PHASE_DONE)     the whole goal is finished
#
# EVERY GOAL IS SINGLE-STEP NOW, so `step` is carried straight through and
# STEP_TIMEOUT_MS bounds a whole goal rather than a fragment of one. The step
# argument stays in the signature regardless: it is what any future multi-step
# goal needs, and taking it out would only have to be undone.

def run_lower(step, entering):
    """Lifter down to grab height."""
    if entering:
        lifter.run_angle(LIFTER_SPEED, LIFTER_GRAB, wait=False)
    return (step, PHASE_DONE) if lifter.done() else (step, PHASE_MOVING)


def run_close(step, entering):
    """Claw closed until it stalls, and the grip verdict that falls out of it.

    The verdict is free - the stall angle is already known - and it is the only
    way hub 1 can learn whether anything was caught, so this goal measures it
    and publishes it. What it does NOT do is act on it: the motion asked for was
    "close", and deciding what an EMPTY or BLOCKED result means belongs to
    hub 1.

    It is also how a deposit is confirmed. Opening the jaws proves nothing about
    what was in them; re-closing does. EMPTY after an OPEN means the victim
    left, HOLDING means it did not and is now re-gripped, ready to retry without
    a fresh capture.
    """
    global grip

    if entering:
        claw.run(-CLAW_SPEED)
    if not closing_stalled():
        return (step, PHASE_MOVING)

    grip = classify_grip(claw.angle())
    return (step, PHASE_DONE)


def run_raise(step, entering):
    """Lifter up to carry height - clear of the floor and of the camera's view.

    CARRY is the only height this reaches, and it is the transport height.
    Clearing the 60 mm evacuation-point wall needs more, and nothing provides it
    - see the note beside LIFTER_CARRY above.
    """
    if entering:
        lifter.run_angle(LIFTER_SPEED, LIFTER_CARRY, wait=False)
    return (step, PHASE_DONE) if lifter.done() else (step, PHASE_MOVING)


def run_open(step, entering):
    """Claw opened.

    grip is left UNKNOWN, which is the truth: open jaws say nothing about what
    was in them a moment ago. Hub 1 settles that by issuing CLOSE afterwards.
    """
    if entering:
        claw.run_angle(CLAW_SPEED, CLAW_OPEN, wait=False)
    return (step, PHASE_DONE) if claw.done() else (step, PHASE_MOVING)


RUNNERS = {
    GOAL_LOWER: run_lower,
    GOAL_CLOSE: run_close,
    GOAL_RAISE: run_raise,
    GOAL_OPEN: run_open,
}


# ============================================================================
# MAIN LOOP
# ============================================================================

def main():
    global grip

    acted_seq = -1
    goal_now = None
    step = 0
    entering = False
    phase = PHASE_IDLE
    shown = None
    step_watch = StopWatch()

    print("Hub 2 ready. Claw and lifter homed at their current positions.")

    while True:
        # ---- 1. sensors, unconditionally --------------------------------
        # hsv().v, NOT reflection(): hub 1 measures every threshold on the hsv
        # value scale, so sending reflection here would hand it numbers from a
        # different scale to compare against LINE_DARK_LEVEL and friends.
        left = left_outer.hsv().v
        right = right_outer.hsv().v

        # ---- 2. command, non-blocking ------------------------------------
        # observe() reads a buffer the radio fills in the background; it never
        # waits for an advertisement. None means nothing heard recently, which
        # under rule 4 means hold whatever we are doing.
        try:
            seq, goal = hub.ble.observe(COMMAND_CHANNEL)
        except (TypeError, ValueError):
            seq, goal = None, None

        if seq is not None and seq != acted_seq:
            acted_seq = seq
            goal_now = goal
            step = 0
            entering = True
            phase = PHASE_MOVING
            grip = GRIP_UNKNOWN
            step_watch.reset()

        # ---- 3. advance one step, never blocking -------------------------
        if phase == PHASE_MOVING:
            runner = RUNNERS.get(goal_now)
            if runner is None:
                # An unrecognised goal is a protocol error, not something to
                # guess at with motors attached.
                phase = PHASE_FAULT
            elif step_watch.time() > STEP_TIMEOUT_MS:
                phase = PHASE_FAULT
            else:
                next_step, phase = runner(step, entering)
                entering = next_step != step
                if entering:
                    step_watch.reset()
                step = next_step

            if phase == PHASE_FAULT:
                # Hold rather than release: a jam is bad, dropping a victim
                # that is already under control is worse.
                claw.hold()
                lifter.hold()

        # ---- 4. publish, unconditionally ---------------------------------
        hub.ble.broadcast((acted_seq, phase, grip, left, right))

        if phase != shown:
            hub.display.char(phase)
            shown = phase

        wait(LOOP_MS)


if __name__ == "__main__":
    main()