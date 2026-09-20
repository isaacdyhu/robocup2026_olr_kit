"""
Bench test - hub 1 side of the hub-to-hub link (SPIKE Prime / Pybricks).

Runs on HUB 1, with robocup_olr_hub2.py running on hub 2. Drives hub 2's four
goals from hub 1's own buttons and prints everything hub 2 reports back, so the
BLE link and the claw/lifter mechanism can be exercised together with no course,
no camera and no line following.

    LEFT  click         L  lower the lifter
    LEFT  double click  C  close the claw
    RIGHT click         R  raise the lifter
    RIGHT double click  O  open the claw

This is a TEST program, not part of the run. robocup_olr_hub1.py is untouched.


SINGLE AND DOUBLE CLICKS, WITHOUT BLOCKING
------------------------------------------
A double click can only be recognised by waiting to see whether a second press
arrives, and waiting is exactly what this loop must not do: hub 2 broadcasts
every 10 ms and the whole point is to watch that stream. So the detector is a
state machine polled once per cycle rather than a wait-for-second-press.

A double click fires on the second PRESS - immediately, because nothing more
needs to be known. A single click fires only after the gap expires AND the
button is up, so holding a button does nothing until it is released. That
asymmetry is deliberate: it makes the double click feel instant while keeping
the single click unambiguous.


WHAT THIS SHARES WITH THE REAL PROGRAMS, AND WHAT IT MUST NOT DRIFT FROM
------------------------------------------------------------------------
The channel numbers and the goal letters below are COPIES of the values in
robocup_olr_hub2.py. Nothing enforces the match, and a mismatch fails silently
in the worst way: hub 2 does not recognise the goal, RUNNERS.get() returns None,
and it reports FAULT - which looks exactly like a jammed motor. If hub 2 answers
every command with FAULT, check these letters first.
"""

from pybricks.hubs import PrimeHub
from pybricks.parameters import Button
from pybricks.tools import StopWatch, wait

# ============================================================================
# CONFIGURATION
# ============================================================================

COMMAND_CHANNEL = 1     # hub 1 -> hub 2
STATE_CHANNEL = 2       # hub 2 -> hub 1

LOOP_MS = 10

# How long after a press to keep waiting for a second one. Long enough to be
# comfortable on a stiff hub button, short enough that a single click does not
# feel laggy - every single click costs this much before it fires.
DOUBLE_CLICK_MS = 350

# Say so if hub 2 goes quiet. Many multiples of hub 2's 10 ms cycle, so this
# only fires on a hub that has actually stopped, not on a few lost packets.
SILENCE_MS = 2000

# ---- Copied from robocup_olr_hub2.py - see the module docstring -------------
GOAL_LOWER = "L"
GOAL_CLOSE = "C"
GOAL_RAISE = "R"
GOAL_OPEN = "O"

PHASE_NAMES = {"I": "IDLE", "M": "MOVING", "D": "DONE", "F": "FAULT"}
GRIP_NAMES = {"?": "UNKNOWN", "E": "EMPTY", "H": "HOLDING", "B": "BLOCKED"}

# ============================================================================
# HARDWARE
# ============================================================================

hub = PrimeHub(broadcast_channel=COMMAND_CHANNEL,
               observe_channels=[STATE_CHANNEL])

clock = StopWatch()


# ============================================================================
# CLICK DETECTION
# ============================================================================

class ClickCounter:
    """Turns presses of one button into single and double clicks.

    Polled, never blocking: update() is called once per loop cycle with the
    current set of pressed buttons and returns 0, 1 or 2.
    """

    def __init__(self, button):
        self.button = button
        self.was_down = False
        self.presses = 0
        self.deadline = 0

    def update(self, pressed_now, now):
        """0 nothing yet, 1 single click, 2 double click."""
        down = self.button in pressed_now
        # Edge, not level. At LOOP_MS = 10 a held button is "pressed" on a
        # hundred consecutive cycles and would otherwise count as a hundred
        # presses.
        new_press = down and not self.was_down
        self.was_down = down

        if new_press:
            self.presses += 1
            self.deadline = now + DOUBLE_CLICK_MS
            if self.presses >= 2:
                self.presses = 0
                return 2
            return 0

        # A single click needs the gap to expire AND the button to be up, so a
        # long hold does nothing until released rather than firing mid-hold.
        if self.presses == 1 and not down and now >= self.deadline:
            self.presses = 0
            return 1

        return 0


# ============================================================================
# REPORTING
# ============================================================================

def describe(state):
    """Hub 2's (seq_echo, phase, grip, left_raw, right_raw), spelled out."""
    if not isinstance(state, tuple) or len(state) != 5:
        return "unexpected payload: %s" % str(state)

    seq, phase, grip, left, right = state
    return ("seq=%s %s grip=%s outer L=%s R=%s"
            % (seq,
               PHASE_NAMES.get(phase, "?" + str(phase)),
               GRIP_NAMES.get(grip, "?" + str(grip)),
               left, right))


# ============================================================================
# MAIN LOOP
# ============================================================================

def main():
    left_clicks = ClickCounter(Button.LEFT)
    right_clicks = ClickCounter(Button.RIGHT)

    seq = 0
    sent = None
    last_report = None
    last_heard = None
    warned = False

    print("Hub 1 BLE bench test.")
    print("  LEFT  click / double  ->  %s lower / %s close"
          % (GOAL_LOWER, GOAL_CLOSE))
    print("  RIGHT click / double  ->  %s raise / %s open"
          % (GOAL_RAISE, GOAL_OPEN))
    print("Waiting for hub 2 ...")

    while True:
        now = clock.time()

        # ---- 1. buttons ---------------------------------------------------
        pressed = hub.buttons.pressed()
        left = left_clicks.update(pressed, now)
        right = right_clicks.update(pressed, now)

        goal = None
        if left == 1:
            goal = GOAL_LOWER
        elif left == 2:
            goal = GOAL_CLOSE
        elif right == 1:
            goal = GOAL_RAISE
        elif right == 2:
            goal = GOAL_OPEN

        # ---- 2. send ------------------------------------------------------
        # A new seq every time, including for the same goal twice running.
        # `goal` is a level, so without it a repeat would be invisible to hub 2
        # and the second press would look like a dead button.
        if goal is not None:
            seq += 1
            sent = goal
            hub.ble.broadcast((seq, goal))
            hub.display.char(goal)
            print("%d  -> seq=%d goal=%s" % (now, seq, goal))

        # ---- 3. receive ---------------------------------------------------
        # observe() returns the last thing heard, or None if nothing recently.
        state = hub.ble.observe(STATE_CHANNEL)

        if state is None:
            if last_heard is not None and now - last_heard > SILENCE_MS:
                if not warned:
                    print("%d  hub 2 has gone quiet" % now)
                    warned = True
        else:
            last_heard = now
            if warned:
                print("%d  hub 2 is back" % now)
                warned = False

            # On CHANGE, not per packet. Hub 2 broadcasts every 10 ms, so a
            # line each would bury the transitions that matter. The sensor
            # readings are left out of the comparison because they jitter
            # constantly and would make every packet look like a change.
            key = state[:3] if len(state) >= 3 else state
            if key != last_report:
                last_report = key
                print("%d  <- %s" % (now, describe(state)))

                # The echo is what closes the loop: hub 1 knows a command
                # landed only when hub 2 repeats its seq back.
                if len(state) >= 2 and state[0] == seq and state[1] == "D":
                    print("%d     %s complete" % (now, sent))

        wait(LOOP_MS)


if __name__ == "__main__":
    main()