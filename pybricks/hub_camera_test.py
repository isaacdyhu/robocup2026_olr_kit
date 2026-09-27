"""
Bench test - hub 1 side of the camera link (SPIKE Prime / Pybricks).

Runs on HUB 1, with robocup_olr_cam.py running on the OpenMV camera. Puts each
of the four questions to the camera from hub 1's own buttons and prints every
answer, so the PUPRemote link, the question protocol and the camera's detectors
can be exercised with no course, no hub 2 and no line following.

    LEFT  click         J  is the black line CONTINUING ahead?
    RIGHT click         M  seam check - line ahead, and how much WHITE
    LEFT  double click  A  is there a black line ANYWHERE ahead?  (raises aim)
    RIGHT double click  Z  nearest evacuation-zone target          (raises aim)

Between presses it keeps polling with whatever question is current and prints
the answer whenever it changes, so the camera's output is always on screen.

*** SET STANDALONE = False in robocup_olr_cam.py *** - with it True the camera
holds its own question and ignores the hub, and every query here will time out.

This is a TEST program, not part of the run. robocup_olr_hub1.py is untouched.


WHY THE POLLING LOOKS LIKE THIS
-------------------------------
The link is not request-response. PUPRemote is a shared variable: the camera
computes an answer from the frame it has already taken and the hub reads
whatever was last put on the wire. So the first read after asking a NEW question
is always stale - it answers the PREVIOUS question.

Every request therefore carries a sequence byte the camera echoes, and the hub
polls until the echo matches. This script prints the polls it discards as well
as the one it accepts, because how many were discarded is the interesting number
when something is wrong.

Questions that move the camera ('A' and 'Z' tilt it up) need a far longer budget
than the others: the camera deliberately WITHHOLDS the echo until the servo has
settled, so until then every poll looks stale. A budget shorter than the
camera's settle time turns a perfectly healthy camera into "unreachable".
"""

from pybricks.hubs import PrimeHub
from pybricks.parameters import Button, Port
from pybricks.tools import StopWatch, wait

from pupremote_hub import PUPRemoteHub

# ============================================================================
# CONFIGURATION
# ============================================================================

CAMERA_PORT = Port.D

LOOP_MS = 10
DOUBLE_CLICK_MS = 350

# ---- Copied from robocup_olr_hub1.py / robocup_olr_cam.py -------------------
# Nothing enforces any of this. A mismatch does not raise - it silently produces
# a plausible wrong answer, or a timeout that looks like dead hardware.
QUESTION_JUNCTION = ord('J')
QUESTION_SEAM = ord('M')
QUESTION_AHEAD = ord('A')
QUESTION_ZONE = ord('Z')

KIND_NAMES = {0: "nothing", 1: "DARK sphere", 2: "LIGHT sphere",
              3: "GREEN point", 4: "RED point"}

CAMERA_CONNECT_TRIES = 10
CAMERA_WAIT_MS = 6              # PUPRemote call timeout

CAMERA_POLL_TRIES = 10
CAMERA_POLL_GAP_MS = 30

# MUST EXCEED robocup_olr_cam.py's TILT_MOVE_MS + TILT_SETTLE_MS, or an aim
# change is indistinguishable from an absent camera. At the time of writing
# those are 500 + 200, and this is 1000.
CAMERA_AIM_SETTLE_MS = 1000
CAMERA_AIM_POLL_TRIES = CAMERA_AIM_SETTLE_MS // CAMERA_POLL_GAP_MS + 10

# How often to say something while waiting out a long settle, in polls. Without
# it an 'A' or 'Z' looks like a three-second hang.
PROGRESS_EVERY = 10

# ============================================================================
# HARDWARE
# ============================================================================

hub = PrimeHub()
clock = StopWatch()


def connect_camera():
    """Connect and register 'look'. Retried, so start-up order does not matter.

    Until robocup_olr_cam.py is running and advertising 'look', the port
    presents its default modes and add_command fails.
    """
    for attempt in range(CAMERA_CONNECT_TRIES):
        try:
            cam = PUPRemoteHub(CAMERA_PORT)
            # Must match the camera side exactly. The 8-byte reply is a power of
            # two deliberately - a non-power-of-two payload makes an invalid
            # LPF2 frame and crashes the sensor.
            cam.add_command('look', to_hub_fmt="hhhh", from_hub_fmt="4s")
            print("Camera connected on port %s." % CAMERA_PORT)
            return cam
        except Exception as error:
            print("Waiting for the OpenMV 'look' client (%d): %s"
                  % (attempt + 1, error))
            wait(500)

    print("No camera found. Is robocup_olr_cam.py running?")
    return None


# ============================================================================
# CLICK DETECTION
# ============================================================================

class ClickCounter:
    """Turns presses of one button into single and double clicks.

    Polled, never blocking. A double click fires on the second PRESS, since
    nothing more needs to be known by then; a single click fires only once the
    gap expires AND the button is up, so a hold does nothing until released.
    """

    def __init__(self, button):
        self.button = button
        self.was_down = False
        self.presses = 0
        self.deadline = 0

    def update(self, pressed_now, now):
        """0 nothing yet, 1 single click, 2 double click."""
        down = self.button in pressed_now
        # Edge, not level: at LOOP_MS = 10 a held button is "pressed" on a
        # hundred consecutive cycles.
        new_press = down and not self.was_down
        self.was_down = down

        if new_press:
            self.presses += 1
            self.deadline = now + DOUBLE_CLICK_MS
            if self.presses >= 2:
                self.presses = 0
                return 2
            return 0

        if self.presses == 1 and not down and now >= self.deadline:
            self.presses = 0
            return 1

        return 0


# ============================================================================
# READING THE REPLY
# ============================================================================
# The four fields mean different things per question, which is the part of this
# protocol most likely to be misread. One place decides.

def describe(question, reply):
    """The camera's (echo, a, b, c) spelled out for the question that asked."""
    _echo, first, second, third = reply

    if question == QUESTION_ZONE:
        return ("kind=%s bearing=%+d deg range=%d mm"
                % (KIND_NAMES.get(first, "?%d" % first), second, third))

    what = "LINE AHEAD" if first == 1 else "no line   "
    unit = "white" if question == QUESTION_SEAM else "black"
    return "%s angle=%+d deg %s=%d%%" % (what, second, unit, third)


# ============================================================================
# ASKING
# ============================================================================

_seq = 0


def ask(camera, question, tries):
    """Put `question` to the camera and wait for an answer computed FOR IT.

    Returns the raw (echo, a, b, c), or None. Prints the discarded polls as
    well as the accepted one - a question that needed 3 polls and one that
    needed 40 are both "working", but only one of them is healthy.
    """
    global _seq

    if camera is None:
        return None

    _seq = (_seq + 1) & 0xFF
    # Byte 2 is the kind filter, used by 'Z' only. 0 means "whatever is
    # nearest"; set it to a KIND_* to hunt one target type.
    request = bytes((question, _seq, 0, 0))

    for attempt in range(tries):
        try:
            reply = camera.call('look', request, wait_ms=CAMERA_WAIT_MS)
        except Exception as error:
            print("  %s query FAILED: %s" % (chr(question), error))
            return None

        if reply[0] == _seq:
            print("  %s answered after %d poll%s: %s"
                  % (chr(question), attempt + 1, "" if attempt == 0 else "s",
                     describe(question, reply)))
            return reply

        if attempt and attempt % PROGRESS_EVERY == 0:
            # Not an error yet. 'A' and 'Z' move the servo, and the camera
            # withholds its echo until it has settled.
            print("  %s still settling (%d polls, echo=%d want %d)"
                  % (chr(question), attempt, reply[0], _seq))

        wait(CAMERA_POLL_GAP_MS)

    print("  %s NEVER ECHOED seq %d after %d polls (%d ms)"
          % (chr(question), _seq, tries, tries * CAMERA_POLL_GAP_MS))
    return None


def watch(camera, question):
    """One poll with the CURRENT question, keeping the link warm.

    Re-sends the question rather than a fixed one, so the camera stays pointed
    where the last button press put it. It does NOT advance the sequence byte,
    so nothing here is treated as a fresh question.

    The link goes stale after about a second without traffic and then costs a
    slow reconnect, so this has to run even when nothing is being asked.
    """
    if camera is None:
        return None
    try:
        return camera.call('look', bytes((question, _seq, 0, 0)),
                           wait_ms=CAMERA_WAIT_MS)
    except Exception:
        return None             # a dropped poll is not worth reporting


# ============================================================================
# MAIN LOOP
# ============================================================================

def main():
    camera = connect_camera()

    left_clicks = ClickCounter(Button.LEFT)
    right_clicks = ClickCounter(Button.RIGHT)

    question = QUESTION_JUNCTION
    last_shown = None
    quiet_since = clock.time()

    print("Hub 1 camera bench test.")
    print("  LEFT  click / double  ->  J junction / A ahead")
    print("  RIGHT click / double  ->  M seam     / Z zone")
    print("  (set STANDALONE = False in robocup_olr_cam.py)")

    while True:
        now = clock.time()

        # ---- 1. buttons ---------------------------------------------------
        pressed = hub.buttons.pressed()
        left = left_clicks.update(pressed, now)
        right = right_clicks.update(pressed, now)

        asked = None
        tries = CAMERA_POLL_TRIES
        if left == 1:
            asked = QUESTION_JUNCTION
        elif left == 2:
            asked, tries = QUESTION_AHEAD, CAMERA_AIM_POLL_TRIES
        elif right == 1:
            asked = QUESTION_SEAM
        elif right == 2:
            asked, tries = QUESTION_ZONE, CAMERA_AIM_POLL_TRIES

        # ---- 2. ask -------------------------------------------------------
        if asked is not None:
            question = asked
            hub.display.char(chr(question))
            print("%d  ask %s" % (now, chr(question)))
            ask(camera, question, tries)
            last_shown = None           # re-print the stream after an answer
            quiet_since = clock.time()

        # ---- 3. keep listening --------------------------------------------
        # On CHANGE, not per poll. The camera recomputes every frame and the
        # answer is usually the same one; a line per poll would bury the
        # transitions that matter.
        reply = watch(camera, question)
        if reply is not None:
            shown = describe(question, reply)
            if shown != last_shown:
                last_shown = shown
                print("%d  %s: %s" % (now, chr(question), shown))
            quiet_since = now
        elif now - quiet_since > 2000:
            print("%d  camera is not answering" % now)
            quiet_since = now

        wait(LOOP_MS)


if __name__ == "__main__":
    main()