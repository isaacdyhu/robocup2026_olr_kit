"""
State X -- junction classifier (design.md Sec5, "Classifier states (X and M)").

X runs while following a black line (state F) and decides what a band of
black under the inner sensors actually means: a real crossing, a polarity
seam, a corner with nothing ahead, or a dead link. It is a *classifier*,
not a traverse -- it reads sensors, asks the camera if needed, stops, and
only then commits to one move. Nothing here drives on entry, which is
exactly what lets a corner resolve inside X for free (design.md:746-750).

Two places where the design doc disagrees with itself -- resolved here,
flagged, not silently picked:

  * Whether case (a) performs a recovery move before returning to F.
    "The recovery move" section (design.md:807-846) and the exit-case
    table both give case (a) a move, on the same reasoning as (e): both
    inner sensors are still black, so F would see error~=0 and drive
    straight into the band. The later "Decisions recorded" section
    (design.md:898-904) says case (a) does *not* move and accepts the
    resulting stutter instead. Implemented here: case (a) moves, same as
    (e). Flagged at the case (a) dispatch below -- confirm which is right.

  * When robot.stop() actually happens relative to the camera query.
    "Stop before believing the camera" (design.md:771-773) reads as
    stop-then-query; the numbered action list (design.md:762-769) puts
    "stop" as step 3, *after* the camera query. Implemented here: stop
    immediately on entry, then read sensors / query the camera. Safer,
    and consistent with "stop before believing".

Interfaces this module expects (none of them are defined yet elsewhere --
see the stubs in the self-test at the bottom for the exact shape):

  robot    .stop()                    non-blocking-safe, halts the base
           .straight(mm)               signed, blocking
           .turn(deg)                  signed, blocking, +deg = clockwise
                                        (matches the follower's turn-rate
                                        sign convention)

  hub2     .get_outer_state() -> snapshot with .outer_L, .outer_R (0-100,
           raw reflection) or None if hub 1's own staleness threshold has
           tripped. Where that threshold lives is a hub-2-link concern,
           not this classifier's (design.md:888-894) -- by the time this
           function sees None, the "no fresh data for > N ms" judgement
           has already been made upstream.

  camera   .ask_junction() -> (line_ahead: bool, black_bg: bool) or None
           if unreachable. NOTE: the actual PUPRemote request byte layout
           for this question is still open (design.md Sec9, open
           question 11 -- the request has to carry "the question", not
           just BLK?/WHT? polarity). This call is a placeholder for
           whatever that resolves to.
"""

try:
    from micropython import const
except ImportError:  # running off the hub (e.g. the self-test below)
    def const(x):
        return x

try:
    from time import sleep_ms
except ImportError:  # desktop Python has no sleep_ms
    import time as _time

    def sleep_ms(ms):
        _time.sleep(ms / 1000)


# --- tunables ---------------------------------------------------------------

# Stricter than the follower's "on the line" threshold: with the inner
# sensors at +-16.5 mm and the line at +-7.9 mm, neither should be over the
# line in normal following, but a tight curve can ride one onto the edge.
# A single shared threshold would eventually fire mid-line (design.md:752-757).
INNER_X_BLACK_LEVEL = const(20)

OUTER_BLACK_LEVEL = const(35)  # hub 2's outer pair, same 0-100 scale

FORWARD_CROSS_MM = 18   # case (b): clear a real crossing, steering off
                         # -- watch the 18.2 mm yawed-crossing margin
                         # (design.md:802-805) if a crossing needs two passes
FORWARD_FLIP_MM = 10    # case (c): commit to the seam before flipping polarity
RECOVERY_BACK_MM = 8    # cases (a)/(e), last turn rate was ~0
RECOVERY_SPIN_DEG = 45  # cases (a)/(e), directed by last turn rate
CASE_D_SPIN_DEG = 45    # case (d), directed by the dark outer sensor

CAMERA_FRAME_MS = 40  # placeholder -- tune to the camera's actual frame time

# Not yet adopted upstream (design.md:855-859): without this, case (e) can
# cycle indefinitely if the recovery move never clears the band. Mirrors
# M's case-(b) three-strike counter, which solves the same shape of problem.
CASE_E_MAX_STRIKES = const(3)


class Dispatch:
    """Where X hands off to. GIVE_UP means: reset and wait for the button."""
    FOLLOW_BLACK = "F"
    FOLLOW_WHITE = "W"
    GIVE_UP = "H"


# --- last commanded turn rate ------------------------------------------------
#
# Cases (a) and (e) have no directional evidence from the outer pair, so they
# fall back on what the follower was doing when it hit the band (design.md:
# 832-846). This is "new state that does not exist yet" per design.md:861-862
# -- F's control loop needs to call record_turn_rate() every tick:
#
#     turn_rate = clamp(K * error * speed, +-TURN_MAX)
#     record_turn_rate(turn_rate)
#     robot.drive(speed, turn_rate)

_last_turn_rate = 0


def record_turn_rate(rate):
    """F calls this every control-loop tick so X has something to read."""
    global _last_turn_rate
    _last_turn_rate = rate


def get_last_turn_rate():
    return _last_turn_rate


# --- state X keeps between visits (cleared by H's reset, design.md Sec4) ----

_case_e_streak = 0


def reset_junction_state():
    """Call this from H's reset. Nothing else in X carries state across runs."""
    global _case_e_streak
    _case_e_streak = 0


def should_enter_junction(left_inner, right_inner):
    """Guard: F -> X. Both inner sensors read black. Nothing else."""
    return (
        left_inner <= INNER_X_BLACK_LEVEL
        and right_inner <= INNER_X_BLACK_LEVEL
    )


def classify_junction(robot, hub2, camera, last_turn_rate=None):
    """
    Run state X to completion and return the next state (Dispatch.*).

    `last_turn_rate` defaults to whatever record_turn_rate() last recorded;
    pass it explicitly in tests so the case-(a)/(e) fallback is deterministic.
    """
    global _case_e_streak

    if last_turn_rate is None:
        last_turn_rate = get_last_turn_rate()

    # Stop immediately -- see the file-header note on stop-vs-query ordering.
    robot.stop()

    outer = hub2.get_outer_state()
    if outer is None:
        return Dispatch.GIVE_UP  # case (g): outer-sensor link is stale

    outer_l_dark = outer.outer_L <= OUTER_BLACK_LEVEL
    outer_r_dark = outer.outer_R <= OUTER_BLACK_LEVEL
    any_dark = outer_l_dark or outer_r_dark

    line_ahead = False
    black_bg = False

    if any_dark:
        # Discard one frame: the answer can be up to a frame old, computed
        # while the robot was still moving into the junction (design.md:
        # 771-773).
        camera.ask_junction()  # discard
        sleep_ms(CAMERA_FRAME_MS)
        reply = camera.ask_junction()  # believe this one

        if reply is None:
            return Dispatch.GIVE_UP  # case (f): camera unreachable

        line_ahead, black_bg = reply

    # --- dispatch -------------------------------------------------------

    if not any_dark:
        # case (a): both outer white, camera not asked.
        #
        # FLAGGED: design.md's exit-case table and "the recovery move"
        # section give this case a move, on the same reasoning as (e) --
        # both inner sensors are still black, so F would see error~=0 and
        # drive straight into the band. A later section ("Decisions
        # recorded") says case (a) does *not* move and accepts the
        # resulting stop-start stutter instead. Implemented here: (a)
        # moves, same as (e) -- confirm which reading the team wants.
        _recover(robot, last_turn_rate)
        _case_e_streak = 0
        return Dispatch.FOLLOW_BLACK

    if black_bg:
        # case (c): black background wins over line-ahead. At a polarity
        # seam the sensors look exactly like a junction, and treating a
        # background change as a crossing drives blindly into it and
        # resumes in the wrong mode (design.md:787-789).
        robot.straight(FORWARD_FLIP_MM)
        _case_e_streak = 0
        return Dispatch.FOLLOW_WHITE

    if line_ahead:
        # case (b): a real crossing. Steering off, fixed distance only --
        # both inner sensors are on the band and the error is meaningless
        # (design.md:791-793).
        robot.straight(FORWARD_CROSS_MM)
        _case_e_streak = 0
        return Dispatch.FOLLOW_BLACK

    if outer_l_dark != outer_r_dark:
        # case (d): exactly one outer dark -- it points straight at the arm.
        # Outer-left dark -> arm goes left -> spin CCW; outer-right dark ->
        # spin CW (design.md:821-827).
        direction = -1 if outer_l_dark else 1
        robot.turn(direction * CASE_D_SPIN_DEG)
        _case_e_streak = 0
        return Dispatch.FOLLOW_BLACK

    # case (e): both outer dark, camera found nothing ahead. Figure 2
    # guarantees a bare unmarked T-junction can't occur, so this means a
    # missed marker or a large black area -- try to recover rather than
    # give up immediately (design.md:848-853).
    _case_e_streak += 1
    if _case_e_streak >= CASE_E_MAX_STRIKES:
        _case_e_streak = 0
        return Dispatch.GIVE_UP

    _recover(robot, last_turn_rate)
    return Dispatch.FOLLOW_BLACK


def _recover(robot, last_turn_rate):
    """
    Shared recovery move for cases (a) and (e): no directional evidence from
    the outer pair, so fall back on the follower's last turn rate
    (design.md:832-846).

    UNVERIFIED SIGN (design.md:864-871): a negative turn rate means the
    follower was steering *left*. Spinning clockwise here turns *away* from
    where the line was last seen -- the green-turn logic argues the
    opposite (spin the way the follower was already steering, sweeping the
    far sensor onto the new arm). Case (a) needs a 33-99 mm bare black band,
    which no tile in the set produces, so this may never get exercised in
    practice -- but do not trust the sign below without testing it.
    """
    if last_turn_rate == 0:
        robot.straight(-RECOVERY_BACK_MM)
    elif last_turn_rate < 0:
        robot.turn(RECOVERY_SPIN_DEG)  # clockwise
    else:
        robot.turn(-RECOVERY_SPIN_DEG)  # counterclockwise


# --- self-test ---------------------------------------------------------------
#
# Runs on desktop Python (not the hub) -- checks classify_junction()'s
# dispatch against every row of design.md's exit-case table (a)-(g) before
# any of this touches real hardware.

if __name__ == "__main__":

    class _Outer:
        def __init__(self, outer_l, outer_r):
            self.outer_L = outer_l
            self.outer_R = outer_r

    class _Hub2Stub:
        def __init__(self, outer):
            self._outer = outer  # None, or (outer_l, outer_r)

        def get_outer_state(self):
            return None if self._outer is None else _Outer(*self._outer)

    class _CameraStub:
        def __init__(self, reply):
            self._reply = reply  # None, or (line_ahead, black_bg)

        def ask_junction(self):
            return self._reply

    class _RobotStub:
        def __init__(self):
            self.log = []

        def stop(self):
            self.log.append(("stop",))

        def straight(self, mm):
            self.log.append(("straight", mm))

        def turn(self, deg):
            self.log.append(("turn", deg))

    WHITE, BLACK = 90, 5  # far outside OUTER_BLACK_LEVEL either direction

    cases = [
        # (label, outer_l, outer_r, camera_reply, last_turn_rate, expect)
        ("a: both white",            WHITE, WHITE, None,           0, Dispatch.FOLLOW_BLACK),
        ("b: line ahead",            BLACK, WHITE, (True, False),  0, Dispatch.FOLLOW_BLACK),
        ("c: black background",     WHITE, BLACK, (False, True),  0, Dispatch.FOLLOW_WHITE),
        ("d: left dark only",       BLACK, WHITE, (False, False), 0, Dispatch.FOLLOW_BLACK),
        ("d: right dark only",      WHITE, BLACK, (False, False), 0, Dispatch.FOLLOW_BLACK),
        ("e: both dark, neither",   BLACK, BLACK, (False, False), 0, Dispatch.FOLLOW_BLACK),
        ("f: camera unreachable",   BLACK, WHITE, None,           0, Dispatch.GIVE_UP),
        ("g: outer state stale",    None,  None,  None,           0, Dispatch.GIVE_UP),
    ]

    for label, outer_l, outer_r, reply, rate, expect in cases:
        reset_junction_state()
        outer = None if outer_l is None else (outer_l, outer_r)
        robot = _RobotStub()
        hub2 = _Hub2Stub(outer)
        camera = _CameraStub(reply)
        result = classify_junction(robot, hub2, camera, last_turn_rate=rate)
        status = "ok" if result == expect else "FAIL"
        print("[%s] %-24s -> %s (want %s)  moves=%s" % (
            status, label, result, expect, robot.log[1:] or robot.log))
        assert result == expect, label

    # case (e) three-strike counter -> H
    reset_junction_state()
    robot, hub2, camera = _RobotStub(), _Hub2Stub((BLACK, BLACK)), _CameraStub((False, False))
    results = [classify_junction(robot, hub2, camera, last_turn_rate=0) for _ in range(CASE_E_MAX_STRIKES)]
    assert results == [Dispatch.FOLLOW_BLACK] * (CASE_E_MAX_STRIKES - 1) + [Dispatch.GIVE_UP]
    print("[ok] e: three-strike counter -> %s" % results)

    print("all cases match design.md's exit-case table.")
