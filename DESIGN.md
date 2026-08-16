# RoboCup Junior Australia 2026 — Open Rescue Line: Design

**Status: working draft.** This records decisions taken so far and the reasoning
behind them. Several sections are still open; those are marked **OPEN**. Expect
this document to be revised as the design is worked through.

Team context: NSW, Open division. Rules referenced are
[RCJA Rescue Line 2026 v26.0](https://www.robocupjunior.org.au/wp-content/uploads/2026/02/RCJA-Rescue-Line-Rules-2026.pdf)
(17 Feb 2026) and the
[2026 RCJA General Rules](https://www.robocupjunior.org.au/wp-content/uploads/2026/02/2026-RCJA-General-Rules.pdf).
Tile geometry is measured from the official SVGs at
<https://rcja.app/rcj_cms/line/tiles/>.

---

## 1. What the rules force

These are the constraints that actually shape the design, with the numbers that
matter.

### Scoring, and where the effort belongs

| Element | Points |
| --- | --- |
| Tile, continuous line | 10 |
| Tile, discontinuous line | 15 |
| Intersection marker followed | +5 |
| Speed bump / debris / bridge / see-saw | +5 each |
| Obstacle negotiated | +10 |
| **Per victim: control 10 + rescue 15 + correct evacuation point 15** | **40** |
| Exit the zone via the access point and reacquire the line | 20 |
| All scorable elements completed | 20 |
| Lack of Progress | −5 each, capped at −20 |

Three victims plus the exit bonus is **140 points**, roughly 40% of a realistic
total, and it gates the 20-point completion bonus. The evacuation zone is the
priority.

### Rules with direct design consequences

- **Control requires a lift** (§6.3.8.3). The victim must leave the field
  surface and stay under control while moving. Pushing or dragging scores zero.
- **Never end the run early.** §6.6.7 applies the full −20 to a team that
  declares an end before the timer expires, and §6.6.9 records maximum time
  anyway if not every element is complete. There is no state in which stopping
  is better than continuing. The program must have no "finished" state — after
  the exit bonus it should go back to line following until the clock runs out.
- **Game length 300 s**, plus 30 s optional calibration.
- **No pre-mapping or dead reckoning** from predefined knowledge (§3.2.3).
  In-run odometry, gyro and perception are all fine; what is prohibited is
  configuring the program with the course layout in advance.
- **Distributed control is allowed** (§3.2.2) but the team must be able to show
  that third-party wireless control is disabled. Two hubs on BLE is fine; be
  ready to demonstrate it at inspection.
- **Doorway is 270 × 270 mm** for Open, and the evacuation-zone entrance is
  narrower still at **228 mm**. That is the binding constraint on robot width.

### Field geometry, measured from the official tiles

- Line width **15.8 mm** (small tiles) / 15.0 mm (large tiles).
- Green intersection markers are **39.6 mm square with the inner edge flush
  with the line edge**, so a marker spans **7.9 mm to 47.5 mm** from the line
  centre.
- Discontinuous line gaps are **59.4 mm** (Broken Road, Gap Cross).
- Corner radii on Square Corner and Inverted Square Wave are **0.3–2 mm** —
  effectively true 90° corners, despite §2.2.2 promising a 40 mm minimum.
- **Inverted-polarity tiles exist in the standard small tile set**: Night Drive,
  Inverted Square Wave, Ramp Night Drive — black background, white line. This
  contradicts §2.1.4's "uniform white background"; the tile art wins.
- NSW, so the WA0 tile set (Silver Lining, Yellow Brick Road, Pause, …) does not
  apply.

### Figure 2 of the rules — the junction guarantee

The figure enumerates every legal junction case. Two conclusions:

1. **There is no two-marker case.** "Green on both sides = U-turn" is not an
   RCJA case, and no tile in any set has markers straddling the line.
2. **A bare unmarked T-junction is "Impossible" from every approach.** If the
   line does not continue straight, a marker is guaranteed to be present.

Conclusion 2 is what allows the junction logic in §5 to stop consulting the
camera on the normal path.

### Evacuation zone

- **1188 × 891 mm**, white walls ≥ 100 mm.
- The black line **ends** at the access point; entry is a smooth reflective
  strip ≥ 25 × 250 mm, through a **228 mm** opening on a long (1188 mm) wall.
- **Exactly two live victims (silver, reflective, conductive) and one dead
  victim (black)** — spheres 40–50 mm diameter, off-centre centre of mass,
  ≤ 80 g, placed randomly on the floor.
- Evacuation points are right-angled triangles, 280 × 280 mm legs, **60 mm
  walls**, hollow centre, in referee-chosen non-entry corners. Per Figure 6 they
  sit in the two corners of the wall **opposite** the entrance. Green takes the
  live victims, red the dead one.
- **Red appears nowhere else on an Open course**, which makes it the highest-
  confidence signal that the robot is at or in the zone.

> Note: rule 2.9.4 still describes an Open "empty capsule" (a black 375 ml can).
> The change log says empty capsules were removed and §2.9.3.1 contradicts it.
> Confirmed with the team: **three spheres only, two silver and one black.**

---

## 2. Hardware

### Hub 1 — full

| Port | Device |
| --- | --- |
| A | left inner colour sensor |
| B | left wheel motor |
| C | ultrasonic distance sensor |
| D | OpenMV H7 Plus (PUPRemote) |
| E | right inner colour sensor |
| F | right wheel motor |

### Hub 2 — ports A and E spare

| Port | Device |
| --- | --- |
| B | left **outer** colour sensor |
| C | claw motor |
| D | lifter motor |
| F | right **outer** colour sensor |

### Sensor array geometry

Sensor centres are 33 mm apart, so relative to the line centre:

```
   -49.5      -16.5    0    +16.5      +49.5      mm
  outer L    inner L  line  inner R   outer R
   (hub 2)   (hub 1)        (hub 1)   (hub 2)
```

Against the measured 15.8 mm line and 39.6 mm marker:

- The inner pair straddles the line, and sits **inside** the marker footprint
  (7.9–47.5 mm) — so **green detection belongs on the inner pair**.
- The outer pair sits **2 mm beyond the marker's far edge** and is therefore
  useless for green. Its value is junction classification (§5).
- With the line edge at ±7.9 mm and the inner sensors at ±16.5 mm there is a
  **±4–5 mm steering deadband** where neither inner sensor has touched the line.
  Not fatal, but it explains why the tuning is delicate and why the sharp-corner
  tiles are hard.

### Victim handling

A claw on a lifter: one motor raises/lowers, one opens/closes. Bench-tested and
working. The lifter must release above the 60 mm evacuation-point wall.

**OPEN:** measure the claw's *capture envelope* — the lateral offset and range
of stop distances that still result in a grip. Those two numbers are the
accuracy specification for the vision system, and nothing downstream can be
tuned sensibly without them.

---

## 3. System architecture

Three processors. Hub 1 is the sole decision-maker; it owns the drive base and
every latency-critical sensor. The other two are peripherals.

```
        ┌──────────────────────── HUB 1 ────────────────────────┐
        │  drive base, inner colour pair, ultrasonic            │
        │  run-level state machine, all decisions               │
        └───────┬───────────────────────────────┬───────────────┘
       PUPRemote│ (port D, hub-initiated)       │ BLE broadcast
                ▼                               ▼
        ┌───────────────┐               ┌───────────────┐
        │   OpenMV      │               │    HUB 2      │
        │ pan/tilt cam  │               │ claw, lifter, │
        │               │               │ outer sensors │
        └───────────────┘               └───────────────┘
```

### Neither link is request-response

This is a property of the transports, not a design preference, and it is the
reason for everything that follows.

**PUPRemote is a shared variable.** Hub 1 is master on the LPF2 link. When it
calls `camera.call(...)` it writes the request bytes and reads back *whatever
the camera most recently put on the wire* — not a value computed in response to
that call. The camera cannot reply to a specific request. The existing code
already compensates for this by discarding one reading and waiting a frame,
because *"the camera's answer can be up to a frame old, quite possibly computed
while the robot was still moving into the junction."* The same code also notes
the link *"goes stale after about a second with no poll and then needs a slow
reconnect"* — so hub 1 must poll continuously whether or not it wants an answer.

**BLE broadcast has no reply channel.** Pybricks hub-to-hub messaging is
connectionless advertising: `broadcast()` sets what a hub advertises,
`observe()` returns the last thing heard, and there is no addressing or
acknowledgement. A request-and-wait exchange would cost two advertising
intervals before hub 1 learned anything, both lossy. Continuous broadcast makes
`observe()` an instant local read instead — which is why §5's outer-sensor check
is a cached lookup rather than a round trip.

### The consequences

**Peripherals precompute; they never compute on demand.** Each runs a
free-running loop and keeps a fresh answer ready, so a poll is a read rather
than a round trip. Making them wait for a command would add a full frame of
processing (camera) or two advertising intervals (hub 2) to every query.

**Hub 1 commands *what* they work on, not *when*.** That is the command
direction, and it is configuration rather than triggering:

| | hub 1 sends | peripheral does | hub 1 reads |
| --- | --- | --- | --- |
| camera | which question to answer | answers it every frame | latest answer, instantly |
| hub 2 | desired actuator goal | drives toward it | actual state, instantly |

**"Waiting for a reply" becomes two different things.** For the camera: after
changing mode, discard a reading and wait ~1 frame, since the payload in flight
may predate the change. For hub 2: wait until the *observed actual state*
matches what was asked — a convergence check, naturally tolerant of packet loss.

**Nothing latency-critical crosses BLE.** The steering error is computed
entirely from hub 1's inner pair. Hub 2 contributes classified state at
10–20 Hz, read only at decision points where the robot is slow or stopped.

**Both BLE directions carry continuously-repeated state, not edge-triggered
commands**, so a dropped packet costs one cycle rather than desynchronising the
hubs. The existing lifter program already has this property; keep it.

### Hub 1 ↔ hub 2 protocol

Two channels, both broadcast continuously.

| channel | direction | payload |
| --- | --- | --- |
| 1 | hub 1 → hub 2 | `(seq, goal)` |
| 2 | hub 2 → hub 1 | `(seq_echo, phase, grip, outer_L, outer_R)` |

| field | values |
| --- | --- |
| `goal` | `STOW` · `CAPTURE` · `RELEASE` |
| `phase` | `IDLE` · `MOVING` · `DONE` · `FAULT` |
| `grip` | `UNKNOWN` · `EMPTY` · `HOLDING` · `BLOCKED` |
| `outer_L`, `outer_R` | raw reflection, 0–100 |

Encoded on the wire as single characters for `goal`, `phase` and `grip`, which
keeps the tuple well inside the advertisement size limit and readable on hub 2's
display.

**Commands are levels, not verbs.** Broadcast is lossy and continuously
repeated, so an edge-triggered command can be missed or seen twice; a level is
idempotent.

**`seq` exists so the same goal can be requested twice.** Levels alone cannot
express a retry — re-sending `CAPTURE` after a failed capture is a no-op. Hub 1
increments `seq` on every new command and hub 2 echoes it, so completion is
`seq_echo == seq and phase == DONE`, unambiguous across retries and tolerant of
loss in both directions.

**The outer sensors send RAW reflection values, not a classification.** An
earlier version of this protocol had hub 2 report `DARK`/`BRIGHT`. Raw is
better and costs the same on the wire: hub 1 owns the polarity *and* will own
the 30 s calibration, so if hub 2 thresholded with its own constants the two
hubs could disagree about what "dark" means. One source of truth.

The original argument for classifying locally was about not streaming raw data
for a steering loop, which still holds — but that is about *latency*, not about
who applies the threshold.

They do not report green either: at ±49.5 mm they sit 2 mm outside a 39.6 mm
marker, so green detection belongs to hub 1's inner pair (§2).

**Timeouts.** If `phase` never reaches `DONE` — a jam, or hub 2 gone — hub 1
gives up: **Q** inside the zone, **H** on the line course.

### Hub 2 state machine

Two things run side by side: a sensor path that is not a state machine, and an
actuator executor that is.

```
IDLE ──── new seq ────→ MOVING ─┬── all steps complete ──→ DONE
                                └── step timeout ────────→ FAULT

DONE  ──── new seq ────→ MOVING
FAULT ──── new seq ────→ MOVING
```

`DONE` and `FAULT` **latch until the next `seq` arrives**. This is what makes
the protocol survive a lossy link: the result stays readable indefinitely, so
hub 1 can miss any number of broadcasts and still learn the outcome. An
executor that returned to `IDLE` on its own could lose a verdict entirely.

```
CAPTURE
  0  lifter → grab height           wait for done
  1  claw   → open                  wait for done
  2  claw   → close until stall     wait for stall
  3  evaluate claw angle → grip
       EMPTY    → DONE          (hub 1 retries, or re-approaches)
       BLOCKED  → DONE          (something larger than a ball)
       HOLDING  → step 4
  4  lifter → carry height          wait for done
  5  → DONE

RELEASE
  0  lifter → release height (clears the 60 mm wall)  wait for done
  1  claw   → open                                    wait for done
  2  claw   → close until stall                       wait for stall
  3  evaluate claw angle
       EMPTY    → ball is gone: released → step 4
       HOLDING  → still gripping → DONE   (hub 1 retries)
  4  lifter → carry height, empty                     wait for done
  5  → DONE

STOW
  0  claw   → closed                wait for done
  1  lifter → park                  wait for done
  2  → DONE
```

Step 2 of `RELEASE` is the re-close test: the claw motor's stall angle is the
only way to know the ball left, and it is observable only on hub 2. It also
helps on failure — if the ball never left, hub 2 has just re-gripped it, so
hub 1 can retry without a fresh capture.

Four rules the loop must obey:

1. **Sensors and broadcast run unconditionally**, outside the executor. Whatever
   the claw is doing, hub 2 reads both sensors and publishes every cycle. That
   is what lets hub 1 distinguish "busy" from "crashed" — a hub 2 mid-capture
   keeps reporting `MOVING`.
2. **Every step is non-blocking.** `run_target(..., wait=False)` and test
   `done()`; one step advances per loop cycle. A blocking call silences the
   broadcast for its duration, and silence is indistinguishable from a crash.
3. **Every step has a deadline.** Exceeding it means `FAULT`, not waiting
   forever.
4. **`observe()` returning `None` means hold.** Never open, never move. The
   existing lifter program already defaults to stop-and-hold on a missing or
   malformed command — but note that for the claw the safe default is *keep
   gripping*, since dropping a victim mid-transit costs the full 40 points.

The same reasoning applies on `FAULT`: **hold both motors**, do not brake or
coast. A jam is bad; dropping a victim already under control is worse.

#### Constraints found while implementing

**`run_until_stalled()` cannot be used.** It blocks and has no `wait=False`, so
it would silence the broadcast for the whole close — breaking rule 1 and making
a busy hub 2 indistinguishable from a dead one. Closing is therefore issued as a
plain `run()` with `Motor.stalled()` polled each cycle. This is the obvious
thing to "simplify" later; do not.

**The claw needs a torque ceiling.** A victim is an 80 g plastic sphere and a
claw closing at full torque can crush it or squirt it out sideways. Set via
`Motor.control.limits(torque=...)`.

**No startup homing — the claw and lifter are placed at home by hand**, and
`reset_angle(0)` records that as the reference. Every angle, including the grip
verdict window, is relative to it. The consequence is worth knowing: a run
started with the claw somewhere other than home will misreport `HOLDING` and
`EMPTY`, and nothing downstream can detect that.

`observe()` is non-blocking: the radio fills a buffer in the background and the
call reads it. Pybricks returns `None` when nothing has been heard for roughly a
second — worth confirming against the firmware in use, since that same staleness
is what X case (g) and M case (g) key on, so it comes for free.

### Camera: one program, two personalities

The hub selects the mode in the 4-byte request (this already happens for
`BLK?` / `WHT?`). The 8-byte `hhhh` reply — a power-of-two payload is required
for a valid LPF2 frame — serves both jobs:

| Mode | Reply `(h, h, h, h)` |
| --- | --- |
| LINE | `ahead, angle, length, coverage` (as today) |
| ZONE | `target_kind, bearing°, distance_mm, confidence` |

`target_kind` ∈ none / dark ball / silver ball / green point / red point / exit
strip. The hub says what to look for; the camera does not guess.

**Pan/tilt as a bearing sensor.** In ZONE mode the camera should close the
tracking loop locally at frame rate and report the pan and tilt angles it
settled at — those *are* the bearing and range to the target. Hub 1 reads two
numbers instead of running a servo loop across the PUP link, and the camera can
tilt down to keep a ball tracked as the robot closes on it.

**OPEN:** exact request byte layout and the ZONE-mode command set.

---

## 4. Hub 1 state machine

```
S ──→ F                                 S = armed, waiting for the button
      │
      ├──→ X ──┬──→ F     corner, or crossing driven through  (§5 a, b, d)
      │        ├──→ W     black background ahead = flip       (§5 c)
      │        └──→ H     camera or hub-2 link unusable       (§5 f, g)
      │
      ├──→ L ─────→ F     green marker left  → turn left   (always to F)
      │
      ├──→ R ─────→ F     green marker right → turn right  (always to F)
      │
      ├──→ O ──┬──→ F
      │        └──→ H
      │
      └──→ B ──┬──→ F     the line is still ahead — spin to it and resume
               ├──→ A     a sphere was located — go straight for it
               ├──→ V     zone confirmed, but no sphere yet — survey
               └──→ H     nothing found, or the camera is unreachable

      W ──→ M ─┬──→ F     back on white background, aligned to the line
               ├──→ W     false alarm — nudge and retry
               └──→ H     exhausted, or a stale link

      K ──────────→ F     leave the zone, reacquire the line  (+20, §7)
      H ──────────→ F     after a button press — never to W
```

**There is no direct `F → W` edge.** Every polarity flip is reached through X.
The colour sensors cannot tell a seam from a crossing on their own — at both,
all four go dark at once, because the crossing arm runs parallel to the sensor
row — so any direct edge would need a camera-derived qualifier and would become
a second place where polarity can be decided. A wrong flip inverts the steering
sign and loses the line within a step, so that decision is kept in one place.

The green turn is split into two states, **L** and **R**, rather than one
parameterised state: the behaviour is mirrored, but explicit states keep the
transition free of a payload and allow the two to be tuned separately. The
recovery state is **H** (hunt) to leave R free for the right turn.

**L and R have no edge to H**, deliberately. A turn that fails to find the
branch leaves the robot on all-bright, which F reads as line-lost, which is B,
which already routes to H — so recovery arrives via `L/R → F → B → H` through a
path that has to exist anyway. See §6.

| | State | Behaviour |
| --- | --- | --- |
| S | armed | wait for the button; do not move |
| F | follow black line | PD steering on the inner pair |
| W | follow white line | same, error sign inverted |
| X | junction classifier | read outers, ask the camera, stop, then dispatch |
| M | seam classifier | the W-mode counterpart of X: is the flip back real? |
| L | green turn, left | back up, pivot on the inner wheel, hunt the branch |
| R | green turn, right | mirror of L |
| O | obstacle bypass | fixed right-side detour, then rejoin the line |
| B | zone check | raise the camera, spin 360° scanning for zone targets |
| V A C D T K Q | evacuation zone | seven states, not one — see §7 |
| H | give up | reset everything, wait for the button, resume in F |

Design notes:

- **Precise mode is a modifier on F, not a state.** Splitting it would double
  every transition for no benefit. Keep it orthogonal.
- **Ramps and the see-saw are not states.** They change dynamics, not
  behaviour. Expose gyro pitch as a modifier available in both F and W — Ramp
  Night Drive needs it in W.
- **H does not search.** It stops, resets, and asks for a human. See below.
- **There is no terminal state**, by design — see §1 on never ending early.

### Following the line (state F)

#### What the sensor pair can actually see

The geometry rules out more than it allows. With the inner sensors at ±16.5 mm,
a line half-width of 7.9 mm and a sensor spot roughly 9 mm across, the transfer
function against robot offset *e* from the line centre is:

| *e* (mm) | left sensor | error `L − R` |
| --- | --- | --- |
| 0 – 4 | on white | **0** — deadband |
| 4 – 13 | line entering the spot | rising, roughly linear |
| 13 – 20 | fully on the line | **saturated** |
| 20 – 29 | line leaving the spot | falling |
| > 29 | on white again | **0** — but the line is lost |

Three consequences:

- **The linear region is about 9 mm wide.** Beyond 13 mm the controller is
  effectively bang-bang. There is no value in an elaborate control law for a
  signal with that little proportional content.
- **No integral term.** The deadband is a region where the plant produces no
  feedback at all. Integral action cannot observe it, so it winds up and then
  overshoots when the signal returns.
- **Error = 0 is ambiguous** — either centred, or 29 mm off and driving away.
  This is exactly why F → B counts distance travelled rather than reacting to
  the error going quiet.

#### Keep the common mode, not just the difference

`L − R` discards the information that says whether the robot is on the line at
all. Carry `min(L, R)` alongside it:

- both bright → inside ±4 mm **or** lost
- one dark → in the 4–29 mm band, so the error is meaningful

> **Correction — distance travelled does NOT separate those two cases.**
> An earlier version of this section claimed it did, and the F → B guard was
> built on that claim. It is wrong, and the arithmetic says so.
>
> Centred on the line, *neither* inner sensor is over it: the sensors sit at
> ±16.5 mm with a spot roughly 9 mm across, so the left one covers −21 to
> −12 mm while the line only reaches −7.9 mm. And partial overlap barely moves
> the reading — reaching `LINE_DARK_LEVEL` needs about 72% of the spot on
> black, which does not happen until the robot is **~11 mm off centre**:
>
> | offset | reading | resets the counter? |
> | --- | --- | --- |
> | 0–4 mm | 100 | no |
> | 8 mm | 61 | no |
> | 11 mm | 31 | yes |
> | 16 mm | 10 | yes |
> | 25 mm | 61 | no |
> | 29 mm | 100 | no |
>
> So the counter resets only on a substantial excursion, and grows while
> following *correctly*. A robot tracking a 594 mm Straight tile well can reach
> the threshold and enter B on the easiest part of the course.
>
> **Recommended fix, not yet applied:** corroborate with the camera — enter B
> only when no sensor has seen the line for `LINE_LOST_MM` *and* the camera
> reports no line ahead. The heartbeat already runs, so that is a cached read
> rather than a new round trip. Alternatives considered: loosening
> `LINE_DARK_LEVEL` to ~80 (cheaper, but a robot genuinely holding ±4 mm still
> never darkens either sensor), or raising `LINE_LOST_MM` above the longest
> straight (removes the false positive but delays real detection until the
> robot is well off the tile, which §6.4.1.8 penalises). See open question 16.

#### The control law

```
loop at ~100 Hz:
    l = normalise(left.hsv().v)          # 0 = black, 100 = white, from calibration
    r = normalise(right.hsv().v)
    error = (l - r) * polarity

    speed = V_MAX - (V_MAX - V_MIN) * min(1, abs(error) / ERROR_FULL)
    turn_rate = clamp(K * error * speed, ±TURN_MAX)

    robot.drive(speed, turn_rate)
```

Four deliberate choices:

**Continuous `drive()`, not blocking steps.** The current implementation calls
`robot.turn(x)` then `robot.straight(1 mm)`, both blocking — a 3° correction at
50 °/s costs 60 ms while the 1 mm of travel costs 28 ms, so most of each cycle
is spent turning and the effective forward speed collapses. A control loop over
a non-blocking `drive()` is both faster and smoother. This is the likely
resolution of open question 3.

**Curvature, not turn rate.** Multiplying by `speed` makes the commanded path
curvature independent of speed, so raising the speed does not invalidate the
gain. The existing design already has this property — the note about turning per
fixed distance being a curvature — and it must be preserved in the continuous
form.

**Speed scheduling is the main performance lever.** Slow on error, fast when
centred. Because the deadband genuinely means "within ±4 mm", both-bright is a
trustworthy go-fast signal rather than a defect. This is what allows a
straight-line speed high enough to fit the 300 s budget without losing corners.

**Everything reads `hsv()`; nothing reads `reflection()`.** They are different
device modes on a Powered Up sensor and switching between them costs a round
trip — so taking green from `hsv()` and brightness from `reflection()` would pay
that cost on both sensors every tick, against the loop rate the whole design
depends on. One `hsv()` per sensor per tick is cached and serves the green gate,
both band tests, the line-lost counter and the control law; only green
*confirmation* takes extra reads, and only with a marker under a sensor.

The consequence is that every threshold is on the hsv **value** scale rather
than the reflection scale, and needs tuning against the real sensor.

**Normalised readings.** Calibrate white and black in the 30 s window (§6.1.2)
and scale to 0–100. Every gain and threshold then becomes venue-independent,
and `LINE_BLACK_LEVEL`, `CROSS_BLACK_LEVEL` and the rest stop being constants
tuned to one lighting setup.

#### Response curve

The implemented constants:

| constant | value | |
| --- | --- | --- |
| `V_MAX` | 100 mm/s | speed at error ≈ 0 |
| `V_MIN` | 40 mm/s | speed at or beyond `ERROR_FULL` |
| `ERROR_FULL` | 60 | error magnitude at which speed reaches `V_MIN` |
| `STEER_GAIN` | 0.04 | deg/mm of curvature per unit of error |
| `TURN_MAX` | 250 deg/s | clamp |

which give:

| error | speed | turn rate | curvature | radius |
| --- | --- | --- | --- | --- |
| 0 | 100 | 0 | — | straight |
| 5 | 95 | 19 | 0.20 °/mm | 286 mm |
| 10 | 90 | 36 | 0.40 °/mm | 143 mm |
| 20 | 80 | 64 | 0.80 °/mm | 72 mm |
| 30 | 70 | 84 | 1.20 °/mm | 48 mm |
| 45 | 55 | 99 | 1.80 °/mm | 32 mm |
| 60 | 40 | 96 | 2.40 °/mm | 24 mm |
| 90 | 40 | 144 | 3.60 °/mm | 16 mm |
| **corner branch** | 40 | 250 | 6.25 °/mm | **9 mm** |

Two things to read off it. The 40 mm minimum legal curve radius (§2.2.2) lands
at about **error 35**, comfortably inside the proportional range rather than up
against a limit. And `TURN_MAX` binds **only in the corner branch** — which is
the intent: proportional where the geometry is gentle, bang-bang where it is
not.

> **The old gain does not transfer.** `TURN_PER_ERROR = 0.27` in the previous
> follower meant 0.27° per 1 mm step, i.e. a curvature of `0.27 × error` deg/mm.
> That reaches the tightest *legal* curve at an error of only 5, and asks for a
> **2.4 mm radius** at full error — 2430 °/s once multiplied by a continuous
> 100 mm/s. It worked in a stop-start regime with 1 mm steps; in continuous
> driving it would simply live in the clamp, which is bang-bang with extra
> steps. `STEER_GAIN` is roughly 8× lower for that reason.

#### Corner detection

The tiles have near-zero-radius corners — 0.3–2 mm on Square Corner and Inverted
Square Wave — and a proportional follower will overshoot them, because the line
leaves the sensors before the robot has turned far enough.

No extra sensor is needed: **sustained saturation is a corner.** A gentle curve
saturates the error briefly; a true corner holds it. If `|error|` stays at
maximum for more than a few millimetres of travel, drop to `V_MIN` and command
maximum curvature until it breaks.

#### Precise mode

Falls out as a second `(V_MAX, K, TURN_MAX)` triple, consistent with it being a
modifier rather than a state.

#### Guard ordering within the loop

1. green → **L** / **R**
2. obstacle, from the ultrasonic → **O**
3. both inner sensors dark → **X**
4. distance since either sensor last read dark ≥ 200 mm → **B**
5. otherwise, steer as above

Green outranks the band test because a marker sits immediately before its
junction; losing that race means driving straight through a turn the course just
instructed.

#### W is the same law, with two guards changed and one removed

White-line mode runs the identical loop with `polarity` inverted, which flips
the sign of the error. The deadband and the saturation behaviour are unchanged.
Three guards differ:

- **Guard 1 (green) does not apply.** Inverted tiles carry no intersection
  markers, so the check is skipped — which also saves two `hsv()` reads per
  cycle. This is what §6's L/R guard means by "the robot is in F".
- **Guard 3 becomes both inner sensors reading *bright*** (→ **M**).
- **Guard 4 (line lost) is deliberately absent.**

In W, centred and lost are *both* "both sensors dark", exactly mirroring F — so
detecting loss would need a distance-since-**bright** counter. It is left out on
purpose, for a stronger reason than simplicity: **losing the line in W is
self-limiting.** With the error at zero the robot drives straight, leaves the
inverted tile within one tile length, and lands on white background — which is
both-bright, which is M, which returns it to F. F's own line-lost counter takes
over from there.

Only three tiles in the whole set are inverted — Night Drive, Inverted Square
Wave and Ramp Night Drive — and all three are single unbranched lines, so there
is little to get lost on.

The exception is **Ramp Night Drive**, where that straight run happens on a 20°
incline and driving off the tile edge is a real risk; §2.5.4 makes clear no
assistance is given to a robot that does.

### Giving up (state H)

H makes no attempt to recover on its own. It stops, resets, and waits for the
Robot Handler to reposition the robot at a Start Location and press the button.
That is a deliberate Lack of Progress under §6.4.1.2.

**The cost is time, not points.** §6.6.6 caps the Lack of Progress deduction at
**20 points in total**, so the first four entries cost 5 each and every one
after that is free. What H actually costs is the clock: the robot is replaced at
City Limits and must re-drive tiles that §6.6.5 will not score a second time.

#### H is a reset, not just a wait

| | reset to |
| --- | --- |
| polarity | **black** — free, see below |
| camera tilt and mode | line-following aim, LINE mode |
| hub 2 goal | `STOW` |
| lockouts, precise mode, counters | cleared |
| drive base | stopped |
| display | a character indicating why it stopped |

Then debounce the button as `main()` already does — wait for release, then
press — so a held button cannot immediately restart the robot.

**Resetting the polarity is the critical one** — and it turns out to be free.
Entering H from W and resuming in F with the polarity still set to white would
invert the steering sign and lose the line within a single step, the most
damaging failure in the design.

But **there is no polarity variable**. F and W are separate states calling one
shared control law with `+1` and `-1`, so being in F *is* black polarity, and
`H → F` resets it by construction. Nothing to remember and nothing to forget.
The camera request byte derives from the current state for the same reason.

#### Why H → F only, never W

The handler repositions at City Limits or a Drop Zone, both of which are
ordinary black line on white. There is no route back into W that does not go
through a fresh seam detection in X.

#### H and S are the same behaviour

Once H performs a full reset and waits for a button, it is behaviourally
identical to S — S simply starts from already-clean state. They are kept
separate here only so the display character can distinguish "armed" from "gave
up", which matters when diagnosing a run. Merging them is a reasonable
simplification if that diagnostic is not wanted.

---

## 5. Classifier states (X and M)

X classifies what a band of black means while following a black line. M is its
counterpart in white-line mode, deciding whether the flip back is real.

### Junction logic (state X)

#### Transition guard: F → X

**Both inner sensors read black.** Nothing else.

Note this makes X an **F-only** construct. In W the background is black, so both
inner sensors read dark during ordinary following and this guard would be
permanently true. The flip back to a black line therefore needs its own
mechanism — the mirror trigger is both inner sensors reading *white* — which is
state **M**, below.

The outer pair is deliberately *not* consulted at the transition. X is a
**classifier**, not a traverse: it reads sensors, asks the camera if needed,
stops, and only then commits to an action. Because it does not drive first, a
corner resolving inside X costs nothing but the classification time — whereas a
state that drove straight on entry would leave the line at every corner.

Keep the black threshold for this test stricter than the one used to decide
"on the line". With the inner sensors at ±16.5 mm and the line at ±7.9 mm
neither should be over the line during normal following, but on a tight curve
one rides onto it, and a single shared threshold would eventually fire mid-line
and drive the robot straight through a bend.

Speed bumps and debris cannot trigger this: §2.4.1 makes bumps "a similar colour
to the tile's background" and §2.4.3 forbids debris from being a colour
otherwise used on the course.

#### Actions inside X

1. Read the latest outer-sensor state broadcast by hub 2. This is a cached
   lookup, not a round trip.
2. If **either** outer sensor reads black — i.e. three or four sensors black —
   ask the camera two questions: *is there a straight black line ahead?* and
   *is there a black background ahead?*
3. Stop the robot, then dispatch.

Stop before believing the camera, and discard one frame: the answer can
otherwise be a frame old, computed while the robot was still moving into the
junction.

#### Exit cases

| | outer state | camera | action | → |
| --- | --- | --- | --- | --- |
| a | both white | not asked | recovery move, below | **F** |
| b | ≥1 black | straight line ahead, no black background | forward 18 mm | **F** |
| c | ≥1 black | black background ahead | forward 10 mm | **W** |
| d | exactly 1 black | neither | spin 45° toward the dark outer, below | **F** |
| e | both black (4 total) | neither | recovery move, below | **F** |
| f | ≥1 black | unreachable / no answer | none | **H** |
| g | outer state stale | — | none | **H** |

Black background takes precedence over line-ahead: at a polarity seam the
sensors look exactly like a junction, and treating a background change as a
crossing drives blindly into it and resumes in the wrong mode.

Case (b) drives a **fixed 18 mm** with steering off. Steering must be off: while
both inner sensors sit on the band the error is meaningless, and acting on it is
exactly what makes the robot hook onto the crossing arm and follow it.

A fixed distance rather than "drive until an inner sensor comes clear". The
closed-loop version is more robust in principle, but it depends on the inner
thresholds being trustworthy at the moment the robot is sitting on a band -
which is when they are least so - and a stop condition that fires immediately
leaves the robot still on the crossing. A fixed step is predictable during
bring-up and always makes progress.

Watch the margin: a crossing arm is 15.8 mm wide *square on*, but at 30° of yaw
its effective width is 15.8/cos 30° = 18.2 mm, already more than 18 mm - and
detection happens partway onto the band rather than at its leading edge. If a
crossing ever needs two passes to clear, this is the number to raise.

#### The recovery move, cases (a), (d) and (e)

All three mean "this is not a crossing and there is nothing ahead", and none can
simply hand back to F as it stands: sitting on the band both inner sensors are
dark, so the error is ~0 and F would drive straight on into it. Each has to move
first — and each uses the best information it actually has, which is not the
same information.

The split follows from what the outer pair can tell you. **Only the asymmetric
case carries direction:** in (a) both outer sensors are white and in (e) both are
black, and neither extreme says anything about which way the line went. (d) is
the one case where one outer sensor is dark and the other is not, and that
difference points straight at the arm.

**Case (d) — spin 45° toward the dark outer sensor.**

Exactly one outer sensor is black here, and *which* one says directly which side
the arm extends to. Outer-left dark means the arm goes left, so spin
counterclockwise; outer-right dark means spin clockwise. That is present
evidence about the course, not an inference from what the robot was doing a
moment ago, so it is preferred wherever it is available.

The classification already computes `left_dark != right_dark` to reach this
case; it simply has to keep which was which.

**Cases (a) and (e) — fall back on the follower's last turn rate.**

Here the outer pair is uninformative — both white in (a), both black in (e) —
so there is no directional evidence at all and the only hint is what the
follower was doing when it hit the band. Taking the last turn rate
`follow_line()` commanded before X was entered, positive being clockwise:

| last turn rate | move |
| --- | --- |
| zero | backward 8 mm |
| negative | spin **clockwise** 45° |
| positive | spin **counterclockwise** 45° |

Zero means the robot met the band while running straight, so there is no hint at
all and backing off is the only option left.

**Case (e) used to go straight to H.** Four black sensors with nothing ahead
means a marker was missed — Figure 2 guarantees a bare unmarked T-junction
cannot occur — or the robot is on a large black area. Giving up there cost a
Lack of Progress and a re-drive of the course; trying to recover first is
cheaper, and §1's never-stop rule argues for attempting something over
stopping.

> **This leaves case (e) unbounded.** If the recovery does not clear the band,
> X re-fires, recovers again, and can cycle indefinitely without making
> progress. M's case (b) has exactly this shape and solves it with a
> three-strike counter before falling through to H; (e) needs the same, or the
> cross-lockout that §5 still lacks.

This needs F to record its last commanded turn rate somewhere X can read it —
new state that does not exist yet.

> **The sign here is unverified.** A negative turn rate means the follower was
> steering *left*, which it does when the line has drifted left — so if the band
> is a corner, the arm probably also goes left, and spinning *clockwise* turns
> away from where the line was last seen. The green-turn logic argues the
> opposite: spin the way the follower was already steering, which sweeps the far
> sensor onto the new arm. Case (a) is close to unreachable in the first place —
> it needs a black band 33–99 mm wide, which no tile in the set produces — so
> this may never matter, but do not assume the sign is right.

#### Why the camera is still asked when only three sensors are black

An earlier version of this design skipped the camera for the three-black case,
on the grounds that a corner's arm extends one way only and so can never darken
the far outer sensor — making three black deterministically a corner.

**That reasoning does not hold.** It assumes simple corners and perpendicular
crossings. At a junction with a *curved* branch — Cross Curves, the roundabouts,
Gridlock — the branch has already bent away from where the outer sensor sits, so
three black occurs on what is not a corner at all, and case (d)'s reverse would
be exactly the wrong response.

Bench testing also found yawed corners rarely produce three black in practice,
so the camera query is paid rarely and there is little to save.

#### Staleness, not reachability

Cases (f) and (g) must key on **staleness thresholds**, not on a single failed
read. BLE broadcast drops individual packets routinely on a healthy link, and
one timed-out camera poll is not a dead camera. Define both as "no fresh data
for more than *N* ms", with N comfortably above the normal update interval —
otherwise the robot will divert to H at random.

#### Decisions recorded, with their trade-offs

**Case (a) does not move before returning to F.** Both inner sensors are still
black on re-entry to F, so the error is ~0 and X may re-trigger on the next
step. This is a stutter rather than a hang — hub 2's state is cached, so
re-classification is cheap — but step 3 stops the robot each time, so a wide
band may be crossed in stop-start fashion. Accepted; watch for it on a real
corner. Note (a) is also close to unreachable: both inner black with both outer
white needs a black band 33–99 mm wide, which no tile in the set produces.

**Case (c) commits the polarity flip on a single camera reading.** The current
code creeps 15 mm and asks a second time, because the consequences are
asymmetric — a wrong flip inverts the steering sign and loses the line within a
step, whereas a missed crossing is recoverable. Accepted as a deliberate
simplification; if false flips appear in testing, reinstating the confirmation
step is the first thing to try.

**Suggestion, not yet adopted.** X could check cached camera coverage before
step 2: if it is already unambiguous, dispatch straight to case (c) without
stopping and without a fresh query. That recovers most of what a direct
`F → W` edge would have saved (~250 ms per seam) while keeping polarity decided
in one place.

**Cases (f) and (g) route a dead camera or a dead BLE link to H.** The trade is
that this also abandons the tile points still earnable by line-following alone,
not just the evacuation zone. Deliberate: without the camera the zone is
unreachable and the run is largely lost anyway. One consequence — if the fault
is permanent, X routes to H at every junction for the remainder of the run, so
the robot will repeatedly stop and ask to be repositioned. That degrades
gracefully with a human in the loop deciding when to stop, and after the fourth
reset the deductions are capped, so the residual cost is clock time only.

### The flip back (state M)

M is the W-mode counterpart of X, and it has to exist separately because X's
guard — both inner sensors black — is the *normal following condition* in W.

Its job is far narrower. Inverted tiles carry no junctions and no green markers:
Night Drive, Inverted Square Wave and Ramp Night Drive are all single unbranched
lines. So M never has to identify a crossing, only whether the flip back is real.

#### Transition guard: W → M

**Both inner sensors read white.**

In W the background is black and the 15.8 mm line sits between the inner
sensors, so both reading white needs a white span of at least 33 mm. The line
alone cannot produce that unless the robot is yawed roughly 60° off it.

#### Actions inside M

1. Stop the robot.
2. Read the outer-sensor state broadcast by hub 2.
3. If exactly one outer sensor reads white, ask the camera whether the
   background ahead is black or white, and whether a black line is visible and
   at what angle.

#### Exit cases

| | outer state | camera | action | → |
| --- | --- | --- | --- | --- |
| a | both white | not asked | none | **F** |
| b | both black | not asked | backward 8 mm | **W** |
| c | exactly one white | black background ahead | spin 10° — CW if outer-**left** is the white one, CCW if outer-**right** | **W** |
| d | exactly one white | white background, black line ahead | turn by the camera's **signed** line angle, pivoting near the white outer sensor | **F** |
| e | exactly one white | white background, no black line | none | **H** |
| f | any | unreachable / no answer | none | **H** |
| g | stale | — | none | **H** |

Case (a) is the common path: a squarely-crossed seam turns all four sensors
white at once, and the robot is already straddling the now-black line, so F can
take over with no camera query and no movement.

Case (d) covers both the left and right variants. The turn direction comes from
the **sign of the measured angle**, not from which outer sensor triggered — the
outer sensor tells you how the robot met the seam, the camera tells you where
the line is, and only the second should decide the turn.

#### Loop control

Case (b) carries a counter: **three consecutive entries → H**. Without it a
persistent condition walks the robot backwards down the course 8 mm at a time.

Cases (c) has **no counter**, by decision: successive 10° nudges are effectively
a search and are expected to converge. If they do not, this is the first place
to look. See open question 10.

#### Note on the case (c) spin direction

A single sensor row yields a *footprint*, not a direction — four readings give
the width and rough centre of the white region but not which way the line runs,
so the 10° spin direction cannot be derived from the sensors alone.

**Suggestion, not yet adopted.** The camera is already being queried in case (c)
and is in W mode, so reporting the **white line's angle** is its native
question and costs nothing extra. That would replace the nudge with a
measurement, and is the strongest available argument that (c) converges without
a counter.

#### Protocol consequence

In case (d) the hub is still in **WHITE** polarity but needs the camera to fit
with the **BLACK** threshold. Today the request byte encodes the hub's polarity
(`BLK?` / `WHT?`) and the camera selects its threshold from it, so as written
this query is not expressible. The request must carry the **state or the
question** rather than the polarity flag. This fits the mode-plus-target request
format already sketched in §3 for the zone.

As in X, cases (f) and (g) must key on **staleness thresholds** rather than a
single failed read.

---

## 6. Green markers, obstacles, and the line-lost path

### Green (states L and R)

#### Transition guard: F → L / F → R

All of the following must hold:

1. The robot is in **F**. This is what skips green detection on inverted tiles,
   where no markers exist — no special case needed, the state machine gives it
   for free.
2. An inner colour sensor reads green (hue window plus saturation and value
   floors, so black, white and grey are rejected whatever their brightness).
3. **Three consecutive samples** of that sensor all read green.
4. At least *N* mm has been travelled since the last L or R completed — the
   lockout, without which the marker just turned at retriggers immediately.

The side that reaches the three-sample threshold first decides L versus R. Both
inner sensors sitting inside a 39.6 mm marker means yaw can put both on green,
so a tiebreak is required rather than optional.

> **Decision, with its trade-off recorded.** The three samples are taken
> back-to-back, so at following speed they span a fraction of a millimetre of
> travel. They therefore reject sensor noise, not a brief spatial false
> positive; a distance-spread confirmation would reject both. Accepted as
> adequate, on the basis that nothing else on an NSW Open line course is green.

**Green outranks the band test.** The marker sits immediately before the
intersection, so there is a window in which an inner sensor is on green while
both-inner-dark could also fire. If X wins that race the robot drives straight
through a turn it was told to take. The evaluation order in the F loop is a
rule, not an artefact of how the code happens to be written.

Timing budget: the marker gives ~40 mm of travel in which to notice it, over a
second at present speeds. If open question 3 forces a much faster follower, that
margin shrinks and confirmation starts competing with the detection deadline.

#### The manoeuvre

Three steps. L is described; R is the mirror image.

**Step 1 — signed straight move.** Drive a configurable distance, **negative for
backward, positive for forward**, before any rotation.

This corrects a *longitudinal* error and cannot be folded into the pivot. On a
differential drive the instantaneous centre of rotation always lies on the line
through the two wheel contact points: it can be slid anywhere along that line,
but it cannot be moved fore or aft. The sensors sit ahead of the wheels, so the
point at which green is detected is not the point to rotate about, and only a
straight move can fix that. Turning from the detection point swings the robot
about the wrong centre and loses the line.

The existing forward 5 mm plus back 26 mm net to a **21 mm reverse**, so ≈−21 mm
is the starting value now that the forward-check step is gone. Signed, because
the correct offset depends on sensor overhang and may fall either side of zero
on a rebuilt chassis.

**Step 2 — blind pivot to the minimum angle.** Rotate about a pivot point whose
**lateral** position is configurable. This is the knob that sweeps the inner
sensor off the green marker: a centre pivot barely moves the sensors, whereas a
pivot at the inner wheel swings them on a half-axle-track radius and clears it.
Generalising the old `on_wheel` boolean to a continuous lateral offset gives
finer control over where the robot ends up.

The minimum angle exists to get past the line **straight ahead**, not the line
behind. At a marked crossroads — Cross Two Shortcuts, the roundabouts — the
straight-on line is still present during the turn, and the far sensor sweeps
across it within roughly 15° of rotation. Stopping there would report "line
found" without having turned at all.

**Step 3 — hunt for the branch with the FAR sensor.** Keep turning, watching the
inner sensor on the *opposite* side to the turn (inner-right for a left turn),
until it reads black or the maximum angle is reached. Then stop.

Two reasons for the far sensor rather than the near one:

1. By the minimum angle it has already swept past the straight-ahead line, so
   the next black it sees is the branch by construction.
2. When it fires, the branch lies under the far side of the robot, so F opens
   with a large, correctly-signed error and steers straight back onto the line.

Note the near sensor is *available* — the marker is not a confounder for a black
test, since green is not black. The old "sensor stays on the green square"
problem concerned re-triggering green detection, which the lockout now handles.
The far sensor is chosen for the two reasons above, not out of necessity.

**Tuning task.** The current window is `TURN_MIN_ANGLE = 63` to
`TURN_MAX_ANGLE = 68` — only 5° in which the sensor check can fire, so in
practice this is a blind ~65° turn and the hunt is decoration. Widen it (~55–100°
as a starting point) so the hunt does real work. A fixed blind angle is the
alternative but a worse one: §2.2.4 allows lines to meet at any angle, and the
tile set includes Cross Diagonal and the hex tiles.

#### Exit

L and R transition **only to F**, whether or not the branch was found.

This is deliberate, not an oversight. A turn that ends with no line under the
robot puts F on all-bright, which is the line-lost trigger, which is **B**,
which already routes to **H**. Recovery comes for free through a path that has
to exist anyway. It is also safe from the obvious hazard: B will not mistake a
failed turn for the evacuation zone, because B's zone exits require the camera
to have seen a zone target.

**Do not add L → H or R → H.** It would duplicate a path that already works.

**OPEN — Dead End semantics.** The Dead End tiles place one marker beside each
terminating end, both on the same side, which matches no Figure 2 case. The
likely reading is "marker, then the line terminates → U-turn", folded into the
not-found return from the branch hunt in L and R. If that reading is confirmed,
a **U** state may be warranted alongside L and R, since a U-turn pivots about
the robot centre rather than a wheel — about a wheel it would finish a full axle
track to the side, off the line it has to rejoin. This is the one case where
guessing wrong means driving off the tile; confirm with the coordinator.

### Obstacle bypass (state O)

Worth +10 per obstacle, and a Lack of Progress if mishandled.

#### Transition guard: F → O

The ultrasonic reads closer than **150 mm**.

#### The manoeuvre

A fixed detour to the right, in three moves:

1. Spin **70° clockwise** on the spot, away from the line
2. Sweep **100° counterclockwise** about a centre **200 mm to the left**, which
   arcs the robot out around the obstacle and back inwards
3. Drive straight until the **inner-right** sensor reads black, or
   `OBSTACLE_REJOIN_MAX` is reached

Move 2 does the work of what was previously three separate steps — out-leg,
turn, return-leg — as a single arc.

#### Exit

- inner-right finds the line → **F**
- maximum distance reached → **H**

Unlike L and R, O does have an edge to H: a bypass that fails to rejoin leaves
the robot displaced and off-heading, which is precisely what H is for.

#### Traced geometry

| after | position | heading |
| --- | --- | --- |
| move 1 | on the line | 70° east of north |
| move 2, peak | **132 mm east** | — |
| move 2, end | 105 mm east, 288 mm north | 30° *west* of north |
| move 3 | needs ~210 mm to reach the line | |

Three limits, all satisfied:

- **132 mm clears the obstacle.** Base diagonal is up to 150 mm, so 75 mm plus
  the robot's half-width. The margin is about 57 mm — thinner than it looks, and
  `OBSTACLE_TURN_OUT` widens the arc more effectively than the pivot distance if
  the robot ever clips one.
- **132 mm is inside the 250 mm field-edge guarantee** (§2.4.5), so a fixed
  right-side detour cannot drive off the field.
- **132 mm is inside the 300 mm** within which §2.4.7 requires the line to be
  reacquired, so the +10 survives a clean detour.

#### Why inner-right in move 3

After move 2 the robot sits east of the line closing on it from the right, so
the inner-**left** sensor crosses first. That is not the sensor to stop on. The
hunt exists to leave the robot somewhere F can take over from, and at a 30°
approach the sensors' perpendicular separation is 33 × cos 30° = **28.6 mm**:

| stop on | left sensor | right sensor | robot centre vs line |
| --- | --- | --- | --- |
| inner-left, first touch | +7.9 (on line) | +36.5 | 22.2 mm east, line outside the pair |
| **inner-right, first touch** | −20.7 (clear) | +7.9 (on line) | 6.4 mm west, line straddled |

The shallower 30° approach is more forgiving than the 45° of the earlier
design: the perpendicular separation grows from 23.3 mm to 28.6 mm against a
15.8 mm line, so the line cannot slip between the sensors unnoticed.

#### Known false positives to tune out

- **Ramps.** A level ultrasonic beam at height *h* strikes a 20° ramp surface at
  *h*/tan 20° ≈ **2.75 h** ahead, so a sensor mounted at 40 mm reads 110 mm at
  the foot of a ramp — inside the 150 mm trigger, while still on flat ground.
  Mount it above **150 × tan 20° ≈ 55 mm** and the reading at the ramp foot
  exceeds the threshold; once climbing, the robot pitches with the surface and
  the beam looks up the ramp, so it self-resolves.
- **The evacuation-zone entrance.** The line runs right up to the access point,
  so F closes on a walled opening. The numbers suggest it is safe — a ~35° cone
  is about 94 mm wide at 150 mm against a 228 mm gap, and past the opening the
  far wall is 891 mm away — but only if the robot arrives reasonably centred,
  and a false O here fires immediately before the highest-value part of the run.
  Test deliberately.

#### Suggestion, not yet adopted

Move 3 crosses the line at 30°, so F inherits that heading error. Spinning 30°
to align the moment the line is detected would hand over a robot that is both
positioned and pointed correctly, at the cost of one short spin.

### Zone check (state B)

#### Transition guard: F → B

**200 mm travelled with neither inner sensor seeing black.**

Note there is no separate gap-bridging behaviour, and none is needed: with both
inner sensors on white the error is 0, so F drives straight of its own accord.
A 59.4 mm Broken Road gap is therefore bridged with no state change and a wide
margin. B fires only when the line has genuinely gone.

That leaves exactly two possibilities — the robot is lost, or it has entered the
evacuation zone — and B's whole job is to tell them apart.

#### Actions inside B

1. Raise the camera from its line-following aim to the zone-scanning tilt.
2. Put the camera into zone mode: search for silver spheres, the black sphere,
   and the green and red evacuation points.
3. Spin 360°, **polling the camera throughout**. On every reply reporting a
   target, append `(type, bearing, range)` to a sighting list held on hub 1,
   where bearing = the hub's gyro heading at that moment plus the target's
   in-frame offset.
4. Stop.
5. The sighting list is the result, and is handed to E.

#### Why hub 1 accumulates the sightings, not the camera

The camera does not know the robot's heading, and the robot keeps rotating after
each sighting — so a bearing reported at the end of the spin is meaningless on
its own. Only hub 1 can stamp a sighting with a heading. Keeping the list on
hub 1 also means the camera continues to report only "what I see right now", so
the 8-byte payload never has to carry five targets at once.

**Record every target, not just the first sphere.** The 360° spin is the only
moment in the run when the whole zone is visible from one place. Three spheres
plus two triangles is five records at negligible cost, and it buys two things:

- E can choose **nearest-first** (§7) rather than being handed whichever target
  the robot happened to be facing when the spin began.
- Both evacuation points are located up front, which is what §7's
  localise-from-the-triangles plan needs. Discarding them here would force E to
  spin again before it could deliver anything.

Two accuracy notes. The camera's answer can be a frame old, so at a fast spin
the stamped heading is off by a few degrees — acceptable, since the initial
bearing only has to bring the target back into frame for E to re-acquire
visually. And range for floor-standing objects comes from where the object's
base sits in the frame, given the camera height and tilt.

#### Exit cases

| | result | action | → |
| --- | --- | --- | --- |
| a | camera unreachable | none | **H** |
| b | black line ahead | spin by the reported angle, restore the line aim | **F** |
| c | list contains at least one sphere | none | **A**, carrying the list |
| d | list non-empty but no sphere — a triangle only | none | **V**, carrying the list |
| e | list empty | none | **H** |

Case (b) **spins by the reported angle** rather than simply handing back to F.
Without it F resumes with zero error, drives straight, and only re-acquires the
line by luck — the camera has already measured where the line is, so throwing
that away would be wasteful.

> **Case (b) can ping-pong.** If the camera sees a line but the robot still
> cannot find it with its sensors, F drives, the counter resets on the
> transition, and `LINE_LOST_MM` later B fires again — costing a tilt cycle and
> a query each time. A bound like M's three-strike counter would cap it.

Case (d) is informative rather than a fallback: the triangles are zone-unique,
so seeing one without a sphere means the robot *is* in the zone and the balls
were occluded or behind it — which is exactly when a fresh survey is right.

B's 360° spin **is** the survey, which is why case (c) can go straight to A
rather than repeating the scan.

**The camera aim** is restored by B itself on every exit that is not a zone
state — (a), (b) and (e). B is the only state that raises the camera, so B is
where it has to be put back: F needs that view immediately, and H would
otherwise reset into line following with the camera still pointed at the
horizon, a fault that survives the button press and quietly breaks every
junction for the rest of the run. Cases (c) and (d) keep it raised, since the
zone states need it for victim tracking.

H's own reset (§4) still lists the camera, which is harmless duplication — the
aim request is idempotent, and B putting back what B moved is the more
localised guarantee.

#### Tilt geometry

**Mount convention on this robot: tilt 0 = camera perpendicular to the floor**
(straight down), negative rotates it up toward horizontal. The servo clamps at
±70°, so **−70 is the most forward it can aim** — 20° below horizontal.

The zone scan uses **−60**, i.e. 30° below horizontal.

Ground intersection at the centre of the frame is `h / tan θ` with θ measured
from horizontal, but the frame spans a wide vertical arc, so the aim point is
far less critical than that figure suggests. For a camera at 150 mm with a
half-FOV of roughly 20–25°, an aim of 30° below horizontal covers ground from
about **105 mm to 1.7 m** — the whole zone, with the far corners near the top of
the frame.

> **Unresolved: the code documents the opposite zero.** Both `pantilt.py` and
> `robocup_open_line_rescue_client_openmv.py` state "tilt 0 = level, + = down",
> and the latter aims with `CAMERA_TILT = 5`. Under the mount convention above
> that would point the camera 5° off vertical, which cannot answer "is there a
> straight line ahead?" and is inconsistent with `MIN_LINE_LENGTH = 150` px.
> One of the two is stale. Every tilt angle in both programs is measured from
> this zero, so resolve it before relying on any of them. See open question 12.

#### Green is the one target that is not zone-unique

Silver spheres, the black sphere and the red evacuation point appear nowhere
else on an Open course. **Green does**: 39.6 mm intersection markers are spread
across the whole line course, and B is entered whenever the robot is lost, not
only at the zone.

The failure case is a robot that loses the line near a junction — where markers
live — spins, catches a marker, and transitions to E in the middle of the course.

**Suggestion, not yet adopted.** Gate green on apparent size. The evacuation
point is 280 mm against the marker's 39.6 mm, so a minimum-area filter removes
the case while keeping green as a target.

#### Consequences recorded

**All line-loss recovery now lands in H.** An earlier design used the last
steering error to separate a gap from a drift off a curve, and searched back
toward the drift side. That is gone: a robot that slides off a curve now drives
straight for 200 mm — away from the line — then spins, finds nothing, and hands
to H from further out than where it went wrong. Accepted — and cheap, now that
H is a human reset rather than a search: the deduction is capped at 20 and the
cost is clock time, not points (§4).

**200 mm is two-thirds of a 297 mm tile**, and §6.4.1.8 makes failing to
reacquire the line before leaving the tile a Lack of Progress. On a small-tile
course a lost robot will often be off the tile before B fires. Accepted as the
cost of a threshold long enough not to false-fire on a 59.4 mm gap.

**The 360° spin happens barely inside a 228 mm entrance.** A full rotation needs
clearance equal to the robot's diagonal and the entrance walls are close.
Retained for now; if it clips, the fallback is three body rotations of 120° with
±60° of pan, which sweeps the same circle with much less swept volume.

---

## 7. Evacuation zone (states V, A, C, D, T, K, Q)

**There is no single "evacuation zone" state.** B enters the group directly at
V or A, and K is the only exit back to F.

**Status: the sub-states below are proposed and not yet agreed**, except for the
entry points, which are settled. The design notes that follow them are settled.

### Sub-states

| | Name | Behaviour |
| --- | --- | --- |
| V | **SURVEY** | spin/pan scan; build or refresh the zone map |
| A | **APPROACH** | visual servo toward the chosen sphere |
| C | **CAPTURE** | lower, open, close, lift; verify by motor angle + camera |
| D | **DELIVER** | navigate to the matching evacuation point |
| T | **DEPOSIT** | position, release over the 60 mm wall, retreat |
| K | **EGRESS** | find the access point, leave, reacquire the line |
| Q | **RECOVER** | in-zone recovery: stuck, lost, or nothing found |

Target selection is deliberately **not** a state. Every state above owns a
distinct actuator behaviour; selection is pure logic with none. It belongs on
the way into A, which is also where the time budget and the "no targets left"
test naturally live.

### Transitions

```
from B ────┬──→ V   zone confirmed, no sphere located yet
           └──→ A   zone confirmed, a sphere located

V SURVEY ──┬──→ A   a sphere is known
           ├──→ K   none left to find, or time is short
           └──→ Q   scan failed

A APPROACH ┬──→ C   within capture range
           ├──→ V   target lost, or not where the map said
           └──→ Q   blocked

C CAPTURE ─┬──→ D   grip confirmed and classified
           ├──→ A   grip failed, ball still visible
           ├──→ V   grip failed, ball gone
           └──→ Q   hub 2 FAULT or timeout

D DELIVER ─┬──→ T   arrived at the correct triangle
           ├──→ A   ball dropped in transit, still visible
           ├──→ V   ball dropped and gone, or triangle not where expected
           └──→ Q   blocked, or hub 2 FAULT

T DEPOSIT ─┬──→ V   released — go find the next
           ├──→ D   release failed, still holding
           └──→ Q   hub 2 FAULT or timeout

K EGRESS ──┬──→ F   out of the zone, line reacquired  (+20)
           └──→ Q   cannot find the exit

Q RECOVER ─┬──→ V   resume the search
           └──→ K   give up rescuing, try to leave
```

**H is unreachable from inside the zone.** H resets the robot for a fresh start
on the *line course*, which is meaningless here — and it would abandon a victim
already under control. Q is the in-zone equivalent. Per §1's never-stop rule, Q
must loop back rather than halt.

The `→ Q` edges on C, D and T carry hub 2 faults specifically: §3 routes a
`FAULT` or a missing `DONE` to Q inside the zone and to H on the line course, so
every state that commands the claw needs somewhere to send that.

### The zone map (settled)

**Neither blind navigation nor a full re-scan between victims.** A hybrid: the
map supplies a coarse bearing, the camera does everything after.

*Why not blind.* §2.7.3.6 warns that although the evacuation points are fixed to
the floor, *"teams should be prepared for slight movements"* — the rules
explicitly say not to trust their recorded position. The triangle's recorded
**range** is weak anyway: it is ~850 mm away at survey time, and range from
base-of-object-in-frame degrades badly with distance, so the map's idea of where
it sits was poor the moment it was written. Add odometry drift across a pickup
and driving blind into a walled corner is a bad bet. T's release is close-loop
visual regardless.

*Why not a full re-scan.* ~6 s each, wanted after every delivery — 18–36 s of a
300 s budget spent rediscovering something already roughly known. And from a
corner, just after depositing, the robot sees little of the zone.

*The useful split:* **the map is not for finding the triangles, it is for
keeping track of the balls.** Triangles are easy targets — 280 mm, saturated
colour, against white walls — so a coarse turn plus visual servoing finds one
every time. The *balls* are what is hard to re-find. And the two connect: every
time the robot servos onto a triangle it gets an absolute position fix, which is
what keeps the ball coordinates trustworthy as odometry drifts. That is the
payoff for recording both triangles back in B.

| step | method |
| --- | --- |
| turn toward the triangle | map bearing, coarse |
| approach it | visual servo on the colour blob |
| position and release | visual, closed loop |
| re-localise on arrival | the triangle gives an absolute fix |
| find the next ball | map, refined visually on approach |
| ball not where the map says | bounded rotation, then V |

The map stays small — a handful of `(type, x, y)` entries in a zone frame, not
SLAM. Derive that frame from **observed** triangle bearings rather than assumed
corner coordinates: the zone's dimensions are published so using them is
probably not pre-mapping, but §3.2.3 is worded broadly and there is no need to
lean on assumed positions when they can be measured.

**The robot also perturbs its own map.** Driving across the zone can nudge the
remaining victims, and 40–50 mm balls with off-centre mass roll unpredictably
when clipped. That is map decay from a cause odometry cannot correct, and it is
why the map is a *hint* to be re-verified visually rather than ground truth.

### D and T in detail

**D — deliver.** Turn to the map bearing, servo on the triangle's colour blob,
and stop at a standoff using the ultrasonic. A zone wall is a flat solid
surface, the ideal reflector, and O cannot fire on those readings because O is
reachable only from F — so the ultrasonic is free for D to use.

**T — deposit.** Two separate questions, two different tests:

1. *Did the ball leave the claw?* Re-close the claw and read the motor angle.
   Opening tells you nothing; re-closing does. This runs on hub 2 as part of the
   `RELEASE` sequence (§3), and it is useful on failure too — if the ball never
   left, hub 2 has just re-gripped it and hub 1 can retry without a fresh
   capture.
2. *Did it land inside the triangle?* Camera, and the robot must **back off
   first** — at the release position the claw and robot body block the view.

Useful geometry: the wall is 60 mm and the ball 40–50 mm, so the ball is shorter
than the wall it must clear. Release therefore has to happen from above 60 mm,
and once the ball is in it physically cannot roll back out. The failure mode is
lateral misplacement, not bouncing out.

A mis-drop needs no new machinery: the ball is now a sphere on the zone floor,
which is exactly what V and A already handle.

### V must ignore delivered victims

A ball sitting *inside* a triangle looks like an available victim. §6.6.5 awards
points only once per element, so re-approaching and re-delivering one is pure
wasted time — potentially the rest of the run cycling a single victim.

**V must ignore any sphere whose position falls inside a known triangle's
footprint.** That is more robust than counting deliveries, because a failed
delivery makes the count wrong in exactly the situation where it matters most.

It also contains the post-exit case: after K → F and the +20 bonus, the robot
carries on line-following per §1's never-stop rule, and the line it is on leads
back toward the zone. Without this rule, re-entering would restart the rescue
cycle on victims already scored.

### Decisions to make

**Never refuse to place a victim.** Scoring is 10 control + 15 rescue + 15
correct point, so a victim placed in the *wrong* triangle still earns **25 of
40** while one you decline to place earns nothing. If classification is
ambiguous, guess — and guess green, since two of the three victims are live.

**The claw needs reach past the robot's nose.** The triangle has 60 mm walls and
a hollow centre and the robot's body cannot enter, so the claw must overhang far
enough to release *inside* the triangle while the robot sits outside the wall.
Measure this before designing T's approach geometry; it may decide whether the
hypotenuse is approached square-on or at an angle.

**When does the time budget force K?** The exit is worth 20 and needs enough
clock left to leave, reacquire the line and complete a full tile; a victim is
worth 40. So the rule is not "exit at T minus X" but "start no new victim you
cannot finish, and always reserve the exit budget". Derive the reserve once a
full pickup-and-deliver cycle has been timed.

### Design notes (settled)

- **Localise from the two triangles, not from odometry.** Both evacuation
  points are in the corners of the wall opposite the entrance, so on entry both
  are ahead of you, roughly 850 mm out. Two known-size coloured landmarks at
  known positions in a known-size rectangle give a full coordinate frame from
  perception alone — no drift over three round trips, and no conflict with
  §3.2.3 since nothing is pre-programmed.
- **Scan with the pan/tilt, not the drive base.** Sweeping the camera from a
  standstill costs no drive time and accumulates no odometry error.
- **Detect balls by shape, not colour.** Silver spheres are specular: they
  mirror the ceiling lights, the walls and the robot, so their apparent colour
  is a property of the venue rather than the ball. Circle detection on edges
  finds both ball types, and a 40–50 mm ball at known camera geometry gives a
  strong expected-radius prior.
- **Classification is the camera's job, and it is a black test, not a silver
  test.** There is no gripper sensor. The type is already recorded per target in
  B's sighting list and refined throughout A as the ball grows in frame, so no
  new mechanism is needed. And because there is **one black victim and two
  silver ones**, the camera never has to positively identify a specular
  surface — a dark sphere is the dead victim and everything else spherical is
  live. A dark sphere on a white floor is the highest-contrast target on the
  field, and all the difficulty with silver becomes irrelevant because silver is
  the residual class.
- **Verify the catch two ways, neither needing a sensor.** Claw **motor angle**
  at stall gives tightness: fully closed means nothing caught, a gap consistent
  with 40–50 mm means a ball, stopping early means something larger. A brief
  camera look at the claw gives presence. They fail differently, so together
  they are strong — and the camera look is also the last chance to catch having
  approached one ball and captured the one beside it, which is otherwise
  undetectable and costs 15 points.
- **Carry height is a compromise.** The claw must be visible to the camera for
  that check but clear of the frame during D, when the camera is servoing on a
  triangle across the zone. Likely: lift to a carry height out of frame, tilting
  down briefly for the check before setting off. This makes open question 6
  load-bearing.
- **Nearest-first, not by type.** All three victims are worth 40, so there is no
  sequencing advantage, and nearest-first maximises the count if time runs out.
- **The last 100 mm is the risk.** The ball leaves the camera's field of view as
  you close. Mitigate by tilting down to track it in, and by stopping visual
  servoing at the last range where it is still visible and driving the final
  short leg open-loop.
- **Terminate gracefully on fewer than three victims.** A Lack of Progress after
  a successful placement restarts the robot from City Limits, but placed victims
  stay placed and their points stand — so a second pass may find only one or two
  balls. Searching until the clock expires would forfeit the exit bonus too.

- **Only silver and red are zone-unique.** Green markers are spread across the
  line course, and black is the colour of every line on the field. What keeps
  the black sphere safe as a target is shape-based detection — a circle detector
  does not fire on a line. The state machine depends on that, so it is not
  merely a robustness preference.

**OPEN:** search pattern, approach control law, deposit manoeuvre, exit
strategy, and the time budget that decides how much searching is affordable.

**Hardest step: A → C.** The ball leaves the camera's view as the robot
closes, its centre of mass is deliberately off-centre so it rolls unpredictably
if nudged, and the claw's capture envelope is still unmeasured (open question
1). Everything else in the zone is navigation between known points; this is the
only step requiring precision about something that can move.

---

## 8. Build order

1. **Hub 1 ↔ hub 2 state protocol.** Everything in the zone depends on it, and
   it is testable on the bench with no vision at all.
2. **Gap bridging (B) and zone-entry detection.** Earns Logic Pool 2 tile points
   on its own and is the only way into the zone.
3. **Four-sensor junction table**, and the simplifications in §5–6.
4. **Evacuation zone states (V, A, C, D, T, K, Q).**
5. **Obstacle bypass (O).** +10 and independent of everything else.

---

## 9. Open questions

| # | Question | Blocks |
| --- | --- | --- |
| 1 | Claw capture envelope — lateral tolerance and depth tolerance | vision accuracy spec, §7 |
| 2 | Dead End semantics: is "marker then line terminates" a U-turn? | §6 |
| 3 | Measured end-to-end course time against the 300 s budget | whether the step-based follower needs replacing with a continuous PD loop |
| 4 | BLE round-trip and loss rate under full load | §3, how much the outer pair can be relied on |
| 5 | Is the row-4 sensor pattern (both inner dark, both outer bright) ever observed? | §5 |
| 6 | Does the claw occlude the camera at close range? | §7 approach design |
| 7 | Branch-hunt window: widen from the current 5° and re-tune min/max | §6, whether the hunt works at all |
| 8 | Ultrasonic mounting height against the ≈55 mm ramp threshold | §6, false O on ramps |
| 9 | Confirm the O arc clears a real obstacle — 132 mm of lateral excursion leaves only ~57 mm of margin | §6, whether the +10 is reachable |
| 10 | Do M case (c) nudges converge without an attempt counter? | §5, risk of an unbounded spin loop in W |
| 11 | Camera request format must carry the state/question, not the hub polarity | §5 M case (d), §3 zone protocol |
| 12 | Reconcile the tilt zero: mount says 0 = straight down, both programs' comments say 0 = level | §6 B, and every tilt angle in both camera programs |
| 13 | No global time supervisor exists — §7 poses "when does the clock force K" but nothing tracks it at run level | §7, and the 20-point exit bonus |
| 14 | S, V, A, K and Q have transitions but no specified actions | implementation |
| 15 | Ramps and the see-saw: §4 calls gyro pitch "a modifier" but nothing implements it | Physical Pool 1, +5 per bridge/see-saw |
| 16 | F → B fires while following correctly — pick a fix (see the correction in §4) | live in code; a false B on any long straight |
| 17 | Measure the claw angle window: empty / 40–50 mm sphere / larger | the entire grip verdict |
| 18 | Measure the claw grip torque ceiling — currently a guess | crushing or ejecting a victim |
| 19 | Measure the lifter positions; `LIFTER_RELEASE` must clear the 60 mm wall | T cannot deposit otherwise |

---

## 10. Implementation

| file | holds |
| --- | --- |
| `pybricks/robocup_olr_hub1.py` | the state machine of §4–§7; states S, F, W and H implemented, the rest are transition-only |
| `pybricks/robocup_olr_hub2.py` | the §3 peripheral loop, complete |

One architectural constraint discovered while writing hub 1, which constrains
every state added later:

**Nothing may call `robot.reset()`.** The green lockout and the line-lost
counter both store an *absolute* odometry reading and subtract from it, so a
reset would move the reference underneath them — the lockout would never expire
and `line_lost_mm()` would go sharply negative. States needing a local zero must
record `odometry_mm()` on entry and subtract. `pivot_turn()` already does this
with `robot.angle()`.

---

## 11. Superseded

`pybricks/robocup_open_line_rescue_robot_spike.py` and
`openmv/robocup_open_line_rescue_openmv.py` are a complete parallel
camera-primary implementation using the `arrow` protocol. The sensor-primary
stack replaced it and the two have diverged. Retire the old pair rather than
maintaining both.
