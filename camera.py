"""
RoboCup Open Line Rescue - OpenMV camera (H7 Plus, firmware v5)

The camera is a peripheral. It never decides anything and it cannot push data:
PUPRemote is a shared variable, not a request-response link, so hub 1 writes a
request and reads back WHATEVER THE CAMERA LAST PUT ON THE WIRE - not a value
computed in response to that particular call.

So the loop computes an answer every frame and leaves it ready. A poll is then a
read rather than a round trip.

WHAT THE HUB ASKS
-----------------
One question at a time, named in the request. The camera runs only the detector
that question needs, which is what keeps the frame budget predictable: with
get_regression() being O(N^2) Theil-Sen and find_circles() a Hough transform,
running every detector every frame would collapse the frame rate.

  'J'  junction (hub state X)  - is a straight BLACK line CONTINUING ahead, and
                                 how much of the view is BLACK?
  'M'  seam (hub state M)      - is a straight BLACK line ahead, at what angle,
                                 and how much of the view is WHITE?
  'A'  ahead (hub state B)     - is there a BLACK line ANYWHERE ahead, at what
                                 angle? Asked from a raised aim.
  'Z'  zone (hub states B, V,   - what evacuation-zone target is nearest, of
       A, D)                     what kind, at what bearing and range? The
                                 request's third byte narrows it to one kind,
                                 or 0 for any.

'J' and 'A' differ by more than the aim. 'J' requires the fitted line to start
at the bottom centre of the frame, because it answers "does the line continue
past THIS junction" - continuous with where the robot already is. 'A' is asked
when the robot may have drifted off the line entirely, so that test is dropped
and the length requirement relaxed: a distant line is shorter in frame.

THE AIM FOLLOWS THE QUESTION
----------------------------
Each question implies where to point, so the hub never sends tilt degrees. It
asks 'A' and the camera raises itself; it asks 'J' again and the camera drops
back. One less protocol field, and the pan/tilt stays owned by the side that
will later need to close a tracking loop around it.

A tilt change does NOT block - pyb.Servo.angle() sets up an interpolation and
returns - so the camera keeps grabbing frames while the servo travels. It would
happily fit a line in a smeared image and publish it with a matching echo. So
while settling it withholds the sequence byte: the echo never matches, and the
hub simply keeps polling until the view is steady.

Both fit a BLACK line: X asks because the robot follows black, and M asks
because the line it is about to rejoin is black even though the robot is still
in white-line mode. That is why the request carries the QUESTION and not the
hub's current polarity - see DESIGN.md open question 11.

PROTOCOL
--------
PUPRemote command 'look':

    from_hub = "4s"   -> b"<question><seq><pad><pad>"
                         question : one of the letters above
                         seq      : one raw byte, incremented by the hub on
                                    every new question

    from_hub byte 2 is a KIND filter, used by 'Z' only and ignored otherwise.

    to_hub   = "hhhh"

      line questions 'J' 'M' 'A':
                      (echo, line_ahead, angle, coverage)
                         line_ahead : 1 if a black line was accepted
                         angle      : degrees from straight ahead, + = right
                         coverage   : % of the view matching the question's
                                      threshold - BLACK for 'J' and 'A', WHITE
                                      for 'M'

      zone question 'Z':
                      (echo, kind, bearing, range_mm)
                         kind       : KIND_* below, 0 when nothing was found
                         bearing    : degrees from straight ahead, + = right
                         range_mm   : estimated ground distance

The payload is exactly 8 bytes, a power of two, because a non-power-of-two size
makes an invalid LPF2 frame and crashes the sensor.

`echo` is what makes a stale answer detectable rather than merely suspected: the
hub knows its question has been answered when echo == the seq it sent. Without
it the only defence is discarding a frame and hoping, which is what the previous
client had to do.

COVERAGE IS RAW, THE HUB DECIDES
--------------------------------
The camera reports a percentage; it does not judge whether that means the
background has flipped. Hub 1 owns the polarity and the thresholds, exactly as
it does for hub 2's sensor readings.

HOW THE ZONE DETECTOR WORKS
---------------------------
The evacuation zone is an almost empty white box: white floor, white walls, and
nothing in it but three balls, two triangles and the robot. So the detector does
not identify anything - it finds WHAT IS NOT FLOOR and sorts a handful of
candidates.

The two triangles are found by COLOUR, the three balls by SHAPE, and the split
is not an optimisation - it is forced.

Green and red are real colours and are unique inside the zone, so one
find_blobs() pass over two windows finds the evacuation points and nothing else
competes with them.

Silver is not a colour. A mirror reports the venue, not the object, so a silver
sphere's LAB values belong to the ceiling lights, the walls and the robot. A
colour-window detector for it was built and failed structurally, not by
mistuning: the ball's bright facets blow past any ceiling low enough to still
exclude the white floor, and what is left inside a neutral mid-tone window is a
web of thin crevices, no one of them a large enough connected component. It
produced NO BLOB AT ALL, so no shape filter could have rescued it.

What is stable about a ball is its OUTLINE. A sphere projects to a circle from
any viewing angle, whatever it happens to be reflecting, so find_circles() is
the primary sphere detector here - run over the frame in two range bands, not as
a confirmation step. Only after a circle is found is LAB consulted, and then as
STATISTICS over the circle's middle:

    l_mean low, l_stdev low      the black victim
    l_mean high, l_stdev HIGH    a silver victim - bright facets, dark crevices
    l_mean high, l_stdev low     bare floor, reject

Variance is the silver signature, and it is exactly what a connected-component
detector could not see. Radius and image row are rigidly coupled for a ball
resting on the floor, which gives a second, free rejection test: a circle whose
size disagrees with its contact row is not a ball on this floor.

STATUS
------
Implemented: the line questions 'J', 'M', 'A', and the zone question 'Z'.
Stubbed:     the access-point strip detector and pan/tilt target tracking.
"""

import math
import sensor
import time
from pantilt import PanTilt
from pupremote import PUPRemoteSensor

# ============================================================================
# CONFIGURATION
# ============================================================================

# ---- Camera orientation ------------------------------------------------------
# Both on to undo a camera mounted upside-down.
CAMERA_VFLIP = True
CAMERA_HMIRROR = True
# Which edge of the frame is nearest the robot; follows the vertical flip. Get
# this wrong and "starts at the bottom centre" tests the far end instead.
NEAR_FIELD_AT_BOTTOM = CAMERA_VFLIP


# ---- Zone: geometry for range estimation -------------------------------------
# Deliberately expressed as MEASURED VIEW ANGLES rather than servo positions, so
# none of it depends on where the servo's zero is (open question 12). Point the
# camera at marks on the floor and read these off the video:
#
#   CAMERA_HEIGHT_MM        lens height above the floor
#   ZONE_CENTRE_BELOW_DEG   angle below horizontal that the CENTRE row of the
#                           frame looks at, when in the zone aim
#   VERTICAL_FOV_DEG        top row to bottom row
#
# Range then follows from the tangent relation for a point on the floor. If that
# proves fiddly, a two-point calibration works as well: note the pixel row of an
# object at two known distances and interpolate.
CAMERA_HEIGHT_MM = 140
# 31, not the 25 this started at. FITTED, not measured: the three balls in a
# standalone frame all read systematically larger than a 25 deg model predicted,
# and the error GREW WITH DISTANCE - the signature of a wrong depression angle
# rather than a wrong ball size. One value reconciles all three to within 0.6 px
# of radius, which a coincidence would not.
#
#     row 200 r 42   predicted 31.9 at 25 deg   41.8 at 31 deg
#     row  78 r 25   predicted 15.9 at 25 deg   24.3 at 31 deg
#     row 100 r 27   predicted 19.0 at 25 deg   27.8 at 31 deg
#
# This is open question 12 - the tilt zero - showing up as data. Confirm against
# a tape measure before trusting the RANGES this produces, because _range_of()
# feeds the whole zone state machine, not just the detector.
ZONE_CENTRE_BELOW_DEG = 25
VERTICAL_FOV_DEG = 30
HORIZONTAL_FOV_DEG = 52

# Anything at or beyond this is reported as "not found": the zone is only
# 1188 x 891 mm, so a larger estimate means the range model has broken down
# rather than that a target is genuinely far away.
ZONE_MAX_RANGE_MM = 1600

# ---- Pan/tilt aim ------------------------------------------------------------
# TILT ZERO IS UNRESOLVED - DESIGN.md open question 12.
#
# pantilt.py and the previous client both document "0 = level, + = down", but
# the mount on this robot puts 0 at PERPENDICULAR DOWN, with negative rotating
# up towards the horizon. Every angle here is measured from that zero, so
# confirm it before trusting any of them. The servo clamps at +-70 either way.
#
# For line work the camera must see well down the line, not at its own wheels:
# the acceptance tests below assume a view reaching into the far field.
CAMERA_TILT_LINE = 5        # aim for the line questions, 'J' and 'M'
CAMERA_PAN = 8              # corrects a camera not quite on the robot centreline

# Aim for the 'A' question, expressed as a DELTA from the line aim rather than
# an absolute angle. The two candidate zero conventions disagree about where 0
# points, but both agree that a lower number points higher - so a relative
# offset states the intent ("look further ahead") without depending on which is
# right. Calibrate the magnitude against the video.
CAMERA_TILT_AHEAD = CAMERA_TILT_LINE - 15

# Aim for the zone question. Targets lie on the floor anywhere from just in
# front of the robot to the far wall 891 mm away, so this wants a wider, longer
# view than line work - the same direction as the 'A' aim.
CAMERA_TILT_ZONE = ZONE_CENTRE_BELOW_DEG - 90

# How long the servo is given to travel and settle before answers are trusted.
# pantilt interpolates over TILT_MOVE_MS and returns immediately, so nothing
# else enforces this.
TILT_MOVE_MS = 500
TILT_SETTLE_MS = 200

# ---- Region of interest ------------------------------------------------------
# A tall narrow strip centred on the robot's path.
#
# Narrow keeps a crossing's perpendicular arm out of the fit, so the regression
# follows the line running away from the robot. Wide tolerates the line sitting
# off-centre without falling outside the strip - but lets more of the arm in,
# which drags the fitted angle towards horizontal and can make a real crossing
# fail MAX_LINE_ANGLE. If crossings get rejected with an angle complaint, narrow
# this before widening the angle limit.
FRAME_WIDTH = 320           # QVGA
ROI_WIDTH = 150
ROI_Y_MIN = 0
ROI_Y_MAX = 200             # keeps the horizon and the robot body out of the fit

ROI_X_MIN = (FRAME_WIDTH - ROI_WIDTH) // 2
ROI_HEIGHT = ROI_Y_MAX - ROI_Y_MIN
ROI = (ROI_X_MIN, ROI_Y_MIN, ROI_WIDTH, ROI_HEIGHT)
ROI_CENTER_X = ROI_X_MIN + ROI_WIDTH // 2
NEAR_EDGE_Y = ROI_Y_MAX if NEAR_FIELD_AT_BOTTOM else ROI_Y_MIN

# ---- Colour thresholds -------------------------------------------------------
# LAB (Lmin, Lmax, Amin, Amax, Bmin, Bmax) on the colour image. Both are
# near-neutral in A and B as well as dark or bright, so a green marker or a
# green evacuation point is rejected rather than fitted as a line - green has a
# strongly negative A and falls outside either window whatever its brightness.
BLACK_THRESHOLD = [(0, 30, -18, 18, -18, 18)]
WHITE_THRESHOLD = [(70, 100, -18, 18, -18, 18)]

# ---- Coverage ----------------------------------------------------------------
# A wide band used to answer "how much of the view is this colour?". On a normal
# tile the black coverage is just the line itself, a few percent; when the
# background inverts it becomes most of the frame. The hub compares the number
# against its own threshold and decides.
COVER_ROI = (10, 0, 300, 200)
COVER_MIN_PIXELS = 200      # ignore specks when summing
COVER_AREA = COVER_ROI[2] * COVER_ROI[3]

# ---- Acceptance tests --------------------------------------------------------
# A fit must pass all of these to count as "a line continues ahead".
MIN_LINE_LENGTH = 150       # px; the test that really separates cross from corner
MAX_LINE_ANGLE = 45         # deg from straight up
NEAR_X_TOLERANCE = 60       # px the near end may sit off the ROI centre line
NEAR_Y_TOLERANCE = 40       # px the near end may sit off the near edge

# Relaxed limits for 'A'. The near-end tests are dropped entirely rather than
# widened - the whole point of the question is that the robot may no longer be
# on the line - and the length falls because a line seen from further away
# occupies fewer pixels.
AHEAD_MIN_LINE_LENGTH = 60
AHEAD_MAX_LINE_ANGLE = 60

# ---- Linear regression -------------------------------------------------------
# get_regression is Theil-Sen: it compares EVERY PAIR of sampled pixels, so cost
# is O(N^2) in the sample count. Strides of 2 quarter N and cut the work ~16x,
# which matters because this runs every frame.
REG_X_STRIDE = 2
REG_Y_STRIDE = 2
REG_MIN_PIXELS = 60
REG_MIN_AREA = 60
REG_TARGET_SIZE = (80, 60)  # firmware v5 area-scales the ROI to this before fitting
# Length of the median pixel-to-pixel delta: how much the pixels agree on one
# direction. Below this there is no usable line.
REG_MIN_MAGNITUDE = 3

# ---- Zone: colour windows ----------------------------------------------------
# LAB (Lmin, Lmax, Amin, Amax, Bmin, Bmax). ALL THREE ARE GUESSES - calibrate
# against real victims and real evacuation points under venue lighting before
# trusting any of them.
#
# The two point windows are strongly coloured, the sphere window strongly
# NEUTRAL. That is what keeps them apart in a single multi-threshold pass: a
# green triangle can never match the sphere window whatever its brightness,
# because green sits far off neutral in A.
#
# The L floors are LOW on purpose. A 60 mm wall casts shadow on itself, and a
# point standing in its own shade is still the target - excluding anything
# darker than mid-tone loses it exactly when the lighting is least kind.
ZONE_GREEN_THRESHOLD = (10, 70, -60, -15, 5, 45)
ZONE_RED_THRESHOLD = (15, 40, 25, 70, 10, 55)

# FALLBACK ONLY, and only for the black victim.
#
# Spheres are found by find_circles() now. This window stays wired behind the
# _circles_ok latch so a firmware that will not run the Hough transform still
# finds the dead victim rather than nothing - the black ball on a white floor is
# the easiest target on the field, and losing it to an API change would be
# absurd. It is a FALLBACK, never a parallel detector: two detectors for one
# object class means two ways to disagree and twice the tuning surface.
#
# There is deliberately no light counterpart. One was tried, and never produced
# a single blob on a silver ball - see the header for why that was structural.
ZONE_SPHERE_DARK_THRESHOLD = (0, 15, -15, 15, -15, 15)

# Order matters: blob.code() is a bitmask over this list, and a blob matching
# more than one is resolved by taking the FIRST match, so the specific colours
# must come before the neutral one.
ZONE_THRESHOLDS = [ZONE_GREEN_THRESHOLD, ZONE_RED_THRESHOLD,
                   ZONE_SPHERE_DARK_THRESHOLD]

# ---- Zone: shape filters -----------------------------------------------------
# A 45 mm ball subtends about 28 px at 500 mm and 15 px at 900 mm on a QVGA
# frame, so area alone separates spheres from the 280 mm triangles long before
# any shape test runs.
SPHERE_MIN_PIXELS = 600
# Generous on purpose. A 45 mm ball is ~105 px across at 150 mm, about 8700 px
# of area, so a tight cap rejects the ball exactly during the final approach -
# the moment the claw most needs to see it. The upper limit is not what keeps
# the triangles out of the sphere class either: the colour windows do that and
# are tested first, and aspect and density handle large non-round things.
SPHERE_MAX_PIXELS = 8000
# A circle has width ~ height and fills about pi/4 = 0.79 of its bounding box.
# A shadow is irregular, a floor seam elongated.
#
# These now carry more weight than they used to. Shadow no longer contaminates
# the DARK class, but it does land squarely in the LIGHT one, so shape is the
# only thing left standing between a cast shadow and a reported silver victim.
# Tighten them if STANDALONE starts drawing yellow circles on the floor.
SPHERE_ASPECT_TOL = 0.35        # |w/h - 1| must be under this
SPHERE_MIN_DENSITY = 0.35       # pixels / (w * h)

# Size is the only shape test the evacuation points get, and it does the one job
# that matters: rejecting a 39.6 mm intersection MARKER glimpsed through the
# entrance, which is green too. That is a size difference, not a shape one.
POINT_MIN_PIXELS = 800

# EFFECTIVELY OFF, and deliberately kept rather than deleted so the reasoning
# stays visible.
#
# This was a hollowness test: the points are three walls 60 mm tall with an open
# centre, so from above the camera sees a coloured OUTLINE and a low fill ratio.
# That is true from above and false from where this camera actually sits - at a
# low oblique angle the NEAR WALL occludes the interior and the point presents as
# a solid coloured bar, density near 1.0. The test would therefore reject the
# genuine target at exactly the viewing angle the robot uses.
#
# It was also guarding against nothing. Inside the zone there is no other green
# or red object, which is precisely why DESIGN.md treats red as the highest-
# confidence signal that the robot is in the zone at all - so colour alone is
# already unique and shape has nothing left to disambiguate.
POINT_MAX_DENSITY = 1.0

# ---- Zone: exposure ----------------------------------------------------------
# The two personalities want DIFFERENT exposures, and this is not a preference.
#
# The silver victims are pressed, scrunched foil - dozens of small facets each
# catching light at a different angle, not one clean highlight and rim. That
# texture is what the detector below keys on, and it only survives at a BRIGHTER
# exposure than auto picks. A darker exposure was tried first, on the assumption
# that a specular ball would blow out to flat white; it is the wrong model for
# this surface.
#
# Auto gain and auto white balance are LOCKED for zone work. They were left on
# for line following, where the thresholds are coarse; here every measurement is
# absolute and a camera that keeps re-deciding what mid-grey means is measuring
# against a moving origin.
ZONE_BRIGHTNESS_FRACTION = 1.3
EXPOSURE_SETTLE_MS = 300        # frames to discard after a mode switch

# ---- Zone: spheres, in two passes --------------------------------------------
# Pass 1 is Hough circles and finds the BLACK victim, whose outline against a
# white floor is the strongest edge on the field. Pass 2 is a texture filter and
# finds the SILVER victims, which pass 1 does not catch at any threshold.
#
# That two-pass shape is a finding, not a preference. Lowering the Hough
# threshold to chase the silver balls does not work: threshold is an
# EDGE-STRENGTH measure, and a foil ball's boundary edge is genuinely weak
# against a white floor. Pushing it down to 1200 added no silver sensitivity and
# only made the black ball's already-strong edge fragment into several
# overlapping duplicate circles. 2500 is the value at which the black ball comes
# back as one sharply-defined circle - do not retune it to hunt silver.
CIRCLE_THRESHOLD = 2500
SPHERE_R_MIN_PX = 10            # below this, shadows and creases at wall edges
                                # start qualifying as circles
SPHERE_R_MAX_PX = 60
CIRCLE_R_STEP = 2               # radius resolution of the accumulator
CIRCLE_STRIDE = 2               # row/column stride of the edge scan

# ---- Zone: dead victim or live one -------------------------------------------
# Brightness only, deliberately never a test for "silver". There is one black
# victim and two silver, so dark-versus-not settles it, and silver's apparent
# colour belongs to whatever it is reflecting.
DARK_SPHERE_LUMA_MAX = 90       # 0-255 luma; at or under this a sample is dark

# A real dead ball is dark across its WHOLE face. A wall-edge shadow that
# happens to fit a circle usually is not - it is a soft gradient or a thin
# crease, dark in places. Averaging a few samples cannot separate those;
# counting what fraction of a grid is actually dark can.
DEAD_BALL_MIN_DARK_FRACTION = 0.8

# ---- Zone: the texture pass --------------------------------------------------
# What a foil ball has that the floor does not is local VARIANCE. A flat white
# background reads near-uniform at any exposure; the foil's micro-facets make
# brightness bounce around inside any small neighbourhood.
#
# image.stdev() does not exist on this firmware, so the variance is built from
# filters that do: blur a grayscale copy with mean(), then difference() it
# against the unblurred original. What survives the subtraction IS the local
# high-frequency detail - flat background cancels to near zero, texture leaves a
# residue. Runs on a separate copy so nothing touches the displayed frame.
TEXTURE_WINDOW = 2              # mean() half-window, (2n+1) square
TEXTURE_MIN = 12                # residue counted as textured, 0-255
# Upper bound: a specular glint off the floor is a near-maximum spike in the
# difference map, far brighter than the foil's internal facet variation.
# Excluding the top of the range keeps moderate texture and drops blown glints.
TEXTURE_MAX = 180
TEXTURE_MIN_PIXELS = 60
# Uneven lighting can leave only PART of a ball above TEXTURE_MIN in any one
# frame - a scattered island rather than one region. Dilating before blobbing
# grows and merges those islands into a single connected patch.
TEXTURE_DILATE = 2
TEXTURE_ROUNDNESS_TOL = 0.35    # |w - h| / max(w, h)
# A SMOOTH object's edge produces a bright RING in the difference map: one
# strong discontinuity at its outline, with a flat face inside. A genuinely
# textured surface lights up across its whole face. Both can pass a
# bounding-box roundness test, so fill is what tells a disc from a ring -
# pixels / (w * h) is low for a ring, high for something filled in.
TEXTURE_MIN_FILL = 0.5

# ---- Questions ---------------------------------------------------------------
QUESTION_JUNCTION = ord('J')    # black line CONTINUING ahead? how much BLACK?
QUESTION_SEAM = ord('M')        # black line continuing ahead? how much WHITE?
QUESTION_AHEAD = ord('A')       # black line ANYWHERE ahead? raised aim
QUESTION_ZONE = ord('Z')        # nearest evacuation-zone target

QUESTIONS = (QUESTION_JUNCTION, QUESTION_SEAM, QUESTION_AHEAD, QUESTION_ZONE)

# What the camera can report for 'Z'. 0 means nothing found, and doubles as the
# "any kind" value in the request's filter byte.
KIND_NONE = 0
KIND_SPHERE_DARK = 1            # the black victim
KIND_SPHERE_LIGHT = 2           # a silver victim
KIND_POINT_GREEN = 3            # evacuation point for the living
KIND_POINT_RED = 4              # evacuation point for the dead

# Which threshold index in ZONE_THRESHOLDS produces which kind. Every index has
# one now: the brightness call that used to be a second get_statistics() pass
# over the blob is carried by WHICH WINDOW MATCHED, so classification and
# segmentation are the same decision instead of two that can disagree.
ZONE_CODE_KIND = {0: KIND_POINT_GREEN, 1: KIND_POINT_RED,
                  2: KIND_SPHERE_DARK}

POINT_KINDS = (KIND_POINT_GREEN, KIND_POINT_RED)

# ---- Standalone debug mode ---------------------------------------------------
# Run the camera on its own, with no hub attached, to check detection by hand:
# hold it over a line, a marker, a victim or an evacuation point and watch the
# IDE frame buffer and the console.
#
# With this on the camera picks its own question instead of waiting to be asked,
# skips the PUPRemote link entirely, and prints EVERY candidate blob with the
# reason it was accepted or rejected - which is the part that actually tells you
# why something is not being seen. The chosen-target line alone cannot.
#
# Leave it False for a real run: it disables comms, so the hub would find no
# camera at all.
STANDALONE = False

# Which question to hold in standalone. The aim follows it exactly as it would
# from a hub request, so this also exercises the tilt change.
#   QUESTION_JUNCTION  line continuing ahead, plus black coverage
#   QUESTION_SEAM      line ahead, plus white coverage
#   QUESTION_AHEAD     line anywhere ahead, raised aim
#   QUESTION_ZONE      nearest zone target
STANDALONE_QUESTION = QUESTION_ZONE

# Kind filter for QUESTION_ZONE. KIND_NONE reports whatever is nearest; set a
# KIND_* to hunt one target type while tuning its threshold.
STANDALONE_KIND = KIND_NONE

# Frames between console reports in standalone. Lower than the normal rate
# because there is no hub to slow down and the whole point is watching values.
STANDALONE_PRINT_EVERY = 10

# Where each question wants the camera pointed.
QUESTION_TILT = {
    QUESTION_JUNCTION: CAMERA_TILT_LINE,
    QUESTION_SEAM: CAMERA_TILT_LINE,
    QUESTION_AHEAD: CAMERA_TILT_AHEAD,
    QUESTION_ZONE: CAMERA_TILT_ZONE,
}

# STANDALONE ONLY: where zone work points while it is being tuned by hand.
#
# ZONE_CENTRE_BELOW_DEG is what _range_of() ASSUMES the centre row looks down
# at, so aiming there makes the physical camera agree with the range model
# instead of merely being near it - which is the whole point when the numbers
# on screen are what is being judged.
#
# Expressed as a DELTA from the line aim, like CAMERA_TILT_AHEAD, because the
# two candidate tilt-zero conventions disagree about where 0 points but both
# agree a lower number points higher. "Up by ZONE_CENTRE_BELOW_DEG from the line
# aim" therefore states the intent without depending on which is right.
#
# NOTE this deliberately DIVERGES from CAMERA_TILT_ZONE, which is what a real
# hub-driven run uses. See _tilt_for().
STANDALONE_ZONE_TILT = ZONE_CENTRE_BELOW_DEG - 90

# ---- PUPRemote ---------------------------------------------------------------
# Set False to run standalone: detection and video only, no hub.
ENABLE_COMMS = True
# Milliseconds spent servicing the link after each frame. The hub polls when it
# wants an answer and process() is what replies, so this has to run often enough
# that the LPF2 link does not time out.
COMMS_PUMP_MS = 25

PRINT_EVERY = 30            # status line every N frames, 0 = never

GREEN = (0, 255, 0)
RED = (255, 0, 0)
YELLOW = (255, 255, 0)
BLUE = (0, 0, 255)


# ============================================================================
# CAMERA SETUP
# ============================================================================

# The exposure auto-exposure settled on at startup, and which personality the
# sensor is currently configured for.
_auto_exposure_us = 0
_exposure_mode = None


def _apply_exposure(question):
    """Configure the sensor for the question's personality. True if it changed.

    Line work and zone work want different sensors, not just different
    detectors. Zone work needs a BRIGHTER frame, because the silver victims are
    found by surface texture and that texture disappears at the exposure auto
    picks for a mostly-white scene - and it needs gain and white balance LOCKED,
    because every zone measurement is absolute.

    Switching is cheap but not instant, so the caller extends the settle window.
    """
    global _exposure_mode

    mode = 'zone' if question == QUESTION_ZONE else 'line'
    if mode == _exposure_mode:
        return False
    _exposure_mode = mode

    if mode == 'zone':
        sensor.set_auto_gain(False)
        sensor.set_auto_whitebal(False)
        sensor.set_auto_exposure(
            False, exposure_us=int(_auto_exposure_us * ZONE_BRIGHTNESS_FRACTION))
    else:
        sensor.set_auto_exposure(True)
        sensor.set_auto_gain(True)
        sensor.set_auto_whitebal(True)

    print("exposure -> %s" % mode)
    return True


def setup_camera():
    """Colour QVGA frames, with exposure settled before anything is measured."""
    global _auto_exposure_us

    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.set_vflip(CAMERA_VFLIP)
    sensor.set_hmirror(CAMERA_HMIRROR)
    sensor.skip_frames(time=2000)
    # Auto gain and white balance stay ON for LINE work, as in the previous
    # client, and are locked only for zone work - see _apply_exposure(). The
    # known risk is unchanged: drive onto a black tile and auto-gain brightens
    # to compensate, shifting every LAB threshold exactly when the polarity flip
    # is being detected. Measure before locking it here too.
    sensor.set_auto_gain(True)
    sensor.set_auto_whitebal(True)

    # Captured AFTER settling, so it is the exposure auto actually chose for
    # this venue. Every zone frame is a multiple of it rather than an absolute
    # microsecond count, which is what lets one constant travel between a lit
    # hall and a dim practice room.
    _auto_exposure_us = sensor.get_exposure_us()
    print("Camera ready. near field=%s ROI=%s"
          % ("bottom" if NEAR_FIELD_AT_BOTTOM else "top", str(ROI)))


# ============================================================================
# FIRMWARE SHIMS
# ============================================================================
# Carried over from the previous client. These exist because firmware v5.0.0
# changed both APIs; do not remove them without checking the version in use.

def _attr(obj, name):
    """Read a line/blob attribute across versions: older firmware exposes x1()
    and magnitude() as methods, v5.x as plain properties."""
    value = getattr(obj, name)
    return value() if callable(value) else value


# get_regression's keywords changed in v5.0.0: `robust` is gone and
# `target_size` was added, and an unknown keyword is now a hard TypeError.
# Probe the v5 form once, then latch.
_reg_target_size_ok = True


def _get_regression(img, threshold):
    """Fit a line to the matching pixels in the ROI, across firmware versions."""
    global _reg_target_size_ok

    if _reg_target_size_ok:
        try:
            return img.get_regression(
                threshold,
                roi=ROI,
                x_stride=REG_X_STRIDE,
                y_stride=REG_Y_STRIDE,
                pixels_threshold=REG_MIN_PIXELS,
                area_threshold=REG_MIN_AREA,
                target_size=REG_TARGET_SIZE,
            )
        except TypeError:
            _reg_target_size_ok = False
            print("get_regression: no target_size on this firmware, omitting")

    return img.get_regression(
        threshold,
        roi=ROI,
        x_stride=REG_X_STRIDE,
        y_stride=REG_Y_STRIDE,
        pixels_threshold=REG_MIN_PIXELS,
        area_threshold=REG_MIN_AREA,
    )


# ============================================================================
# LINE DETECTORS
# ============================================================================

def _order_near_first(x1, y1, x2, y2):
    """Return the segment endpoints as (near_x, near_y, far_x, far_y)."""
    first_is_near = (y1 >= y2) if NEAR_FIELD_AT_BOTTOM else (y1 <= y2)
    if first_is_near:
        return x1, y1, x2, y2
    return x2, y2, x1, y1


def _segment_angle(near_x, near_y, far_x, far_y):
    """Angle of the segment from straight ahead: 0 = ahead, + = leaning right.

    Worked out from the ENDPOINTS rather than from the fit's theta(), which is a
    Hough-convention normal angle - a vertical line is theta 0, not 90 - and is
    easy to get backwards.
    """
    return math.degrees(math.atan2(far_x - near_x, abs(near_y - far_y)))


NO_LINE = {'ahead': False, 'fitted': False, 'angle': 0, 'length': 0,
           'near_x': 0, 'near_y': 0, 'far_x': 0, 'far_y': 0, 'why': 'no fit'}


def detect_line_ahead(img, continuous=True,
                      min_length=MIN_LINE_LENGTH,
                      max_angle=MAX_LINE_ANGLE):
    """Fit the BLACK pixels in the ROI and decide whether a line is there.

    Always black: 'J' asks because the robot follows a black line, 'M' because
    the line it is about to rejoin is black even though the robot is still in
    white-line mode, and 'A' because it is looking for that same black line
    further off.

    continuous  require the fit to START at the bottom centre, i.e. to be
                continuous with where the robot already is. True for 'J' and
                'M', which ask whether the line carries on from HERE. False for
                'A', which is asked precisely when the robot may have drifted
                off it - keeping the test there would reject the very case the
                question exists to catch.

    Returns a dict with the fit and an 'ahead' verdict. 'why' names the first
    test that failed, which is what to watch while tuning.
    """
    line = _get_regression(img, BLACK_THRESHOLD)
    if line is None or _attr(line, 'magnitude') < REG_MIN_MAGNITUDE:
        return dict(NO_LINE)

    near_x, near_y, far_x, far_y = _order_near_first(
        _attr(line, 'x1'), _attr(line, 'y1'),
        _attr(line, 'x2'), _attr(line, 'y2'))

    angle = _segment_angle(near_x, near_y, far_x, far_y)
    length = math.sqrt((far_x - near_x) ** 2 + (far_y - near_y) ** 2)

    result = {'ahead': False, 'fitted': True, 'angle': int(angle),
              'length': int(length), 'near_x': near_x, 'near_y': near_y,
              'far_x': far_x, 'far_y': far_y, 'why': ''}

    # Pointing away from the robot, not across it.
    if abs(angle) > max_angle:
        result['why'] = 'angle %d' % int(angle)
        return result

    # Continuous with where the robot already is, not some line off to one side.
    if continuous:
        if abs(near_x - ROI_CENTER_X) > NEAR_X_TOLERANCE:
            result['why'] = 'near x off by %d' % int(near_x - ROI_CENTER_X)
            return result
        if abs(near_y - NEAR_EDGE_Y) > NEAR_Y_TOLERANCE:
            result['why'] = 'near y off by %d' % int(abs(near_y - NEAR_EDGE_Y))
            return result

    # Reaches past the crossing arm into the far field. This is the test that
    # does the real work: at the moment the hub asks, the robot is sitting ON
    # the arm, so the near field is black either way. What separates a crossing
    # from a corner is black CONTINUING.
    if length < min_length:
        result['why'] = 'short (%d < %d)' % (int(length), min_length)
        return result

    result['ahead'] = True
    result['why'] = 'ok'
    return result


def measure_coverage(img, threshold):
    """Percentage of COVER_ROI whose pixels match `threshold`.

    Merged blobs are summed, so this is a genuine pixel count rather than a
    bounding-box estimate. Reported raw: the hub decides what it means.
    """
    pixels = 0
    for blob in img.find_blobs(threshold, roi=COVER_ROI, merge=True,
                               pixels_threshold=COVER_MIN_PIXELS,
                               area_threshold=COVER_MIN_PIXELS):
        pixels += _attr(blob, 'pixels')
    percent = pixels * 100 // COVER_AREA
    return 100 if percent > 100 else percent


# ============================================================================
# ZONE DETECTORS
# ============================================================================
# One find_blobs() pass finds every zone target: the call takes a LIST of
# thresholds and tags each blob with code(), so green, red, dark-neutral and
# light-neutral come back from a single pass over the image rather than four.

NO_TARGET = {'kind': KIND_NONE, 'bearing': 0, 'range': 0,
             'cx': 0, 'cy': 0, 'shape': None, 'why': 'nothing found'}

# Per-frame record of every candidate blob and what became of it. Filled only in
# STANDALONE, because building it costs allocations the real run does not need -
# and because on the robot nobody is reading it.
_zone_debug = []

# Every candidate's outline for the overlay, as
# (kind, accepted, cx, cy, radius, corners). Kept for ALL candidates, not just
# the winner: seeing what was found and rejected is most of what makes the
# picture worth looking at.
_zone_shapes = []


def _bearing_of(cx):
    """Horizontal angle from straight ahead, + = right."""
    return (cx - FRAME_WIDTH / 2.0) * HORIZONTAL_FOV_DEG / FRAME_WIDTH


def _range_of(base_y, frame_height):
    """Ground distance to a point resting on the floor at image row `base_y`.

    Uses the BASE of the blob, not its centre: a ball's bottom edge is where it
    touches the floor, and a triangle's is where its wall meets it. The centre
    would sit above the floor by half the object's height and read as further
    away than it is.

    Returns 0 when the geometry gives no usable answer - a row at or above the
    horizon, or a distance beyond the zone's own dimensions, both of which mean
    the flat-floor model has broken down rather than that the target is far.
    """
    below_centre = base_y - frame_height / 2.0
    angle = ZONE_CENTRE_BELOW_DEG + below_centre * VERTICAL_FOV_DEG / frame_height
    if angle <= 1:
        return 0
    distance = CAMERA_HEIGHT_MM / math.tan(math.radians(angle))

    if distance <= 0 or distance > ZONE_MAX_RANGE_MM:
        return 0
    return int(distance)


def _first_code_index(code):
    """Lowest threshold index a blob matched.

    code() is a bitmask, and a blob can match more than one window. Taking the
    lowest is what makes the ordering in ZONE_THRESHOLDS meaningful: the
    specific colours are listed first, so a green triangle that also happens to
    fall inside the neutral sphere window is still reported as green.
    """
    for index in range(len(ZONE_THRESHOLDS)):
        if code & (1 << index):
            return index
    return -1


# find_circles() keyword sets have moved between firmware versions in the same
# way get_regression()'s did, so probe once and latch rather than assuming.
_circles_ok = True


def _luma(img, x, y):
    """0-255 brightness of one pixel.

    get_pixel() takes its coordinate as a single tuple on this firmware, the
    same pattern as draw_cross() and draw_string(), and returns (r, g, b).
    """
    r, g, b = img.get_pixel((x, y))
    return 0.299 * r + 0.587 * g + 0.114 * b


def _classify_sphere(img, cx, cy, r):
    """Dead victim or live one. Returns (kind, reason).

    Shape already established that this is a ball; this only asks how dark it
    is. It grids the face and counts, rather than averaging a handful of
    samples, because averaging cannot separate a uniformly dark ball from a soft
    shadow gradient that happened to fit a circle - and counting can.

    The grid offset is r//3, not r//2. The four CORNER samples combine a
    horizontal and a vertical offset, so they sit at offset*sqrt(2) from the
    centre: at r//2 that is 0.71r, easily past the true edge whenever the Hough
    fit reports a slightly oversized radius, sampling bright floor and flipping
    a real dead ball to live. At r//3 the corners land at 0.47r, comfortably
    inside even a loose fit.
    """
    offset = max(1, r // 3)
    dark = 0
    total = 0

    for grid_x in (-offset, 0, offset):
        for grid_y in (-offset, 0, offset):
            px = min(max(cx + grid_x, 0), img.width() - 1)
            py = min(max(cy + grid_y, 0), img.height() - 1)
            total += 1
            if _luma(img, px, py) <= DARK_SPHERE_LUMA_MAX:
                dark += 1

    if dark / float(total) >= DEAD_BALL_MIN_DARK_FRACTION:
        return (KIND_SPHERE_DARK, 'dead (%d/%d dark)' % (dark, total))
    return (KIND_SPHERE_LIGHT, 'live (%d/%d dark)' % (dark, total))


def _sphere_circles(img):
    """Pass 1. Hough circles over the whole frame - the black victim.

    Returns a list of (cx, cy, r). Empty when the firmware will not run
    find_circles(), which latches _circles_ok False and hands the black victim
    to the colour fallback.
    """
    global _circles_ok

    if not _circles_ok:
        return []

    try:
        circles = img.find_circles(threshold=CIRCLE_THRESHOLD,
                                   x_stride=CIRCLE_STRIDE,
                                   y_stride=CIRCLE_STRIDE,
                                   r_min=SPHERE_R_MIN_PX,
                                   r_max=SPHERE_R_MAX_PX,
                                   r_step=CIRCLE_R_STEP)
    except (TypeError, AttributeError):
        _circles_ok = False
        print("find_circles unavailable on this firmware, "
              "falling back to the dark colour window")
        return []

    return [(_attr(c, 'x'), _attr(c, 'y'), _attr(c, 'r')) for c in circles]


def _textured_spheres(img, existing):
    """Pass 2. Local texture - the silver victims, which pass 1 never catches.

    Returns (cx, cy, radius, kept, reason) for EVERY candidate, rejected ones
    included, so the overlay can show what was considered.

    `existing` is pass 1's list of (cx, cy, r). A texture speck landing inside a
    circle pass 1 already fitted is dropped before the shape tests even run:
    pass 1's fit is the trustworthy one, and a surface mark or a compression
    artefact inside an identified ball must not get to relitigate it. The same
    reasoning applies within this pass - one ball whose texture fragments into
    two passing blobs must not be reported as two victims.
    """
    texture = img.copy()
    texture = texture.to_grayscale()
    blurred = texture.copy()
    blurred.mean(TEXTURE_WINDOW)
    texture.difference(blurred)
    texture.binary([(TEXTURE_MIN, TEXTURE_MAX)])
    texture.dilate(TEXTURE_DILATE)

    candidates = []
    accepted = []

    # binary() has already reduced this to on/off, so one wide window is the
    # whole threshold.
    for blob in texture.find_blobs([(128, 255)],
                                   pixels_threshold=TEXTURE_MIN_PIXELS,
                                   area_threshold=TEXTURE_MIN_PIXELS,
                                   merge=True):
        cx = _attr(blob, 'cx')
        cy = _attr(blob, 'cy')
        width = _attr(blob, 'w')
        height = _attr(blob, 'h')
        radius = (width + height) // 4

        inside = None
        for ex, ey, er in existing:
            if ((cx - ex) ** 2 + (cy - ey) ** 2) ** 0.5 < er:
                inside = 'inside a pass-1 circle'
                break
        if inside:
            candidates.append((cx, cy, radius, False, inside))
            continue

        # A glint can be small and round enough to pass the shape tests. A real
        # ball's texture patch should be roughly BALL-SIZED, so the same radius
        # bounds pass 1 uses apply here - a lone highlight rarely lands in that
        # window even after dilation.
        if not SPHERE_R_MIN_PX <= radius <= SPHERE_R_MAX_PX:
            candidates.append((cx, cy, radius, False,
                               'radius %d outside %d-%d'
                               % (radius, SPHERE_R_MIN_PX, SPHERE_R_MAX_PX)))
            continue

        duplicate = None
        for ax, ay, ar in accepted:
            if ((cx - ax) ** 2 + (cy - ay) ** 2) ** 0.5 < (radius + ar):
                duplicate = 'duplicate of a texture blob already taken'
                break
        if duplicate:
            candidates.append((cx, cy, radius, False, duplicate))
            continue

        roundness = abs(width - height) / float(max(width, height))
        fill = _attr(blob, 'pixels') / float(width * height)

        if roundness > TEXTURE_ROUNDNESS_TOL:
            candidates.append((cx, cy, radius, False,
                               'not round (%.2f > %.2f)'
                               % (roundness, TEXTURE_ROUNDNESS_TOL)))
            continue
        if fill < TEXTURE_MIN_FILL:
            candidates.append((cx, cy, radius, False,
                               'ring not disc (fill %.2f < %.2f)'
                               % (fill, TEXTURE_MIN_FILL)))
            continue

        accepted.append((cx, cy, radius))
        candidates.append((cx, cy, radius, True,
                           'textured (round %.2f fill %.2f)' % (roundness, fill)))

    return candidates


def _classify_blob(img, blob, index):
    """Decide what one candidate blob is. Returns (kind, reason).

    kind is KIND_NONE when the blob is rejected, and reason always says why -
    which is what STANDALONE prints. A detector that only reports its winner
    cannot tell you whether a victim was missed because the colour window is
    wrong, the shape filters are too tight, or it was never a blob at all.
    """
    width = _attr(blob, 'w')
    height = _attr(blob, 'h')
    if width < 1 or height < 1:
        return (KIND_NONE, 'degenerate box')

    pixels = _attr(blob, 'pixels')
    density = pixels / float(width * height)
    aspect = float(width) / height

    kind = ZONE_CODE_KIND.get(index, KIND_NONE)
    if kind == KIND_NONE:
        return (KIND_NONE, 'unmapped threshold %d' % index)

    if kind in POINT_KINDS:
        if pixels < POINT_MIN_PIXELS:
            return (KIND_NONE, 'point too small (%d < %d)'
                    % (pixels, POINT_MIN_PIXELS))
        if density > POINT_MAX_DENSITY:
            return (KIND_NONE, 'point too solid (%.2f > %.2f)'
                    % (density, POINT_MAX_DENSITY))
        return (kind, 'point ok')

    if pixels < SPHERE_MIN_PIXELS:
        return (KIND_NONE, 'sphere too small (%d < %d)'
                % (pixels, SPHERE_MIN_PIXELS))
    if pixels > SPHERE_MAX_PIXELS:
        return (KIND_NONE, 'sphere too big (%d > %d)'
                % (pixels, SPHERE_MAX_PIXELS))
    if abs(aspect - 1.0) > SPHERE_ASPECT_TOL:
        return (KIND_NONE, 'not square (aspect %.2f)' % aspect)
    if density < SPHERE_MIN_DENSITY:
        return (KIND_NONE, 'too hollow (%.2f < %.2f)'
                % (density, SPHERE_MIN_DENSITY))
    # No brightness test here any more. Dark-versus-silver was decided the
    # moment the pixel matched one sphere window rather than the other, and the
    # old get_statistics() pass was measuring the SAME box the shadow had
    # already inflated - so it could only ever agree with, or be misled by, the
    # segmentation it was meant to check.
    return (kind, 'dark sphere' if kind == KIND_SPHERE_DARK
            else 'light sphere')


def detect_zone_target(img, wanted=KIND_NONE):
    """Find the NEAREST zone target, optionally of one kind only.

    wanted  KIND_* to narrow the search, or KIND_NONE for any.

    Returns a dict describing one target, or NO_TARGET.

    Nearest rather than largest: the hub navigates on range, and the 360 degree
    survey will sweep the rest into view anyway. A frame holding two targets
    reports one and loses nothing.

    TWO detectors feed one ranking. Circles find the spheres, colour blobs find
    the evacuation points, and both hand their candidates to the same consider()
    so that "nearest" is decided across the whole frame rather than per method.
    """
    global _zone_debug, _zone_shapes

    frame_height = img.height()
    _zone_shapes = []
    if STANDALONE:
        _zone_debug = []

    # A one-element list rather than a plain name: MicroPython has no nonlocal,
    # and consider() has to be able to replace it.
    best = [None]

    def consider(kind, reason, shape, base_y, note):
        """Rank one candidate, record it for the overlay, and trace it.

        shape is (cx, cy, radius, width, height, corners) - enough for the
        overlay to draw either the object's real outline or its plain bounding
        box, and the choice between those is what tells confirmed from merely
        considered.

        Every candidate reaches the overlay whether it was accepted or not.
        A detector that only draws its winner cannot tell you whether a victim
        was missed because it was never found, or found and then rejected.
        """
        cx = shape[0]
        distance = _range_of(base_y, frame_height) if kind else 0

        if kind and distance == 0:
            reason = 'range out of model (base row %d)' % base_y
            kind = KIND_NONE
        elif kind and wanted != KIND_NONE and kind != wanted:
            reason = 'filtered out (wanted %d)' % wanted
            kind = KIND_NONE

        _zone_shapes.append((kind, kind != KIND_NONE, shape))

        if STANDALONE:
            _zone_debug.append(
                "%s %s%s"
                % (note, reason,
                   "  -> %d mm @ %+d deg" % (distance, int(_bearing_of(cx)))
                   if kind else ""))

        if kind and (best[0] is None or distance < best[0]['range']):
            best[0] = {'kind': kind, 'bearing': int(_bearing_of(cx)),
                       'range': distance, 'cx': cx, 'cy': shape[1],
                       'shape': shape, 'why': reason}

    # ---- spheres, pass 1: outline ------------------------------------------
    # First, because it is what sets _circles_ok - and the colour pass below
    # needs to know whether it is the fallback or not.
    circles = _sphere_circles(img)
    for cx, cy, r in circles:
        kind, reason = _classify_sphere(img, cx, cy, r)
        consider(kind, reason, (cx, cy, r, 2 * r, 2 * r, None),
                 min(cy + r, frame_height - 1),
                 "circle r%-3d" % r)

    # ---- spheres, pass 2: texture ------------------------------------------
    # Runs whether or not pass 1 did. It is told what pass 1 found so the same
    # ball cannot be counted twice, but an empty list is a valid answer: if
    # find_circles() is unavailable the silver victims are the ones that still
    # need finding, and gating this on the latch would lose them along with the
    # black one. A smooth black ball reaching here presents as a RING in the
    # difference map, which the fill test rejects.
    for cx, cy, r, kept, reason in _textured_spheres(img, circles):
        if kept:
            kind, reason = _classify_sphere(img, cx, cy, r)
        else:
            kind = KIND_NONE
        consider(kind, reason, (cx, cy, r, 2 * r, 2 * r, None),
                 min(cy + r, frame_height - 1),
                 "texture r%-3d" % r)

    # ---- evacuation points, by colour --------------------------------------
    # merge=False deliberately. Merging joins OVERLAPPING blobs regardless of
    # which threshold they matched, so a ball resting against a green wall would
    # become one blob, take the lowest code, and be reported as an evacuation
    # point. The targets here are discrete objects that gain nothing from
    # merging, and a silent misclassification costs far more.
    for blob in img.find_blobs(ZONE_THRESHOLDS, merge=False,
                               pixels_threshold=SPHERE_MIN_PIXELS,
                               area_threshold=SPHERE_MIN_PIXELS):
        index = _first_code_index(_attr(blob, 'code'))
        if index < 0:
            continue

        kind, reason = _classify_blob(img, blob, index)

        # The dark window is the SPHERE FALLBACK and nothing else. While
        # find_circles() is working it must stay silent, or the black victim is
        # detected twice by two methods that can disagree about where it is.
        if kind == KIND_SPHERE_DARK and _circles_ok:
            kind, reason = KIND_NONE, 'dark blob suppressed, circles are live'

        width = _attr(blob, 'w')
        height = _attr(blob, 'h')
        consider(kind, reason,
                 (_attr(blob, 'cx'), _attr(blob, 'cy'),
                  max(width, height) // 2, width, height,
                  _attr(blob, 'corners')),
                 _attr(blob, 'y') + height,
                 "th%d %4dpx %3dx%-3d" % (index, _attr(blob, 'pixels'),
                                          width, height))

    return best[0] if best[0] is not None else dict(NO_TARGET)


KIND_COLOUR = {KIND_SPHERE_DARK: BLUE, KIND_SPHERE_LIGHT: YELLOW,
               KIND_POINT_GREEN: GREEN, KIND_POINT_RED: RED}

# Rejected candidates are drawn too, in one muted colour, so the picture shows
# what was considered as well as what won.
REJECTED_COLOUR = (90, 90, 90)


def _draw_outline(img, kind, accepted, shape, colour, thickness):
    """The MARK says how far a candidate got, before its colour says what it is.

    Confirmed gets the object's own geometry: a circle at the fitted radius,
    sitting on the ball's edge, or a polygon through the blob's corners
    following a triangle's slant. A glance then shows whether the shape tests
    agreed with the object, which is the whole question while tuning.

    Still a candidate gets a plain bounding BOX - deliberately the crudest mark
    on the frame. A circle drawn around a rejected blob reads as a detection at
    a glance, which is exactly the confusion this overlay exists to prevent, and
    a rectangle can never be mistaken for a fitted outline.
    """
    cx, cy, radius, width, height, corners = shape

    if not accepted:
        img.draw_rectangle((cx - width // 2, cy - height // 2, width, height),
                           color=colour, thickness=thickness)
        return

    if kind in POINT_KINDS and corners:
        for index in range(len(corners)):
            start_point = corners[index]
            end_point = corners[(index + 1) % len(corners)]
            img.draw_line((start_point[0], start_point[1],
                           end_point[0], end_point[1]),
                          color=colour, thickness=thickness)
        return

    img.draw_circle((cx, cy, radius), color=colour, thickness=thickness)


def draw_overlay(img, result, question, coverage):
    """The LINE answer, drawn the way draw_zone_overlay() draws the zone one.

    Three things, because three things decide the verdict: the ROI the fit was
    allowed to look in, the fit itself, and why it was accepted or rejected.
    A frame that shows only an accepted line cannot tell you whether a rejected
    one failed on angle, on continuity or on length - and those want different
    fixes.

    Colour carries the verdict so it reads at a glance while the robot is
    moving: green accepted, red rejected, and the reason string beside it names
    the first test that failed.
    """
    img.draw_rectangle(ROI, color=REJECTED_COLOUR, thickness=1)

    colour = RED
    if result['fitted']:
        colour = GREEN if result['ahead'] else RED
        img.draw_line((result['near_x'], result['near_y'],
                       result['far_x'], result['far_y']),
                      color=colour, thickness=2)
        # The NEAR end is the one the continuity test judges, so mark which end
        # the fit decided that was.
        img.draw_cross((result['near_x'], result['near_y']), color=colour)

    img.draw_string((4, 4), "%s %s a=%+d L=%d c=%d%%"
                    % (chr(question),
                       'ahead' if result['ahead'] else result['why'],
                       result['angle'], result['length'], coverage),
                    color=colour)


def draw_zone_overlay(img, target):
    """Draw every candidate, with the chosen one picked out."""
    for kind, accepted, shape in _zone_shapes:
        colour = KIND_COLOUR.get(kind, REJECTED_COLOUR) if accepted \
            else REJECTED_COLOUR
        _draw_outline(img, kind, accepted, shape, colour, 1)

    if target['kind'] == KIND_NONE:
        img.draw_string((4, 4), "Z none", color=RED)
        return

    colour = KIND_COLOUR[target['kind']]
    _draw_outline(img, target['kind'], True, target['shape'], colour, 3)
    img.draw_cross((target['cx'], target['cy']), color=colour)
    img.draw_string((4, 4), "Z kind=%d b=%+d r=%dmm"
                    % (target['kind'], target['bearing'], target['range']),
                    color=colour)


# ============================================================================
# PROTOCOL
# ============================================================================

# The answer the hub will read on its next poll, as (echo, ahead, angle, cover).
# Recomputed every frame; the callback only hands it back.
_payload = (0, 0, 0, 0)

# What the hub last asked, the sequence byte that came with it, and the kind
# filter that rode in byte 2 (used by 'Z' only).
_question = QUESTION_JUNCTION
_seq = 0
_wanted_kind = KIND_NONE

# Where the platform is currently aimed, and until when its answers are
# untrustworthy. None means "not aimed yet", so the first frame always moves.
_aim_tilt = None
_settle_until = 0


def _tilt_for(question):
    """Where the camera should point for this question. ONE answer, one place.

    The main loop moves the servo to this and aimed_for() compares against it,
    so they must never disagree: if they did, the aim check would report "not
    yet pointed" forever and every sequence byte would be withheld.
    """
    if STANDALONE and question == QUESTION_ZONE:
        return STANDALONE_ZONE_TILT
    return QUESTION_TILT.get(question, CAMERA_TILT_LINE)


def aimed_for(question):
    """True if the platform is already pointed where `question` wants it, and
    has stopped moving.

    BOTH conditions matter. Checking only the settle timer leaves a hole on the
    very first poll of a new question: that request arrives BEFORE the main loop
    has noticed the aim must change, so nothing is settling yet, the sequence
    byte gets adopted, and the next frame - taken as the servo starts - is
    published with a matching echo. The aim comparison closes it, because the
    mismatch is visible from the moment the question changes.
    """
    if _tilt_for(question) != _aim_tilt:
        return False
    return time.ticks_diff(_settle_until, time.ticks_ms()) <= 0


def settling():
    """True while the servo is still travelling or has just arrived."""
    return time.ticks_diff(_settle_until, time.ticks_ms()) > 0


def look(request):
    """PUPRemote 'look' callback, run when the hub polls.

    Records the question for the NEXT frame to answer and returns the answer
    computed from the LAST one. It cannot do better than that: the frame is
    already taken by the time this runs, which is exactly why the reply carries
    an echo of the sequence byte it was computed for.
    """
    global _question, _seq, _wanted_kind

    if request and len(request) >= 3:
        _wanted_kind = request[2]
    if request and len(request) >= 2:
        asked = request[0]
        if asked in QUESTIONS:
            # The question is adopted immediately - the main loop needs it to
            # know where to aim - but the SEQUENCE BYTE is withheld until the
            # camera is actually pointed where that question wants and has
            # stopped moving. The echo then cannot match, so the hub keeps
            # polling rather than trusting a fit taken from a smeared frame.
            # Nothing else stops it: the move is non-blocking.
            _question = asked
            if aimed_for(asked):
                _seq = request[1]

    return _payload


pup = PUPRemoteSensor(power=False, max_packet_size=16)
pup.add_command('look', to_hub_fmt="hhhh", from_hub_fmt="4s")


def pump_comms(duration_ms):
    """Service the link for `duration_ms`.

    A corrupted frame can leave a garbage mode index that makes process() raise;
    ignore it and carry on rather than killing the program mid-run.
    """
    start = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start) < duration_ms:
        try:
            pup.process()
        except Exception:
            pass
        time.sleep_ms(1)


# ============================================================================
# MAIN LOOP
# ============================================================================

def main():
    global _payload, _aim_tilt, _settle_until, _question, _wanted_kind

    setup_camera()

    platform = PanTilt()
    platform.center()
    print("Aim follows the question: line %d, ahead %d, zone %d, pan %d."
          % (CAMERA_TILT_LINE, CAMERA_TILT_AHEAD,
             _tilt_for(QUESTION_ZONE), CAMERA_PAN))
    print("Tests: |angle| <= %d, near end within (%d, %d) of bottom centre, "
          "length >= %d px"
          % (MAX_LINE_ANGLE, NEAR_X_TOLERANCE, NEAR_Y_TOLERANCE,
             MIN_LINE_LENGTH))

    clock = time.clock()
    frames = 0

    if STANDALONE:
        _question = STANDALONE_QUESTION
        _wanted_kind = STANDALONE_KIND
        print("STANDALONE: holding question %s, kind filter %d, no hub link."
              % (chr(STANDALONE_QUESTION), STANDALONE_KIND))

    while True:
        clock.tick()
        question = _question

        # Aim follows the question. Moving only on a CHANGE matters: pantilt
        # re-issues the interpolation every call, so moving unconditionally
        # would restart the travel every frame and the camera would never
        # settle.
        wanted = _tilt_for(question)
        if wanted != _aim_tilt:
            platform.move(tilt=wanted, pan=CAMERA_PAN, ms=TILT_MOVE_MS)
            _aim_tilt = wanted
            _settle_until = time.ticks_add(time.ticks_ms(),
                                           TILT_MOVE_MS + TILT_SETTLE_MS)
            print("aim -> %s tilt %d" % (chr(question), wanted))
            #time.sleep(1)


        # The sensor itself follows the question, not only the detector.
        if _apply_exposure(question):
            deadline = time.ticks_add(time.ticks_ms(), EXPOSURE_SETTLE_MS)
            if time.ticks_diff(deadline, _settle_until) > 0:
                _settle_until = deadline

        img = sensor.snapshot()

        # ONE detector per frame, chosen by the question. That is what keeps
        # the frame budget predictable - running the line fit and the zone pass
        # together would halve the rate for no benefit, since the hub only ever
        # wants one answer.
        if question == QUESTION_ZONE:
            target = detect_zone_target(img, _wanted_kind)
            if settling():
                target = dict(NO_TARGET)
                target['why'] = 'settling'
            _payload = (_seq, target['kind'], target['bearing'],
                        target['range'])
            draw_zone_overlay(img, target)
            result = None
            coverage = 0
        else:
            if question == QUESTION_AHEAD:
                result = detect_line_ahead(img, continuous=False,
                                           min_length=AHEAD_MIN_LINE_LENGTH,
                                           max_angle=AHEAD_MAX_LINE_ANGLE)
            else:
                result = detect_line_ahead(img)

            coverage = measure_coverage(
                img, WHITE_THRESHOLD if question == QUESTION_SEAM
                else BLACK_THRESHOLD)

            # While the servo is travelling the frame is unusable, so publish
            # the verdict as "nothing seen". The withheld sequence byte is what
            # actually protects the hub - this just avoids drawing a confident
            # overlay from a smeared image.
            if settling():
                result = dict(NO_LINE)
                result['why'] = 'settling'

            _payload = (_seq,
                        1 if result['ahead'] else 0,
                        result['angle'],
                        coverage)

            draw_overlay(img, result, question, coverage)

        # No hub in standalone, so nothing to service - and pumping a link
        # nobody is on would only cost frame time.
        if ENABLE_COMMS and not STANDALONE:
            pump_comms(COMMS_PUMP_MS)

        frames += 1

        if STANDALONE and frames % STANDALONE_PRINT_EVERY == 0:
            if question == QUESTION_ZONE:
                print("--- frame %d  %.1f fps  %d candidate(s)"
                      % (frames, clock.fps(), len(_zone_debug)))
                for entry in _zone_debug:
                    print("      " + entry)
                print("    chosen: kind=%d bearing=%+d range=%dmm (%s)"
                      % (target['kind'], target['bearing'], target['range'],
                         target['why']))
            else:
                print("--- frame %d  %.1f fps  %s ahead=%d angle=%+d "
                      "cover=%d%% (%s)"
                      % (frames, clock.fps(), chr(question),
                         1 if result['ahead'] else 0, result['angle'],
                         coverage, result['why']))

        if PRINT_EVERY and not STANDALONE and frames % PRINT_EVERY == 0:
            if result is None:
                print("%d: Z seq=%d kind=%d bearing=%+d range=%dmm (%s) %.1f fps"
                      % (frames, _seq, target['kind'], target['bearing'],
                         target['range'], target['why'], clock.fps()))
            else:
                print("%d: %s seq=%d ahead=%d angle=%d cover=%d%% (%s) %.1f fps"
                      % (frames, chr(question), _seq,
                         1 if result['ahead'] else 0, result['angle'],
                         coverage, result['why'], clock.fps()))


if __name__ == "__main__":
    main()