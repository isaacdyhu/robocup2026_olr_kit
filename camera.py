"""
camera.py -- combined LINE mode + ZONE mode vision (design.md Sec3:
"Camera: one program, two personalities").

Merges two scripts that were developed and tuned separately:
  - LINE mode (black/white line following, background/polarity
    detection) -- this file's earlier form.
  - ZONE mode (sphere and evacuation-triangle detection, distance and
    bearing estimation) -- developed in isolation in
    evac_zone/zone_scan.py so it could be tuned without needing to fake
    "lost on a white background" first. That file is left as-is and
    still works standalone if ZONE mode needs isolated tuning again;
    this is where its tuned constants and functions land once they work.

ZONE_MODE_ENABLED below is a manual switch, for now: it picks ONE mode
for the whole run rather than automatically transitioning between them
the way design.md's real state machine does (state B: white background,
no line found, for real -- not a hand-flipped constant). Automatic
switching is future work; flip this and re-run to test the other mode
until then.
"""

import math
import sensor
import time

# --- camera setup -----------------------------------------------------------

sensor.reset()
sensor.set_pixformat(sensor.RGB565)  # colour, so the overlay can use colour
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)  # let auto-exposure settle before reading it below

sensor.set_auto_gain(False)      # must be off for stable thresholding
sensor.set_auto_whitebal(False)  # ditto
sensor.set_vflip(True)    # camera mounted upside-down -- flip vertically...
sensor.set_hmirror(True)  # ...and horizontally, for a full 180 deg rotation
clock = time.clock()

_full_frame = sensor.snapshot()  # discarded -- just used to read real dimensions
FULL_W = _full_frame.width()
FULL_H = _full_frame.height()

_auto_exposure_us = sensor.get_exposure_us()

# --- mode switch -------------------------------------------------------------

# Manual for now (see the file header) -- True runs ZONE mode for the
# whole session, False runs LINE mode. Flip and re-run to test the other.
ZONE_MODE_ENABLED = False

# --- tunables (LINE mode) -----------------------------------------------
#
# LAB thresholds: (L min, L max, A min, A max, B min, B max). L runs 0-100
# in OpenMV regardless of pixel format; A/B are left wide open since black
# and white here are brightness-only distinctions, not colour ones.
# TUNE THESE against the real course -- these are untested placeholders.

BLACK_THRESHOLD = (0, 35, -128, 127, -128, 127)
WHITE_THRESHOLD = (65, 100, -128, 127, -128, 127)

MIN_BLOB_PIXELS = 40       # ignore noise specks
MIN_LINE_LENGTH_PX = 150   # matches the retired program's MIN_LINE_LENGTH
BACKGROUND_COVERAGE = 50   # % of the frame a colour must cover to count as
                           # "the background" rather than just a thick line

# Which line to track when the background classification below can't tell
# (start-up, before either coverage number means anything yet, or a frame
# where neither colour clearly dominates). Whatever this picks only lasts
# until the first confident background reading.
DEFAULT_POLARITY = "black"  # "black" or "white"

# LINE cares about black/white line contrast; this is the brightness this
# file used before ZONE mode's needs were understood.
LINE_BRIGHTNESS_FRACTION = 0.5

# --- tunables (ZONE mode) ------------------------------------------------
#
# Narrowed window as a fraction of the full frame, centred -- both axes
# shrink together, which is what actually narrows the captured field of
# view (crops the sensor's readout, isn't just a resize). Currently 1.0
# (no crop) -- that's what testing has actually used successfully so far;
# shrink these once ZONE mode needs to look at a smaller, more distant
# patch of floor instead of the whole scene.
ZONE_WINDOW_W_FRAC = 1
ZONE_WINDOW_H_FRAC = 1

# The silver balls are pressed, scrunched foil, not a smooth mirror --
# lots of small facets each catching light at a different angle, not one
# clean highlight-to-rim gradient. That texture needs a *brighter*
# exposure to stay visible; a darker exposure was tried first on the
# (wrong) assumption they were a smooth specular surface that would blow
# out to flat white.
ZONE_BRIGHTNESS_FRACTION = 1.3

# Evacuation-point corners are real, saturated colours (LAB: L, A, B each
# min/max). UNTESTED PLACEHOLDERS -- tune against the real triangles.
GREEN_THRESHOLD = (0, 80, -80, -10, 0, 60)
RED_THRESHOLD = (0, 80, 20, 80, 0, 60)

# Triangles are ~280 mm -- much bigger than the 39.6 mm course markers, so
# filter harder for noise. This also sidesteps design.md's green-marker-vs-
# triangle ambiguity in state B: ZONE mode here only ever runs because the
# manual switch says so, never because a course marker was mistaken for it.
ZONE_MIN_BLOB_PIXELS = 100

# find_circles' Hough-style vote threshold -- higher is stricter.
# Confirmed working at 2500: dead ball found as a single, sharply-defined
# circle, correctly classified. Lower values (tried to 1200) did not add
# sensitivity to the silver balls' weak edge -- they just made the dead
# ball's already-strong edge produce several overlapping duplicate
# circles instead of one. Silver-ball detection needed the texture pass
# below instead, not further pushes on this number.
CIRCLE_THRESHOLD = 2500
SPHERE_R_MIN_PX = 10  # raised from 6: small dark specks (shadows/creases
                      # at wall edges) were qualifying as circles below that
SPHERE_R_MAX_PX = 60  # untested, depends on scan distance and window size

# --- distance estimation ---------------------------------------------------
#
# Ground-plane projection: camera height + tilt angle + which row of the
# frame a point sits in is enough to solve for real-world distance, since
# the floor is a known flat plane. Chosen over estimating distance from
# the ball's *apparent size* because our sphere radius estimates are
# already known to be crude (bounding-box-derived, inflated by dilation
# in the texture pass) -- a size-based distance calc would inherit all of
# that error directly, whereas this only needs the pixel ROW where the
# ball touches the floor, not an accurate radius.
#
# ALL THREE OF THESE ARE UNVERIFIED PLACEHOLDERS, and CAMERA_TILT_DEG in
# particular inherits a real, currently-unresolved ambiguity: design.md's
# own Sec6 flags a contradiction between the mount's documented tilt-zero
# convention and what the existing camera code assumes. Don't trust
# estimate_distance_mm()'s output until it's checked against a tape
# measure at a couple of known real distances -- if there's a consistent
# offset or scale error, that's these constants needing correction, not a
# bug in the formula itself.
CAMERA_HEIGHT_MM = 150    # camera lens height above the floor
CAMERA_TILT_DEG = 30      # angle of the optical axis below horizontal;
                          # 0 = looking at the horizon, 90 = straight down
CAMERA_VFOV_DEG = 45      # vertical field of view of the CURRENT frame --
                          # depends on the lens and on ZONE_WINDOW_H_FRAC,
                          # so recalibrate this if that fraction changes

# 0-255 luma; a point sampled darker than this counts as "dark" when
# classify_sphere() below grids the circle's face -- shape already told
# us it's a ball, this only distinguishes dead from live, never tests
# for "silver" directly (design.md Sec7: silver is specular, its
# apparent colour is whatever's around it).
DARK_SPHERE_LUMA_MAX = 90

# A real dead ball is uniformly dark across its whole face; a wall-edge
# shadow that happens to get picked up as a circle usually isn't -- it's
# a soft gradient or a thin crease, dark in places rather than dark
# everywhere. Requiring most of a grid of sample points to be dark
# (rather than just averaging a handful of them) is what actually
# filters that case out. Untested.
DEAD_BALL_MIN_DARK_FRACTION = 0.8

# Texture-based detection for the silver balls (see the note above
# find_textured_spheres() below): a blur-then-subtract residue turns
# "subtly textured surface" into "clearly bright region" without relying
# on any absolute brightness/colour threshold. All untested placeholders.
TEXTURE_WINDOW = 2          # blur half-window -- (2*n+1)x(2*n+1) px
TEXTURE_MIN = 12            # residue level counted as "textured", 0-255 scale
# Upper bound: a direct specular highlight/reflection on the floor is a
# near-maximum-brightness spike in the difference map -- much brighter
# than the foil's subtler internal facet variation. Excluding the very
# top of the range keeps moderate texture, drops blown-out glints.
TEXTURE_MAX = 180
TEXTURE_MIN_PIXELS = 60     # ignore small noisy specks

# Uneven lighting (different facets of the foil catching light at
# different angles) can mean only PART of a ball's surface clears
# TEXTURE_MIN in any given frame, leaving a small scattered island of
# "textured" pixels instead of one region covering the ball -- which then
# fails the roundness/fill checks below even though the underlying signal
# is real. Dilating the thresholded map before blob detection grows and
# merges nearby islands into one connected region. Untested.
TEXTURE_DILATE = 2
ROUNDNESS_TOLERANCE = 0.35  # |w - h| / max(w, h) must be under this to
                            # count as "roughly round" rather than some
                            # other textured patch (a scratch, a shadow edge)

# A smooth object's edge (the dead ball, say) produces a bright *ring* in
# the texture map -- one strong brightness discontinuity at its outline --
# even though its face is perfectly flat and has no real surface texture.
# A genuinely textured surface like a foil ball lights up across its
# whole face instead. Both can look "round" via the bounding-box check
# above, so this is the check that tells a filled disc from a thin ring:
# pixels / (w * h) is small for a ring (most of its bounding box is empty),
# much higher for something filled in. Untested.
TEXTURE_MIN_FILL = 0.5

# Draws every texture-pass candidate blob, pass or fail, with its
# measured roundness/fill printed -- turn off once the silver balls are
# reliably detected and this is just visual clutter.
DEBUG_TEXTURE_CANDIDATES = True

# --- colours -------------------------------------------------------------

RED = (255, 40, 40)
BLUE = (40, 120, 255)
GREEN = (40, 220, 40)
YELLOW = (230, 220, 40)
TEXT_WHITE = (255, 255, 255)
MAGENTA = (230, 60, 220)
CYAN = (40, 220, 220)
GREY = (140, 140, 140)

# --- vision: LINE mode -----------------------------------------------------


def scan(img, threshold, min_pixels=MIN_BLOB_PIXELS):
    """
    Find blobs for one colour. Returns (biggest_blob_or_None, coverage) --
    coverage is 0-100, the % of the frame matching the threshold, counting
    every blob found, not just the biggest (design.md Sec3: a thin line
    gives low coverage, a background that's this colour gives high coverage).
    """
    blobs = img.find_blobs(
        [threshold],
        pixels_threshold=min_pixels,
        area_threshold=min_pixels,
        merge=True,
    )

    frame_px = img.width() * img.height()
    matched_px = 0
    biggest = None

    for b in blobs:
        matched_px += b.pixels
        if biggest is None or b.pixels > biggest.pixels:
            biggest = b

    coverage = min(100, int(100 * matched_px / frame_px))
    return biggest, coverage


def draw_line(img, biggest, colour):
    """
    Draw one blob as "the line" (rectangle, centroid cross, fitted axis)
    and return its (ahead, angle, length) reading. Nothing is drawn, and
    zeros are returned, if biggest is None.
    """
    if biggest is None:
        return 0, 0, 0

    img.draw_rectangle((biggest.x, biggest.y, biggest.w, biggest.h), color=colour, thickness=2)
    img.draw_cross((biggest.cx, biggest.cy), color=colour, size=6, thickness=2)

    # The fitted axis *is* the visible angle -- draw it in green regardless
    # of which polarity it came from, so it reads as "the line", not "a blob".
    #
    # major_axis_line doesn't exist on this object in OpenMV v5.0.0 (either
    # renamed or dropped in the attrtuple rewrite -- the docs site 404s
    # right now so this couldn't be confirmed). Built from scratch instead,
    # from fields already confirmed to exist (cx, cy, w, h, rotation), so
    # it can't break again on a future API-name guess. This is an
    # approximation -- max(w, h) as the half-length instead of the blob's
    # true fitted-axis length -- plenty good enough to see the lean.
    half_len = max(biggest.w, biggest.h) / 2
    dx = half_len * math.cos(biggest.rotation)
    dy = half_len * math.sin(biggest.rotation)
    axis_line = (
        int(biggest.cx - dx), int(biggest.cy - dy),
        int(biggest.cx + dx), int(biggest.cy + dy),
    )
    img.draw_line(axis_line, color=GREEN, thickness=2)

    length_px = biggest.h  # vertical (forward) extent in pixels
    ahead = 1 if length_px >= MIN_LINE_LENGTH_PX else 0

    # blob.rotation is only unique over [0, 180) degrees -- it can't tell
    # "leaning right at the top" from "leaning right at the bottom" of the
    # frame on its own. Fine near dead-ahead, ambiguous near a sharp turn.
    # Also unconfirmed: whether .rotation is radians (the historical
    # convention, hence math.degrees() here) or already degrees in v5.0.0 --
    # if the printed angle looks like it's off by a factor of ~57, that's it.
    angle_deg = int(math.degrees(biggest.rotation)) - 90

    return ahead, angle_deg, length_px


def classify_background(blk_coverage, wht_coverage):
    """
    Which colour dominates the frame, if either does. This is the same
    signal design.md's X state uses to ask "is there a black background
    ahead?" -- a thin 15.8 mm line never reaches BACKGROUND_COVERAGE, only
    a genuine background-sized region of one colour does.
    """
    if blk_coverage >= BACKGROUND_COVERAGE and blk_coverage > wht_coverage:
        return "black"
    if wht_coverage >= BACKGROUND_COVERAGE and wht_coverage > blk_coverage:
        return "white"
    return "unclear"


# --- vision: ZONE mode -------------------------------------------------------
#
# NOTE ON UNVERIFIED API SURFACE: find_circles()'s result objects are
# accessed here as attributes (c.x, not c.x()), matching the attrtuple
# pattern already confirmed for Blob on this firmware (OpenMV v5.0.0).
# Not independently confirmed for circle objects specifically -- if this
# errors, it'll be the same "isn't callable" / "no attribute" shape of
# error as the earlier blob fixes, and the same fix (add or remove the
# parentheses).


def _luma(img, x, y):
    """
    0-255 brightness estimate from a single pixel.

    get_pixel() wants its coordinate as a single (x, y) tuple on this
    firmware, same pattern as draw_cross/draw_rectangle/draw_string --
    called that way, it returns the (r, g, b) tuple as documented, no
    rgbtuple flag or manual RGB565 decoding needed after all.
    """
    r, g, b = img.get_pixel((x, y))
    return 0.299 * r + 0.587 * g + 0.114 * b


def classify_sphere(img, cx, cy, r):
    """
    Dead (black) vs live (silver). Shape (find_circles) already found this
    as a ball -- this only samples brightness, deliberately not a colour
    test (design.md Sec7).

    Grids the circle's face and requires most of the grid to be dark,
    rather than just averaging a handful of points -- a real dead ball is
    dark essentially everywhere across its face, while a wall-edge shadow
    that happens to get picked up as a circle is usually a soft gradient
    or a thin crease, dark only in places. Averaging a few points can't
    tell those apart; counting what fraction is actually dark can.

    offset is r//3, not r//2: the four *corner* grid points combine a
    horizontal and vertical offset, so their actual distance from centre
    is offset*sqrt(2) -- at r//2 that's ~0.71r, easily past the true edge
    if the Hough fit ever reports a slightly oversized radius (a shadow
    blending into the fit, say), sampling bright background instead of
    the ball and occasionally flipping a real dead ball to "live". At
    r//3 the corners land at ~0.47r, comfortably inside even a somewhat
    oversized fit.
    """
    offset = max(1, r // 3)  # 3x3 grid inside the circle
    dark_count = 0
    total_count = 0

    for gx in (-offset, 0, offset):
        for gy in (-offset, 0, offset):
            px = min(max(cx + gx, 0), img.width() - 1)
            py = min(max(cy + gy, 0), img.height() - 1)
            total_count += 1
            if _luma(img, px, py) <= DARK_SPHERE_LUMA_MAX:
                dark_count += 1

    return "dead" if (dark_count / total_count) >= DEAD_BALL_MIN_DARK_FRACTION else "live"


def find_textured_spheres(img, existing):
    """
    Silver-ball candidates via local texture, not colour or edge-strength.
    Returns a list of (cx, cy, r) -- every candidate that passes, not just
    the first, since the course has two live victims and both need to be
    findable in the same frame.

    `existing` is the list of (kind, x, y, r) balls pass 1 (Hough circles)
    already found. Any texture candidate whose centre falls inside one of
    those is skipped outright, before the roundness/fill checks even run
    -- pass 1's circle fit is the trustworthy one, and a stray textured
    speck inside an already-identified ball's face (a surface mark, sensor
    noise, a compression artefact) shouldn't get to relitigate what that
    ball already correctly is. The same reasoning applies between texture
    candidates themselves: one physical ball whose texture fragments into
    more than one passing blob shouldn't get reported twice.

    The balls are pressed, scrunched foil -- uniformly dull, but with
    enough fine surface texture that no single absolute brightness or
    colour value reliably separates them from the white floor (that's why
    CIRCLE_THRESHOLD chasing didn't work: it's an edge-strength measure,
    and this surface's boundary edge is genuinely weak). What it does
    have is local *variance* the floor doesn't: a flat white background
    reads near-uniform at any exposure, while the foil's micro-facets
    make brightness bounce around within any small neighbourhood on the
    ball's surface.

    image.stdev() -- a direct local-variance filter -- doesn't exist on
    this firmware (AttributeError, not a calling-convention mismatch like
    other surprises here). Built on filters confirmed to actually exist
    instead: blur a copy with mean(), then difference() it against the
    unblurred version. What's left over after subtracting a blurred copy
    from the original *is* the local high-frequency detail -- flat
    background cancels out to near zero, the foil's texture leaves a
    visible residue. After thresholding, dilate() grows and merges nearby
    islands of "textured" pixels, needed because uneven lighting across
    the foil's facets can mean only part of a ball clears TEXTURE_MIN in
    any given frame. Runs on a separate grayscale copy so it never
    touches what's actually drawn on screen.
    """
    texture = img.copy()
    texture = texture.to_grayscale()
    blurred = texture.copy()
    blurred.mean(TEXTURE_WINDOW)
    texture.difference(blurred)
    texture.binary([(TEXTURE_MIN, TEXTURE_MAX)])
    texture.dilate(TEXTURE_DILATE)

    blobs = texture.find_blobs(
        [(128, 255)],  # binary() already reduced this to on/off
        pixels_threshold=TEXTURE_MIN_PIXELS,
        area_threshold=TEXTURE_MIN_PIXELS,
        merge=True,
    )

    accepted = []
    for b in blobs:
        radius = (b.w + b.h) // 4

        inside_existing = any(
            ((b.cx - ex) ** 2 + (b.cy - ey) ** 2) ** 0.5 < er
            for (_, ex, ey, er) in existing
        )
        # A glint can be small and round enough to pass roundness/fill
        # just as well as real ball texture -- but a real ball's blob
        # should be roughly ball-sized, using the same radius bounds the
        # Hough pass uses. A lone highlight dot, even after dilation,
        # generally won't happen to land in that exact size range.
        wrong_size = not (SPHERE_R_MIN_PX <= radius <= SPHERE_R_MAX_PX)
        # And the same reasoning as inside_existing, but against
        # candidates THIS pass has already accepted -- one physical ball
        # whose texture fragmented into two separate passing blobs
        # shouldn't be reported as two balls.
        overlaps_accepted = any(
            ((b.cx - ax) ** 2 + (b.cy - ay) ** 2) ** 0.5 < (radius + ar)
            for (ax, ay, ar) in accepted
        )

        if inside_existing or wrong_size or overlaps_accepted:
            if DEBUG_TEXTURE_CANDIDATES:
                # Grey = excluded outright before the roundness/fill
                # checks even run -- inside a ball pass 1 already found,
                # the wrong size to plausibly be a ball, or a duplicate
                # of a ball this same pass already accepted.
                img.draw_rectangle((b.x, b.y, b.w, b.h), color=GREY, thickness=1)
            continue

        roundness = abs(b.w - b.h) / max(b.w, b.h)
        fill = b.pixels / (b.w * b.h)
        passed = roundness <= ROUNDNESS_TOLERANCE and fill >= TEXTURE_MIN_FILL

        if DEBUG_TEXTURE_CANDIDATES:
            # Every remaining candidate, pass or fail -- yellow = passed
            # both checks, cyan = found but rejected by one of them.
            debug_colour = YELLOW if passed else CYAN
            img.draw_rectangle((b.x, b.y, b.w, b.h), color=debug_colour, thickness=1)
            img.draw_string((b.x, max(0, b.y - 10)),
                             "r=%.2f f=%.2f" % (roundness, fill), color=debug_colour)

        if passed:
            accepted.append((b.cx, b.cy, radius))

    return accepted


def _focal_px(img):
    """
    Focal length in pixels, derived from CAMERA_VFOV_DEG and the current
    frame height. Shared by the distance and bearing calculations below:
    a sensor with square pixels (the normal case for a machine-vision
    sensor like this one) has one physical focal length that converts to
    the same pixel-focal-length on both axes, so this same value is valid
    for the horizontal axis too -- no separate horizontal-FOV constant
    needed.
    """
    half_h_px = img.height() / 2
    return half_h_px / math.tan(math.radians(CAMERA_VFOV_DEG / 2))


def estimate_distance_mm(img, cy, r):
    """
    Ground-plane distance estimate to a detected sphere, in mm.

    Projects from the ball's BASE, not its centroid: `cy` is the sphere's
    centre row, so `cy + r` is the bottom of its bounding circle -- the
    point actually touching the floor. Using the centroid instead would
    put every estimate systematically short by roughly one ball radius'
    worth of the true ground-plane geometry, since the centroid sits one
    radius above the floor, not on it.

    Returns None if the projected ray points above the horizon (shouldn't
    happen for anything actually in frame on the floor, but a divide-by-
    zero/negative-tangent guard costs nothing).
    """
    base_row = cy + r
    focal_px = _focal_px(img)
    half_h_px = img.height() / 2

    dy_px = base_row - half_h_px  # positive = below frame centre
    ray_angle_deg = CAMERA_TILT_DEG + math.degrees(math.atan(dy_px / focal_px))

    if ray_angle_deg <= 0:
        return None

    return CAMERA_HEIGHT_MM / math.tan(math.radians(ray_angle_deg))


def estimate_bearing_deg(img, cx):
    """
    Horizontal angle of a detected object from the centre of the frame,
    in degrees. Positive = right of centre, negative = left -- this is
    the "bearing" design.md's ZONE mode reply format calls for (Sec3).
    """
    half_w_px = img.width() / 2
    dx_px = cx - half_w_px
    return math.degrees(math.atan(dx_px / _focal_px(img)))


def find_spheres(img):
    """
    Ball detection, two independent passes.

    Pass 1 -- shape (Hough circles), confirmed working for the dead
    (black) ball: found as a single, sharply-defined circle, correctly
    classified. Left exactly as validated; lowering CIRCLE_THRESHOLD to
    chase the silver balls previously made this pass unstable (multiple
    overlapping, sometimes misclassified duplicate circles) for no gain
    on the silver balls, so don't retune it here again.

    Pass 2 -- texture (find_textured_spheres() above), aimed specifically
    at the silver balls, which pass 1 doesn't catch. It's told what pass 1
    already found and excludes anything inside those circles itself, so
    the same ball can't get double-counted between the two methods.
    Returns every candidate it accepts, not just one, since the course
    has two live victims.
    """
    circles = img.find_circles(
        threshold=CIRCLE_THRESHOLD,
        x_stride=2, y_stride=2,
        r_min=SPHERE_R_MIN_PX, r_max=SPHERE_R_MAX_PX, r_step=2,
    )

    found = []
    for c in circles:
        kind = classify_sphere(img, c.x, c.y, c.r)
        dist_mm = estimate_distance_mm(img, c.y, c.r)
        bearing_deg = estimate_bearing_deg(img, c.x)
        colour = TEXT_WHITE if kind == "dead" else MAGENTA
        img.draw_circle((c.x, c.y, c.r), color=colour, thickness=2)
        img.draw_string((c.x - c.r, c.y - c.r - 12), kind, color=colour)
        img.draw_string((c.x - c.r, c.y - c.r - 22),
                         "%s %.0fdeg" % ("%dmm" % dist_mm if dist_mm is not None else "?", bearing_deg),
                         color=colour)
        found.append((kind, c.x, c.y, c.r, dist_mm, bearing_deg))

    existing_for_texture_pass = [(k, x, y, r) for (k, x, y, r, _, _) in found]
    for (tx, ty, tr) in find_textured_spheres(img, existing_for_texture_pass):
        kind = classify_sphere(img, tx, ty, tr)
        dist_mm = estimate_distance_mm(img, ty, tr)
        bearing_deg = estimate_bearing_deg(img, tx)
        colour = TEXT_WHITE if kind == "dead" else MAGENTA
        img.draw_circle((tx, ty, tr), color=colour, thickness=2)
        img.draw_string((tx - tr, ty - tr - 12), kind, color=colour)
        img.draw_string((tx - tr, ty - tr - 22),
                         "%s %.0fdeg" % ("%dmm" % dist_mm if dist_mm is not None else "?", bearing_deg),
                         color=colour)
        found.append((kind, tx, ty, tr, dist_mm, bearing_deg))

    return found


def find_triangle(img, threshold, colour, label):
    """One evacuation-point corner -- a plain colour blob, gated big."""
    biggest, _ = scan(img, threshold, min_pixels=ZONE_MIN_BLOB_PIXELS)
    if biggest is None:
        return None

    img.draw_rectangle((biggest.x, biggest.y, biggest.w, biggest.h), color=colour, thickness=2)
    img.draw_string((biggest.x, biggest.y - 12), label, color=colour)
    return biggest.cx, biggest.cy


# --- apply startup mode configuration ----------------------------------------


def _set_brightness(fraction):
    sensor.set_auto_exposure(False, exposure_us=int(_auto_exposure_us * fraction))


if ZONE_MODE_ENABLED:
    _win_w = int(FULL_W * ZONE_WINDOW_W_FRAC)
    _win_h = int(FULL_H * ZONE_WINDOW_H_FRAC)
    _win_x = (FULL_W - _win_w) // 2
    _win_y = (FULL_H - _win_h) // 2
    sensor.set_windowing((_win_x, _win_y, _win_w, _win_h))
    _set_brightness(ZONE_BRIGHTNESS_FRACTION)
else:
    _set_brightness(LINE_BRIGHTNESS_FRACTION)

# --- main loop -------------------------------------------------------------

tracking = DEFAULT_POLARITY  # LINE mode's persisted polarity guess

while True:
    clock.tick()
    img = sensor.snapshot()

    if ZONE_MODE_ENABLED:
        # --- ZONE mode ------------------------------------------------------

        spheres = find_spheres(img)
        green_pos = find_triangle(img, GREEN_THRESHOLD, GREEN, "GREEN")
        red_pos = find_triangle(img, RED_THRESHOLD, RED, "RED")

        img.draw_string((4, 4), "ZONE  spheres=%d green=%s red=%s" %
                         (len(spheres), "y" if green_pos else "n", "y" if red_pos else "n"),
                         color=TEXT_WHITE)
        img.draw_string((4, img.height() - 12), "fps=%.1f" % clock.fps(), color=TEXT_WHITE)

        print("ZONE spheres:", spheres, "green:", green_pos, "red:", red_pos)

    else:
        # --- LINE mode --------------------------------------------------

        # Dead-ahead reference, drawn first so the line overlay sits on
        # top of it rather than under it.
        cx = img.width() // 2
        img.draw_line((cx, 0, cx, img.height()), color=YELLOW, thickness=1)

        # Both polarities scanned every frame -- background classification
        # needs both coverages regardless of which one ends up tracked.
        blk_blob, blk_coverage = scan(img, BLACK_THRESHOLD)
        wht_blob, wht_coverage = scan(img, WHITE_THRESHOLD)
        background = classify_background(blk_coverage, wht_coverage)

        # The line to track is always the opposite colour to the
        # background (design.md Sec4's F/W split). "unclear" leaves
        # `tracking` wherever it already was.
        if background == "black":
            tracking = "white"
        elif background == "white":
            tracking = "black"

        if tracking == "black":
            biggest, colour, label, coverage = blk_blob, RED, "BLK", blk_coverage
        else:
            biggest, colour, label, coverage = wht_blob, BLUE, "WHT", wht_coverage

        ahead, angle, length = draw_line(img, biggest, colour)

        img.draw_string((4, 4), "%s ahead=%d angle=%d len=%d cov=%d%%" %
                         (label, ahead, angle, length, coverage), color=colour)
        img.draw_string((4, 16), "background=%s (blk=%d%% wht=%d%%)" %
                         (background, blk_coverage, wht_coverage), color=TEXT_WHITE)
        img.draw_string((4, img.height() - 12), "fps=%.1f" % clock.fps(), color=TEXT_WHITE)

        print(label, ahead, angle, length, coverage, "| background:", background)

# --- next steps -----------------------------------------------------------
#
# Automatic switching: design.md Sec6 (state B) triggers ZONE mode from a
# real white-background/no-line-found condition, not a hand-flipped
# constant -- wiring that up (with its own frame-count or distance-based
# debounce) is the natural next step once both modes are trusted enough
# to hand control to each other automatically.
#
# Hub link: LINE mode's scan()/classify_background() and ZONE mode's
# find_spheres()/find_triangle()/estimate_distance_mm()/
# estimate_bearing_deg() already compute exactly what design.md's LINE
# and ZONE reply formats need (Sec3) -- drop the drawing calls, cache
# results per frame, and wire commands up as PUPRemote channels. That
# needs pupremote.py + lpf2.py (github.com/antonvh/PUPRemote) copied onto
# the camera's own storage first -- that's the "OPENMV" import from
# earlier in this project's history, and it isn't there yet.
