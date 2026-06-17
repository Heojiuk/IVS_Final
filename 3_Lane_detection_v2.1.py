#!/usr/bin/env python3
"""
3_Lane_detection_v2.1.py  -- Perception module (heading + curvature added)

Changes vs v2:
  D1. center_fit               : ego-lane center polynomial = average of the two
                                 bounding marking fits (yellow + one green).
  D2. compute_heading_curvature: heading_rad + curvature_1pm from the center fit
                                 tangent/curvature at the car (CONTROL_Y).
  D3. build_lane_data_v2       : now also fills lane_heading_rad / lane_curvature_1pm.
  D4. draw_hud                 : prints heading in BOTH rad (bus value) and deg,
                                 plus curvature (1/m).

Notes:
  - heading depends only on the BEV x/y pixel-scale RATIO (M_PER_PX_X / M_PER_PX_Y),
    not on absolute scale -> correct even before measuring lane width, IF the BEV
    is isotropic. curvature (1/m) DOES need the absolute scale -> measure
    REAL_LANE_WIDTH_M on the track.
  - sign of heading/offset and LEFT/RIGHT must be verified on hardware (hflip+vflip).

Everything else is identical to v2.
"""

import cv2
import math
import numpy as np
from collections import deque
from picamera2 import Picamera2
from libcamera import Transform

# ============================================================
# Camera / BEV geometry  (keep in sync with 2_Apply_IPM.py)
# ============================================================
PREVIEW_SIZE     = (640, 360)
POINTS_REF_SIZE  = (1280, 720)
FRAME_RATE       = 40

SRC_POINTS = [
    [369, 325],
    [1129, 334],
    [860, 177],
    [501, 174],
]

WARP_W    = 400
WARP_H    = 600
MARGIN_X  = 100

# ============================================================
# Color thresholds (HSV)
# ============================================================
YELLOW_LOW  = (18, 80, 80)
YELLOW_HIGH = (35, 255, 255)
GREEN_LOW   = (40, 50, 50)
GREEN_HIGH  = (90, 255, 255)

# ============================================================
# Lane-detection tuning
# ============================================================
EGO_CENTER_X   = WARP_W // 2
LANE_WIDTH_PX  = 150
NEAR_FIELD_FRAC = 0.5
CONTROL_Y      = int(WARP_H * 0.90)

N_WINDOWS      = 10
WIN_MARGIN     = 45
MIN_PIX        = 30
MIN_LANE_PIX   = 150
MIN_BASE_SUM   = 255 * 4
PEAK_MIN_DIST  = 50

MAX_OFFSET_PX  = LANE_WIDTH_PX
EMA_ALPHA      = 0.4
OVERLAY_ALPHA  = 0.45

# ============================================================
# [v2.1] Metric conversion (for heading scale + curvature in 1/m)
# ============================================================
REAL_LANE_WIDTH_M = 0.30                          # MEASURE on track! real lane width (m)
M_PER_PX_X = REAL_LANE_WIDTH_M / LANE_WIDTH_PX     # horizontal scale (m/px)
M_PER_PX_Y = M_PER_PX_X                            # vertical scale; set from BEV depth if anisotropic

# ============================================================
# [B1] Ego-lane vote rows  (fraction of WARP_H, bottom->top)
# ============================================================
VOTE_Y_FRACS = [0.92, 0.82, 0.72, 0.62, 0.52]

# ============================================================
# [C1/C3] FitEMA -- smooth polyfit coefficients over time
# ============================================================
FIT_EMA_ALPHA = 0.30   # lower = smoother, higher = more responsive
MAX_MISS      = 8      # frames without detection before fit is discarded


class FitEMA:
    """Exponential moving average applied to numpy polyfit coefficient arrays."""

    def __init__(self, alpha: float = FIT_EMA_ALPHA, max_miss: int = MAX_MISS):
        self.alpha    = alpha
        self.max_miss = max_miss
        self.fit      = None   # current smoothed coefficients (ndarray or None)
        self._miss    = 0      # consecutive frames with no detection

    def update(self, new_fit):
        """
        Call once per frame.
        new_fit : ndarray (from np.polyfit) or None if marking not detected.
        Returns : smoothed fit (ndarray) or None if lost too long.
        """
        if new_fit is None:
            self._miss += 1
            if self._miss > self.max_miss:
                self.fit = None   # give up -- marking truly gone
            return self.fit       # return last good fit (or None)

        self._miss = 0
        if self.fit is None:
            self.fit = new_fit.copy()
        else:
            self.fit = self.alpha * new_fit + (1.0 - self.alpha) * self.fit
        return self.fit


# one FitEMA instance per marking, shared across frames
_fit_ema: dict[str, FitEMA] = {
    k: FitEMA() for k in ["left_green", "yellow", "right_green"]
}

# ============================================================
# [B3] LaneStateSanity -- per-frame sanity guard
# ============================================================
class LaneStateSanity:
    """
    Guards against physically implausible jumps between consecutive frames.

    Rules:
      1. Lane width: EMA-tracked; >40 % single-frame change -> revert to EMA value.
      2. Lane offset: >60 px single-frame jump -> revert to previous value.
      3. Ego lane: majority of last 10 frames wins (single-frame flip suppressed).
    """

    MAX_WIDTH_JUMP_FRAC = 0.40   # 40 % width change threshold
    MAX_OFFSET_JUMP_PX  = 60     # px
    WIDTH_EMA_ALPHA     = 0.15
    EGO_HISTORY_LEN     = 10

    def __init__(self):
        self._prev        = {}
        self._width_ema   = None
        self._ego_history = deque(maxlen=self.EGO_HISTORY_LEN)

    def check(self, data: dict) -> dict:
        # -- 1. Lane width --------------------------------------
        w = data.get("lane_width_px")
        if w is not None:
            if self._width_ema is None:
                self._width_ema = w
            else:
                ratio = w / self._width_ema
                lo, hi = 1 - self.MAX_WIDTH_JUMP_FRAC, 1 + self.MAX_WIDTH_JUMP_FRAC
                if lo < ratio < hi:
                    self._width_ema = (self.WIDTH_EMA_ALPHA * w
                                       + (1 - self.WIDTH_EMA_ALPHA) * self._width_ema)
                else:
                    # revert both width and center (they come from the same pair)
                    data["lane_width_px"]      = self._width_ema
                    data["ego_lane_center_px"] = self._prev.get(
                        "ego_lane_center_px", data.get("ego_lane_center_px"))

        # -- 2. Offset jump -------------------------------------
        off      = data.get("lane_offset_px")
        prev_off = self._prev.get("lane_offset_px")
        if off is not None and prev_off is not None:
            if abs(off - prev_off) > self.MAX_OFFSET_JUMP_PX:
                data["lane_offset_px"]   = prev_off
                data["lane_offset_norm"] = self._prev.get(
                    "lane_offset_norm", data.get("lane_offset_norm"))

        # -- 3. Ego lane majority vote --------------------------
        if data.get("ego_lane") is not None:
            self._ego_history.append(data["ego_lane"])
        if len(self._ego_history) >= 5:
            left_n  = self._ego_history.count("LEFT")
            right_n = self._ego_history.count("RIGHT")
            data["ego_lane"]      = "LEFT" if left_n > right_n else "RIGHT"
            data["adjacent_lane"] = "RIGHT" if data["ego_lane"] == "LEFT" else "LEFT"

        # snapshot non-None values for next frame
        self._prev = {k: v for k, v in data.items() if v is not None}
        return data


_sanity = LaneStateSanity()

# ============================================================
# Original helper functions  (unchanged from v2)
# ============================================================

def make_transform(src_points):
    src = np.float32(src_points)
    dst = np.float32([
        [MARGIN_X,          WARP_H],
        [WARP_W - MARGIN_X, WARP_H],
        [WARP_W - MARGIN_X, 0],
        [MARGIN_X,          0],
    ])
    return cv2.getPerspectiveTransform(src, dst)


def color_masks(bev_bgr):
    hsv    = cv2.cvtColor(bev_bgr, cv2.COLOR_BGR2HSV)
    yellow = cv2.inRange(hsv, np.array(YELLOW_LOW),  np.array(YELLOW_HIGH))
    green  = cv2.inRange(hsv, np.array(GREEN_LOW),   np.array(GREEN_HIGH))
    kernel = np.ones((3, 3), np.uint8)
    yellow = cv2.morphologyEx(yellow, cv2.MORPH_OPEN, kernel)
    green  = cv2.morphologyEx(green,  cv2.MORPH_OPEN, kernel)
    return yellow, green


def find_bases(mask, max_n):
    y0   = int(WARP_H * (1 - NEAR_FIELD_FRAC))
    hist = np.sum(mask[y0:, :], axis=0).astype(float)
    bases = []
    for _ in range(max_n):
        x = int(np.argmax(hist))
        if hist[x] < MIN_BASE_SUM:
            break
        bases.append(x)
        lo, hi = max(0, x - PEAK_MIN_DIST), min(len(hist), x + PEAK_MIN_DIST)
        hist[lo:hi] = 0
    return sorted(bases)


def trace_marking(mask, base_x):
    h, w = mask.shape
    window_height = h // N_WINDOWS
    nz = mask.nonzero()
    ny, nx = np.array(nz[0]), np.array(nz[1])

    x_current = base_x
    inds = []
    for win in range(N_WINDOWS):
        y_low  = h - (win + 1) * window_height
        y_high = h - win * window_height
        x_low  = x_current - WIN_MARGIN
        x_high = x_current + WIN_MARGIN
        good   = ((ny >= y_low) & (ny < y_high) &
                  (nx >= x_low) & (nx < x_high)).nonzero()[0]
        inds.append(good)
        if len(good) > MIN_PIX:
            x_current = int(np.mean(nx[good]))

    inds = np.concatenate(inds) if inds else np.array([], int)
    if len(inds) > MIN_LANE_PIX:
        fit = np.polyfit(ny[inds], nx[inds], 2)
        return fit, (nx[inds], ny[inds])
    return None, (np.array([]), np.array([]))


def poly_x(fit, y):
    return fit[0] * y * y + fit[1] * y + fit[2]


def detect_markings(yellow_mask, green_mask):
    """Original v1 detector -- returns raw (un-smoothed) fits."""
    markings = {"left_green": None, "yellow": None, "right_green": None}

    y_bases = find_bases(yellow_mask, max_n=1)
    if y_bases:
        fit, _ = trace_marking(yellow_mask, y_bases[0])
        if fit is not None:
            markings["yellow"] = fit

    g_bases = find_bases(green_mask, max_n=2)
    green_fits = []
    for gb in g_bases:
        fit, _ = trace_marking(green_mask, gb)
        if fit is not None:
            green_fits.append(fit)

    if markings["yellow"] is not None:
        yx    = poly_x(markings["yellow"], CONTROL_Y)
        left  = [f for f in green_fits if poly_x(f, CONTROL_Y) < yx]
        right = [f for f in green_fits if poly_x(f, CONTROL_Y) >= yx]
        if left:
            markings["left_green"]  = max(left,  key=lambda f: poly_x(f, CONTROL_Y))
        if right:
            markings["right_green"] = min(right, key=lambda f: poly_x(f, CONTROL_Y))
    elif len(green_fits) >= 1:
        green_fits.sort(key=lambda f: poly_x(f, CONTROL_Y))
        markings["left_green"] = green_fits[0]
        if len(green_fits) >= 2:
            markings["right_green"] = green_fits[-1]

    return markings


# ============================================================
# [C2] detect_markings_v2  -- raw detect + FitEMA per marking
# ============================================================

def detect_markings_v2(yellow_mask, green_mask) -> dict:
    """
    Drop-in replacement for detect_markings().
    Applies FitEMA to each marking so fits stay smooth across frames.
    """
    raw = detect_markings(yellow_mask, green_mask)
    smoothed = {}
    for key in ["left_green", "yellow", "right_green"]:
        smoothed[key] = _fit_ema[key].update(raw[key])
    return smoothed


# ============================================================
# Original build_lane_data  (unchanged)
# ============================================================

def build_lane_data(markings) -> dict:
    data = {
        "ego_lane":           None,
        "adjacent_lane":      None,
        "num_lanes_detected": 0,
        "lane_offset_px":     None,
        "lane_offset_norm":   None,
        "lane_width_px":      None,
        "ego_lane_center_px": None,
        "markings": {k: (markings[k] is not None) for k in markings},
    }

    yellow = markings["yellow"]
    lg     = markings["left_green"]
    rg     = markings["right_green"]

    yx  = poly_x(yellow, CONTROL_Y) if yellow is not None else None
    lgx = poly_x(lg,     CONTROL_Y) if lg     is not None else None
    rgx = poly_x(rg,     CONTROL_Y) if rg     is not None else None

    present = sum(v is not None for v in (lgx, yx, rgx))
    data["num_lanes_detected"] = max(0, present - 1)

    if yx is None:
        return data

    if EGO_CENTER_X < yx:
        data["ego_lane"]      = "LEFT"
        data["adjacent_lane"] = "RIGHT"
        left_b  = lgx if lgx is not None else yx - LANE_WIDTH_PX
        right_b = yx
    else:
        data["ego_lane"]      = "RIGHT"
        data["adjacent_lane"] = "LEFT"
        left_b  = yx
        right_b = rgx if rgx is not None else yx + LANE_WIDTH_PX

    lane_center = (left_b + right_b) / 2.0
    width       = abs(right_b - left_b)
    offset      = float(np.clip(lane_center - EGO_CENTER_X,
                                -MAX_OFFSET_PX, MAX_OFFSET_PX))

    data["lane_width_px"]      = float(width)
    data["ego_lane_center_px"] = float(lane_center)
    data["lane_offset_px"]     = offset
    data["lane_offset_norm"]   = float(np.clip(offset / (LANE_WIDTH_PX / 2.0), -1, 1))
    return data


# ============================================================
# [B1] classify_ego_lane_robust  -- 5-row majority vote
# ============================================================

def classify_ego_lane_robust(markings) -> tuple[str | None, str | None]:
    """
    Sample the yellow fit at VOTE_Y_FRACS rows and take a majority vote.
    Returns (ego_lane, adjacent_lane) or (None, None) if yellow is absent.
    """
    if markings["yellow"] is None:
        return None, None

    votes = {"LEFT": 0, "RIGHT": 0}
    for frac in VOTE_Y_FRACS:
        y   = int(WARP_H * frac)
        yx  = poly_x(markings["yellow"], y)
        key = "LEFT" if EGO_CENTER_X < yx else "RIGHT"
        votes[key] += 1

    ego = "LEFT" if votes["LEFT"] > votes["RIGHT"] else "RIGHT"
    adj = "RIGHT" if ego == "LEFT" else "LEFT"
    return ego, adj


# ============================================================
# [D1] center_fit  -- ego-lane center polynomial
# ============================================================

def center_fit(markings, ego_lane):
    """
    Lane-center polynomial = average of the two markings bounding the ego lane.
    Polyfit coeffs are linear, so averaging coeffs == averaging the curves.
    Falls back to the yellow fit alone if a boundary is missing (parallel assumption).
    Returns ndarray of coeffs [A, B, C] or None.
    """
    y, lg, rg = markings["yellow"], markings["left_green"], markings["right_green"]
    if y is None:
        return None
    if ego_lane == "LEFT" and lg is not None:
        return (y + lg) / 2.0
    if ego_lane == "RIGHT" and rg is not None:
        return (y + rg) / 2.0
    return y   # one boundary missing -> use yellow slope (lanes ~parallel)


# ============================================================
# [D2] compute_heading_curvature  -- from center fit at the car
# ============================================================

def compute_heading_curvature(markings, ego_lane):
    """
    heading_rad : angle between lane-center direction and vehicle forward axis
                  (BEV vertical). 0 = aligned. atan(dx/dy) at the car (CONTROL_Y).
    curvature_1pm : lane curvature in 1/m (isotropic-BEV approximation).
    Returns (heading_rad, curvature_1pm) or (None, None) if no center fit.
    """
    fit = center_fit(markings, ego_lane)
    if fit is None:
        return None, None

    A, B  = fit[0], fit[1]
    y     = CONTROL_Y
    slope = 2.0 * A * y + B                       # dx/dy at car (pixels)

    heading_rad   = float(math.atan(slope * (M_PER_PX_X / M_PER_PX_Y)))
    kappa_px      = (2.0 * A) / (1.0 + slope ** 2) ** 1.5   # 1/px (isotropic approx)
    curvature_1pm = float(kappa_px / M_PER_PX_X)
    return heading_rad, curvature_1pm


# ============================================================
# [B2] build_lane_data_v2  -- v1 + robust ego + sanity + [D3] heading/curvature
# ============================================================

def build_lane_data_v2(markings) -> dict:
    """
    1. Call original build_lane_data (geometry unchanged).
    2. Override ego_lane with robust 5-row vote result.
       If vote disagrees with geometry, recalculate lane boundaries
       so the published offset is consistent with the voted ego lane.
    3. Pass through LaneStateSanity.
    4. [D3] Fill heading_rad / curvature_1pm from the center fit.
    """
    data = build_lane_data(markings)

    # -- Step 1: robust ego vote ----------------------------
    ego_vote, adj_vote = classify_ego_lane_robust(markings)

    if ego_vote is not None and ego_vote != data.get("ego_lane"):
        # Vote disagrees -- recalculate geometry with the voted lane
        yellow = markings["yellow"]
        lg     = markings["left_green"]
        rg     = markings["right_green"]
        yx     = poly_x(yellow, CONTROL_Y)
        lgx    = poly_x(lg, CONTROL_Y) if lg is not None else None
        rgx    = poly_x(rg, CONTROL_Y) if rg is not None else None

        if ego_vote == "LEFT":
            left_b  = lgx if lgx is not None else yx - LANE_WIDTH_PX
            right_b = yx
        else:
            left_b  = yx
            right_b = rgx if rgx is not None else yx + LANE_WIDTH_PX

        lane_center = (left_b + right_b) / 2.0
        width       = abs(right_b - left_b)
        offset      = float(np.clip(lane_center - EGO_CENTER_X,
                                    -MAX_OFFSET_PX, MAX_OFFSET_PX))

        data["ego_lane"]           = ego_vote
        data["adjacent_lane"]      = adj_vote
        data["lane_width_px"]      = float(width)
        data["ego_lane_center_px"] = float(lane_center)
        data["lane_offset_px"]     = offset
        data["lane_offset_norm"]   = float(np.clip(offset / (LANE_WIDTH_PX / 2.0), -1, 1))

    elif ego_vote is not None:
        # Vote agrees -- just make sure the fields are set
        data["ego_lane"]      = ego_vote
        data["adjacent_lane"] = adj_vote

    # -- Step 2: sanity guard -------------------------------
    data = _sanity.check(data)

    # -- Step 3 [D3]: heading + curvature (uses final ego_lane) ----
    head, curv = compute_heading_curvature(markings, data.get("ego_lane"))
    data["lane_heading_rad"]   = head
    data["lane_curvature_1pm"] = curv

    return data


# ============================================================
# Visualization helpers  ([D4] HUD prints heading rad+deg, curvature)
# ============================================================

def draw_overlay(overlay, markings, data):
    ploty  = np.linspace(0, WARP_H - 1, WARP_H)
    colors = {
        "left_green":  (0, 255, 0),
        "right_green": (0, 255, 0),
        "yellow":      (0, 255, 255),
    }
    for key, color in colors.items():
        fit = markings[key]
        if fit is not None:
            xs  = poly_x(fit, ploty)
            pts = np.int32(np.column_stack((xs, ploty)))
            cv2.polylines(overlay, [pts], False, color, 3)

    # Reference line = vehicle forward axis (BEV vertical). heading is measured
    # as the angle of the center line relative to THIS line.
    cv2.line(overlay, (EGO_CENTER_X, WARP_H - 1), (EGO_CENTER_X, 0),
             (255, 255, 255), 1)

    # [D5] Derived lane-center polynomial (the curve heading/curvature come from)
    cfit = center_fit(markings, data.get("ego_lane"))
    if cfit is not None:
        ploty = np.linspace(0, WARP_H - 1, WARP_H)
        xs    = poly_x(cfit, ploty)
        pts   = np.int32(np.column_stack((xs, ploty)))
        cv2.polylines(overlay, [pts], False, (255, 0, 255), 2)   # magenta = center fit

        # [D5] heading tangent at the car: the center direction we compare to vertical
        y0    = CONTROL_Y
        x0    = poly_x(cfit, y0)
        slope = 2.0 * cfit[0] * y0 + cfit[1]          # dx/dy at car
        dy    = 150                                    # draw forward (upward) 150 px
        x1    = x0 - slope * dy                         # forward = decreasing y
        cv2.line(overlay, (int(x0), int(y0)), (int(x1), int(y0 - dy)),
                 (0, 128, 255), 2)                      # orange = heading direction

    if data["ego_lane_center_px"] is not None:
        cx = int(data["ego_lane_center_px"])
        cv2.line(overlay, (cx, WARP_H - 1), (cx, WARP_H - 120), (255, 0, 255), 2)
        cv2.circle(overlay, (cx, CONTROL_Y),    6, (255, 0, 255), -1)
        cv2.circle(overlay, (EGO_CENTER_X, CONTROL_Y), 6, (255, 255, 255), -1)


def draw_hud(img, data):
    def put(text, row, color=(0, 0, 255)):
        cv2.putText(img, text, (10, 25 + 24 * row),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    ego  = data["ego_lane"] or "?"
    put(f"EGO LANE: {ego}   lanes={data['num_lanes_detected']}", 0)
    if data["lane_offset_px"] is None:
        put("offset: --", 1, (0, 0, 255))
    else:
        put(f"offset: {data['lane_offset_px']:+.0f}px "
            f"({data['lane_offset_norm']:+.2f})", 1)
    w = data["lane_width_px"]
    put(f"lane width: {w:.0f}px" if w else "lane width: --", 2)
    m    = data["markings"]
    seen = "".join([
        "L" if m["left_green"]  else "-",
        "Y" if m["yellow"]      else "-",
        "R" if m["right_green"] else "-",
    ])
    put(f"markings[L Y R]: {seen}", 3)

    # [D4] heading: bus value (rad) + human-readable (deg)
    h = data.get("lane_heading_rad")
    if h is None:
        put("heading: --", 4, (0, 0, 255))
    else:
        put(f"heading: {h:+.3f} rad ({math.degrees(h):+.1f} deg)", 4)

    # [D4] curvature (1/m)
    c = data.get("lane_curvature_1pm")
    if c is None:
        put("curvature: --", 5, (0, 0, 255))
    else:
        put(f"curvature: {c:+.3f} 1/m", 5)


# ============================================================
# main  -- two call-sites use the v2 functions
# ============================================================

def main():
    sx = PREVIEW_SIZE[0] / POINTS_REF_SIZE[0]
    sy = PREVIEW_SIZE[1] / POINTS_REF_SIZE[1]
    src_scaled = [[x * sx, y * sy] for x, y in SRC_POINTS]
    M = make_transform(src_scaled)

    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"size": PREVIEW_SIZE, "format": "YUV420"},
        raw={"size": (2304, 1296)},
        transform=Transform(hflip=1, vflip=1),
        controls={"FrameRate": FRAME_RATE},
        buffer_count=4,
    )
    picam2.configure(config)
    picam2.start()

    print("Lane detection v2.1 (Perception). Press ESC to quit.")
    print(f"  FitEMA alpha={FIT_EMA_ALPHA}, max_miss={MAX_MISS}")
    print(f"  Ego vote rows (frac): {VOTE_Y_FRACS}")
    print(f"  Scale: m_per_px_x={M_PER_PX_X:.5f} (REAL_LANE_WIDTH_M={REAL_LANE_WIDTH_M})")
    print(f"  Sanity: max_width_jump={LaneStateSanity.MAX_WIDTH_JUMP_FRAC*100:.0f}%,"
          f" max_offset_jump={LaneStateSanity.MAX_OFFSET_JUMP_PX}px,"
          f" ego_history={LaneStateSanity.EGO_HISTORY_LEN}f")

    smooth_offset = None
    try:
        while True:
            yuv = picam2.capture_array()
            bgr = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_I420)

            bev = cv2.warpPerspective(bgr, M, (WARP_W, WARP_H))   # unchanged
            yellow_mask, green_mask = color_masks(bev)

            markings = detect_markings_v2(yellow_mask, green_mask)
            data     = build_lane_data_v2(markings)

            if data["lane_offset_px"] is not None:
                o = data["lane_offset_px"]
                smooth_offset = (o if smooth_offset is None
                                 else EMA_ALPHA * o + (1 - EMA_ALPHA) * smooth_offset)
                data["lane_offset_smooth_px"] = float(smooth_offset)
            else:
                data["lane_offset_smooth_px"] = None

            # ---- `data` is the perception output for the bus ----
            #   lane_offset_m   = lane_offset_smooth_px * M_PER_PX_X
            #   lane_heading_rad = data["lane_heading_rad"]      (already rad)
            #   lane_curvature_1pm = data["lane_curvature_1pm"]  (already 1/m)

            overlay = bev.copy()
            draw_overlay(overlay, markings, data)
            vis = cv2.addWeighted(overlay, OVERLAY_ALPHA, bev, 1 - OVERLAY_ALPHA, 0)
            draw_hud(vis, data)

            masks_view = cv2.merge([np.zeros_like(green_mask),
                                    green_mask, yellow_mask])

            cv2.imshow('BEV Raw',        bev)
            cv2.imshow('Lane Detection', vis)
            cv2.imshow('Masks',          masks_view)

            if cv2.waitKey(1) & 0xFF == 27:
                break
    except KeyboardInterrupt:
        pass
    finally:
        picam2.stop()
        picam2.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
