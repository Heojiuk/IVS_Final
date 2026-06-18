#!/usr/bin/env python3
"""
lane_detection.py  -- 차선 인지 처리 모듈 (BEV+HSV, ex-3_Lane_detection_v2.3)

Changes vs v2.2:
  F1. center_fit (reconstruct) : build the ego-lane center from ANY visible markings +
                                 fixed lane width. Yellow missing? estimate it from the
                                 green(s). -> offset/heading survive yellow dropouts and
                                 lane-change crossings (no more all "--").
  F2. build_lane_data_v2       : offset/center now DRIVEN by center_fit (not hard-anchored
                                 on yellow), so it works whenever any marking is visible.
  F3. draw_overlay (clip)      : polylines stop at the frame edge -> no out-of-frame
                                 loop on sharp curves (the v2.2 magenta-loop bug).
  F4. CurvFilter               : reject blow-up curvature (radius < MIN) + light EMA.

Kept from v2.2: near-field ego detect, EgoLaneTracker hysteresis, heading/curvature,
                HUD rad+deg, center-line viz.

Notes:
  - Reconstruction assumes the green|lane|yellow|lane|green structure with fixed width.
  - sign of heading/offset and LEFT/RIGHT must be verified on hardware (hflip+vflip).
  - curvature (1/m) needs REAL_LANE_WIDTH_M measured on the track.
  - lane-change TARGET-lane following is added at integration time (read DriveCommand);
    standalone here always follows the current (ego-lock) lane.
"""

import cv2
import math
import numpy as np
from collections import deque

# ============================================================
# Camera / BEV geometry  (keep in sync with 2_Apply_IPM.py)
# ============================================================
PREVIEW_SIZE     = (640, 360)
POINTS_REF_SIZE  = (1280, 720)
FRAME_RATE       = 40

SRC_POINTS = [          # 광각(CM3 Wide) 재캘리 — POINTS_REF_SIZE(1280x720) 좌표계
    [1,    631],        # bottom-left
    [1234, 639],        # bottom-right
    [825,  404],        # top-right
    [429,  401],        # top-left
]

WARP_W    = 400
WARP_H    = 600
MARGIN_X  = 100

# ============================================================
# Color thresholds (HSV)
# ============================================================
YELLOW_LOW  = (25, 90, 130)   # H↑/V↑: 골판지 박스(H~22,V~117) 등 저색조·어두운 황색 오탐 배제
YELLOW_HIGH = (35, 255, 255)
GREEN_LOW   = (40, 70, 50)    # S↑: 저채도 회색 바닥 false-green 배제 (테이프 S 대부분 ≥78)
GREEN_HIGH  = (90, 255, 255)

# ============================================================
# Lane-detection tuning
# ============================================================
EGO_CENTER_X   = 201        # 차 중심선의 BEV 위치(px) — 캘리: 차 자로재서 정확히 중앙일 때 lane_center
                            #   (광각 재캘리 — 중앙 −1.5cm 편향 보정 207→201)
LANE_WIDTH_PX  = 100        # BEV 측정 한 차로폭(px) — 광각 재캘리 100px↔24cm
NEAR_FIELD_FRAC = 0.5
CONTROL_Y      = int(WARP_H * 0.97)   # 측정 기준행 — 차에 더 가깝게 (BEV 최하단 근처)

N_WINDOWS      = 10
WIN_MARGIN     = 45
MIN_PIX        = 30
MIN_LANE_PIX   = 150
MIN_BASE_SUM   = 255 * 4
PEAK_MIN_DIST  = 50

MAX_OFFSET_PX  = LANE_WIDTH_PX
OVERLAY_ALPHA  = 0.45

# ============================================================
# [v2.1] Metric conversion (for heading scale + curvature in 1/m)
# ============================================================
REAL_LANE_WIDTH_M = 0.24                          # 실측: 한 차로폭(차선 라인 포함) 0.24m
M_PER_PX_X = REAL_LANE_WIDTH_M / LANE_WIDTH_PX     # horizontal scale (m/px) = 0.24/100 = 0.0024
M_PER_PX_Y = M_PER_PX_X / 3.0                      # vertical scale — heading 캘리: 실측 θ vs heading로
                                                   #   비율 M_PER_PX_X/M_PER_PX_Y≈3.0 (BEV 세로 압축 보정)

# [F4] curvature gate: reject fits sharper than the track's min radius (blow-ups)
MIN_CURVE_RADIUS_M = 0.20                          # real min ~0.35m; 0.20 is a loose gate
MAX_CURVATURE_1PM  = 1.0 / MIN_CURVE_RADIUS_M      # |curvature| above this -> reject + hold
CURV_EMA_ALPHA     = 0.30                          # light smoothing on accepted curvature

# ============================================================
# [B1] Ego-lane vote rows  (fraction of WARP_H, bottom->top)  (kept as fallback)
# ============================================================
VOTE_Y_FRACS = [0.92, 0.82, 0.72, 0.62, 0.52]

# ============================================================
# [E1/E2] Near-field ego-lane detection (ORIGINAL image) + hysteresis
# ============================================================
NEAR_ROI_Y0_FRAC = 0.80    # bottom 20% of the ORIGINAL frame = ground in front of car
NEAR_MIN_SUM     = 255 * 5 # min column-sum to accept a yellow/green line in the ROI
SWITCH_FRAMES    = 8       # consecutive opposite reads required to switch ego lane
                           # (yaw glitch = a few frames -> ignored; lane change = sustained)

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
    Ego-lane majority vote over the last N frames (single-frame flip suppressed).

    Width/offset jump guards were removed: those values already come from
    FitEMA-smoothed fits, and the revert-to-previous guard could permanently
    stick on a sustained change (e.g. ego flip) instead of converging.
    """

    EGO_HISTORY_LEN = 10

    def __init__(self):
        self._ego_history = deque(maxlen=self.EGO_HISTORY_LEN)

    def check(self, data: dict) -> dict:
        # Ego lane majority vote
        if data.get("ego_lane") is not None:
            self._ego_history.append(data["ego_lane"])
        if len(self._ego_history) >= 5:
            left_n  = self._ego_history.count("LEFT")
            right_n = self._ego_history.count("RIGHT")
            data["ego_lane"]      = "LEFT" if left_n > right_n else "RIGHT"
            data["adjacent_lane"] = "RIGHT" if data["ego_lane"] == "LEFT" else "LEFT"
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
# [E1] detect_ego_lane_nearfield  -- ego lane from ORIGINAL image bottom ROI
# ============================================================

def detect_ego_lane_nearfield(bgr):
    """
    Decide ego lane from the ground right in front of the car (bottom ROI of the
    ORIGINAL, un-warped frame). Robust to yaw/curve: near-field lines have not yet
    swept to the other side. No polyfit -- just where the yellow line sits vs the car.

    Convention (same as BEV): yellow to the RIGHT of car -> LEFT lane,
                              yellow to the LEFT  of car -> RIGHT lane.
    Returns "LEFT" / "RIGHT" / None (yellow not seen near).
    """
    h, w = bgr.shape[:2]
    roi  = bgr[int(h * NEAR_ROI_Y0_FRAC):, :]          # bottom band = car front
    hsv  = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    ymask = cv2.inRange(hsv, np.array(YELLOW_LOW), np.array(YELLOW_HIGH))

    yhist = np.sum(ymask, axis=0).astype(float)        # column sums
    if yhist.max() < NEAR_MIN_SUM:
        return None                                    # no yellow near -> no reading
                                                       # (녹색은 2개라 모호 → 기준 안 씀)
    yellow_x = int(np.argmax(yhist))
    car_x    = w // 2                                  # camera optical axis ~ car center
    return "LEFT" if car_x < yellow_x else "RIGHT"


# ============================================================
# [E2] EgoLaneTracker  -- hold lane, switch only on sustained opposite reads
# ============================================================

class EgoLaneTracker:
    """
    Holds the current ego lane. A single yaw-glitch read does NOT change it;
    only SWITCH_FRAMES consecutive opposite reads (a real lane change) do.
    """

    def __init__(self, switch_frames: int = SWITCH_FRAMES):
        self.switch_frames = switch_frames
        self.ego        = None    # current locked lane
        self._candidate = None    # lane we're counting toward
        self._count     = 0       # consecutive opposite reads

    def update(self, observed):
        """observed: 'LEFT'/'RIGHT'/None (near-field read). Returns locked ego lane."""
        if observed is None:
            return self.ego                # no reading -> keep current
        if self.ego is None:
            self.ego = observed            # first lock
            self._candidate, self._count = None, 0
            return self.ego
        if observed == self.ego:
            self._candidate, self._count = None, 0   # confirms current -> reset counter
            return self.ego
        # observed disagrees with locked lane -> count toward a switch
        if observed == self._candidate:
            self._count += 1
        else:
            self._candidate, self._count = observed, 1
        if self._count >= self.switch_frames:
            self.ego = observed            # sustained -> commit lane change
            self._candidate, self._count = None, 0
        return self.ego


_ego_tracker = EgoLaneTracker()


# ============================================================
# [D1] center_fit  -- ego-lane center polynomial
# ============================================================

def _shift(fit, dx):
    """Return a copy of a polyfit shifted horizontally by dx px (constant term only)."""
    c = fit.copy(); c[2] += dx
    return c


def center_fit(markings, ego_lane):
    """
    (held) ego 차로의 중앙선 폴리핏. 구조: green | lane | yellow | lane | green.

    1) 노란선 보이면 — 노란선 기준 (가장 신뢰):
         RIGHT: (yellow + right_green)/2,  green 없으면 yellow + half
         LEFT : (left_green + yellow)/2,   green 없으면 yellow - half
    2) 노란선 없으면 (곡선서 시야 밖 등) — 차로는 못 바꿨으니 held ego 유효:
         RIGHT: 보이는 green 중 '오른쪽 경계'(max x) - half  → 중심은 그 왼쪽
         LEFT : 보이는 green 중 '왼쪽 경계'(min x)  + half  → 중심은 그 오른쪽
    Returns ndarray [A,B,C] or None.
    """
    y, lg, rg = markings["yellow"], markings["left_green"], markings["right_green"]
    half = LANE_WIDTH_PX / 2.0
    if ego_lane not in ("LEFT", "RIGHT"):
        return None

    # 1) 노란선 보이면 노란선 기준
    if y is not None:
        if ego_lane == "LEFT":
            return (lg + y) / 2.0 if lg is not None else _shift(y, -half)
        return (y + rg) / 2.0 if rg is not None else _shift(y, +half)

    # 2) 노란선 없음: held ego + 보이는 green 바깥경계에서 half 안쪽으로
    greens = [g for g in (lg, rg) if g is not None]
    if greens:
        if ego_lane == "RIGHT":
            return _shift(max(greens, key=lambda f: poly_x(f, CONTROL_Y)), -half)
        return _shift(min(greens, key=lambda f: poly_x(f, CONTROL_Y)), +half)

    return None


# ============================================================
# [F4] CurvFilter  -- reject blow-up curvature, then light EMA
# ============================================================

class CurvFilter:
    def __init__(self, max_curv=MAX_CURVATURE_1PM, alpha=CURV_EMA_ALPHA):
        self.max_curv = max_curv
        self.alpha    = alpha
        self.val      = None

    def update(self, c):
        if c is None:
            return self.val                          # no measurement -> hold
        if abs(c) > self.max_curv:
            return self.val                          # physically implausible -> reject + hold
        self.val = c if self.val is None else self.alpha * c + (1 - self.alpha) * self.val
        return self.val


_curv_filter = CurvFilter()


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

def build_lane_data_v2(markings, forced_ego=None) -> dict:
    """
    [F2] Center-fit-driven. ego_lane = forced_ego (near-field tracker) if given, else
    the BEV vote, else geometry. Offset/center/heading/curvature all come from the
    reconstructed lane-center fit -- so they survive a missing yellow line.
    """
    data = build_lane_data(markings)   # markings flags, num_lanes, fallback fields

    # -- Step 1: choose ego lane (near-field tracker wins; vote/geometry fallback) --
    if forced_ego is not None:
        ego = forced_ego
    else:
        ego = classify_ego_lane_robust(markings)[0] or data.get("ego_lane")

    # -- Step 2: geometry from the reconstructed center fit (yellow not required) --
    cfit = center_fit(markings, ego) if ego is not None else None
    if cfit is not None:
        cx     = poly_x(cfit, CONTROL_Y)
        offset = float(np.clip(cx - EGO_CENTER_X, -MAX_OFFSET_PX, MAX_OFFSET_PX))
        data["ego_lane"]           = ego
        data["adjacent_lane"]      = "RIGHT" if ego == "LEFT" else "LEFT"
        data["ego_lane_center_px"] = float(cx)
        data["lane_offset_px"]     = offset
        data["lane_offset_norm"]   = float(np.clip(offset / (LANE_WIDTH_PX / 2.0), -1, 1))
        if data["lane_width_px"] is None:
            data["lane_width_px"] = float(LANE_WIDTH_PX)   # reconstructed -> assume fixed

    # -- Step 3: sanity guard -------------------------------
    data = _sanity.check(data)

    # -- Step 3b: tracker is authoritative -- override sanity's ego vote --
    if forced_ego is not None:
        data["ego_lane"]      = forced_ego
        data["adjacent_lane"] = "RIGHT" if forced_ego == "LEFT" else "LEFT"

    # -- Step 4: heading + curvature from the center fit, with curvature gate+EMA --
    head, curv = compute_heading_curvature(markings, data.get("ego_lane"))
    data["lane_heading_rad"]   = head
    data["lane_curvature_1pm"] = _curv_filter.update(curv)

    return data


# ============================================================
# Visualization helpers  ([D4] HUD prints heading rad+deg, curvature)
# ============================================================

def _clip_poly_points(fit):
    """[F3] Points of x=fit(y) from the car (bottom) upward, stopping at the frame edge.
    Prevents drawing the parabola's out-of-frame loop on sharp curves."""
    ys = np.arange(WARP_H - 1, -1, -1)
    xs = poly_x(fit, ys)
    inside = (xs >= 0) & (xs < WARP_W)
    if not inside[0]:
        return None                                 # car-position point already out of frame
    cut = len(ys) if inside.all() else int(np.argmin(inside))  # stop at first out-of-frame
    if cut < 2:
        return None
    return np.int32(np.column_stack((xs[:cut], ys[:cut])))


def draw_overlay(overlay, markings, data):
    colors = {
        "left_green":  (0, 255, 0),
        "right_green": (0, 255, 0),
        "yellow":      (0, 255, 255),
    }
    for key, color in colors.items():
        fit = markings[key]
        if fit is not None:
            pts = _clip_poly_points(fit)
            if pts is not None:
                cv2.polylines(overlay, [pts], False, color, 3)

    # Reference line = vehicle forward axis (BEV vertical). heading is measured
    # as the angle of the center line relative to THIS line.
    cv2.line(overlay, (EGO_CENTER_X, WARP_H - 1), (EGO_CENTER_X, 0),
             (255, 255, 255), 1)

    # [D5] Derived lane-center polynomial (the curve heading/curvature come from)
    cfit = center_fit(markings, data.get("ego_lane"))
    if cfit is not None:
        pts = _clip_poly_points(cfit)                # [F3] clipped -> no out-of-frame loop
        if pts is not None:
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
        off_cm = data['lane_offset_px'] * M_PER_PX_X * 100.0   # 버스로 가는 값 (cm)
        put(f"offset: {data['lane_offset_px']:+.0f}px / "
            f"{off_cm:+.1f}cm ({data['lane_offset_norm']:+.2f})", 1)
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

    # [CAL] 차로중앙 px vs EGO_CENTER_X — 차 정확히 중앙일 때 이 lane_center 값을 EGO_CENTER_X로
    cx = data.get("ego_lane_center_px")
    cx_txt = "--" if cx is None else f"{cx:.0f}"
    put(f"lane_center: {cx_txt}px   EGO_CENTER_X: {EGO_CENTER_X}", 6)


# 카메라 루프/표시는 통합(sensing.camera_loop + lane_pipeline)이 담당.
# 이 모듈은 순수 처리 함수만 제공한다 (standalone main 없음).
