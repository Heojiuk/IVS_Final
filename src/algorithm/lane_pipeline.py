"""차선 처리 어댑터 — lane_detection 모듈을 써서 프레임 1장 → Scene 계약 5튜플.

  process(bgr)      -> (lane_valid, current_lane, lane_offset_m, lane_heading_rad, lane_curvature_1pm)
  process_view(bgr) -> (위 5튜플, BEV 시각화 이미지)  # 디버그용

lane_detection 의 모듈 상태(FitEMA·sanity·EgoLaneTracker·CurvFilter)는 싱글톤이라 프레임 간 유지됨.
"""
import cv2

from algorithm import lane_detection as _L

# BEV 변환행렬 1회 계산
_sx = _L.PREVIEW_SIZE[0] / _L.POINTS_REF_SIZE[0]
_sy = _L.PREVIEW_SIZE[1] / _L.POINTS_REF_SIZE[1]
_src = [[x * _sx, y * _sy] for x, y in _L.SRC_POINTS]
_M = _L.make_transform(_src)

# ego_lane(LEFT/RIGHT) → current_lane(1/2)  ⚠️ V2V·decision 차로번호 규약과 일치해야 함
_LANE_NUM = {"LEFT": 1, "RIGHT": 2}


def _compute(bgr):
    """공통 처리: BEV·마킹·data 반환 (process/process_view 가 공유)."""
    bev = cv2.warpPerspective(bgr, _M, (_L.WARP_W, _L.WARP_H))
    ymask, gmask = _L.color_masks(bev)
    markings = _L.detect_markings_v2(ymask, gmask)

    observed = _L.detect_ego_lane_nearfield(bgr)        # 원본 근거리에서 ego 판정
    ego = _L._ego_tracker.update(observed)              # 히스테리시스
    data = _L.build_lane_data_v2(markings, forced_ego=ego)
    return bev, markings, data


def _to_result(data):
    """data dict → Scene 계약 5튜플."""
    valid        = data["lane_offset_px"] is not None
    current_lane = _LANE_NUM.get(data.get("ego_lane"), 0)
    off_px       = data["lane_offset_px"]
    offset_m     = float(off_px * _L.M_PER_PX_X) if off_px is not None else 0.0
    heading      = data.get("lane_heading_rad")
    curv         = data.get("lane_curvature_1pm")
    return (valid, current_lane, offset_m,
            float(heading) if heading is not None else 0.0,
            float(curv) if curv is not None else 0.0)


def process(bgr):
    """프레임(BGR, lores) → Scene 계약 5튜플. update_lane(*process(bgr)) 로 사용."""
    _, _, data = _compute(bgr)
    return _to_result(data)


def process_view(bgr):
    """디버그용: (5튜플, BEV 시각화 이미지). 마젠타 중앙선 + heading(rad+deg) + curvature HUD."""
    bev, markings, data = _compute(bgr)
    overlay = bev.copy()
    _L.draw_overlay(overlay, markings, data)
    vis = cv2.addWeighted(overlay, _L.OVERLAY_ALPHA, bev, 1.0 - _L.OVERLAY_ALPHA, 0)
    _L.draw_hud(vis, data)                               # heading rad+deg, curvature 출력
    return _to_result(data), vis
