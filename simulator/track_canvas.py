"""TrackCanvas: tkinter Canvas widget for track + two-vehicle animation + track objects."""
import math
from dataclasses import dataclass

# ── 트랙 지오메트리 (Rounded Rectangle / Stadium) ─────────────────────
# 실제 트랙: 직선 구간 + 반원 곡선. 타원 아님.
#   전체 폭 2.5m, 전체 높이 2.05m
#   반원 반지름 R = 1.025m, 직선 절반 길이 S = 0.225m  (S*2 + R*2 = 2.5m)
_STRAIGHT = 0.225   # 직선 구간 절반 길이 (m)
_TRACK_LINES = [
    (1.225, '#22aa22', 3),   # 외곽 초록  (R = 1.025 + 0.20)
    (1.025, '#ddcc00', 3),   # 노란 중앙선 (R = 1.025)
    (0.825, '#22aa22', 3),   # 내곽 초록  (R = 1.025 - 0.20)
]
_CANVAS_W, _CANVAS_H = 900, 560
_SCALE_X = 200   # px/m 가로 (세로와 동일 — 반원 끝을 정원으로 유지, 실제 비율 반영)
_SCALE_Y = 200   # px/m 세로
_CX, _CY = _CANVAS_W // 2, _CANVAS_H // 2


def _stadium_pts(straight, radius, n=90):
    """Rounded rectangle 세계좌표 꼭짓점 리스트 (반시계방향).
    straight: 직선 절반 길이(m), radius: 반원 반지름(m)"""
    pts = []
    # 오른쪽 반원: 위(π/2) → 아래(-π/2)
    for i in range(n + 1):
        theta = math.pi / 2 - math.pi * i / n
        pts.append((straight + radius * math.cos(theta),
                    radius * math.sin(theta)))
    # 아래 직선: 오른→왼
    pts.append((-straight, -radius))
    # 왼쪽 반원: 아래(-π/2) → 왼쪽(-π) → 위(-3π/2). 왼쪽으로 볼록하도록 cos≤0 구간.
    for i in range(1, n + 1):
        theta = -math.pi / 2 - math.pi * i / n
        pts.append((-straight + radius * math.cos(theta),
                    radius * math.sin(theta)))
    # 위 직선: 왼→오른 (시작점으로 복귀, create_polygon이 자동 닫음)
    return pts


# 차선 중앙 반경 (1=내측: 황선1.025·내초록0.825 사이, 2=외측: 황선·외초록1.225 사이)
_LANE_CENTER_R = {1: 0.925, 2: 1.125}


def _seg_line(pts, x0, y0, x1, y1, step):
    """(x0,y0)→(x1,y1) 직선을 step 간격 분할해 추가 (끝점 제외 — 다음 세그먼트가 이음)."""
    n = max(1, int(round(math.hypot(x1 - x0, y1 - y0) / step)))
    for i in range(n):
        t = i / n
        pts.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t))


def _seg_arc(pts, cx, cy, r, a0, a1, step):
    """중심(cx,cy)·반경 r 호를 a0→a1 로 step 간격 분할해 추가 (끝점 제외)."""
    n = max(1, int(round(abs(a1 - a0) * r / step)))
    for i in range(n):
        a = a0 + (a1 - a0) * i / n
        pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))


def _stadium_loop_points(radius, step):
    """반경 radius 스타디움을 CCW(하단=오른쪽 진행)로 한 바퀴, step 간격 균등 점 리스트.
    하단직선→우반원→상단직선→좌반원 순서의 닫힌 루프."""
    S, pts = _STRAIGHT, []
    _seg_line(pts, -S, -radius,  S, -radius, step)                  # 하단 직선 (+x)
    _seg_arc (pts,  S, 0, radius, -math.pi / 2,  math.pi / 2,   step)  # 우 반원
    _seg_line(pts,  S,  radius, -S,  radius, step)                  # 상단 직선 (-x)
    _seg_arc (pts, -S, 0, radius,  math.pi / 2,  3 * math.pi / 2, step)  # 좌 반원
    return pts


def stadium_lane_path(start_wx, start_wy, lane, step=0.05):
    """차선(lane) 중앙선을 따라 CCW 한 바퀴 dense 경로. start 점에 가장 가까운 지점부터 시작."""
    radius = _LANE_CENTER_R.get(lane, _LANE_CENTER_R[1])
    loop = _stadium_loop_points(radius, step)
    i0 = min(range(len(loop)),
             key=lambda i: (loop[i][0] - start_wx) ** 2 + (loop[i][1] - start_wy) ** 2)
    return loop[i0:] + loop[:i0]


# ── 트랙 오브젝트 ─────────────────────────────────────────────────────
OBJ_OBSTACLE  = 'obstacle'   # 장애물  → front_clear=False
OBJ_STOP_LINE = 'stop_line'  # 정지선  → stop_signal=True
OBJ_TURN_LEFT = 'turn_left'  # 좌회전 이정표 → lane_curvature_1pm=+2.0

# 오브젝트 종류별 감지 반경 (m)
_DETECT_R = {
    OBJ_OBSTACLE:  0.40,
    OBJ_STOP_LINE: 0.18,
    OBJ_TURN_LEFT: 0.45,
}


@dataclass
class TrackObject:
    kind: str
    wx: float
    wy: float


# ── 좌표 변환 / 움직임 모델 ────────────────────────────────────────────
def world_to_screen(wx, wy, cx=_CX, cy=_CY, scale=None,
                    scale_x=_SCALE_X, scale_y=_SCALE_Y):
    """World (m, y-위) → screen (px, y-아래). scale 지정 시 균등 적용 (테스트 호환)."""
    if scale is not None:
        scale_x = scale_y = scale
    return cx + wx * scale_x, cy - wy * scale_y


def apply_motion_model(x, y, heading, throttle, steer, dt, k_v, k_w):
    """단순 비례 움직임 모델. (new_x, new_y, new_heading) 반환."""
    heading = heading + steer * k_w * dt
    x = x + throttle * k_v * math.cos(heading) * dt
    y = y + throttle * k_v * math.sin(heading) * dt
    return x, y, heading


def _local_to_screen(sx, sy, heading, lx, ly):
    c, s = math.cos(heading), math.sin(heading)
    return sx + lx * c - ly * s, sy - lx * s - ly * c


# ── 차량 그리기 ───────────────────────────────────────────────────────
def _draw_car(canvas, sx, sy, heading, body_color, wheel_color, tag, L=18, W=12,
              ghost=False, label=None):
    # ghost=True → stipple(망점)로 반투명 '유령' 효과 + 머리 위 라벨
    stip = 'gray50' if ghost else ''
    b_out = '#8899aa' if ghost else '#444'
    body = [(L, -W*0.6), (L, W*0.6), (-L, W), (-L, -W)]
    bpts = []
    for lx, ly in body:
        px, py = _local_to_screen(sx, sy, heading, lx, ly)
        bpts.extend([px, py])
    canvas.create_polygon(bpts, fill=body_color, outline=b_out, width=1.5,
                          stipple=stip, tags=tag)
    for wlx, wly in [(L*0.6, W*0.95), (L*0.6, -W*0.95),
                      (-L*0.6, W*0.95), (-L*0.6, -W*0.95)]:
        wcx, wcy = _local_to_screen(sx, sy, heading, wlx, wly)
        wpts = []
        for clx, cly in [(5, 2.5), (5, -2.5), (-5, -2.5), (-5, 2.5)]:
            px, py = _local_to_screen(wcx, wcy, heading, clx, cly)
            wpts.extend([px, py])
        canvas.create_polygon(wpts, fill=wheel_color, outline='#111', width=1,
                              stipple=stip, tags=tag)
    cmx, cmy = _local_to_screen(sx, sy, heading, L*0.55, 0)
    canvas.create_rectangle(cmx-3, cmy-3, cmx+3, cmy+3, fill='#111', outline='#666',
                            stipple=stip, tags=tag)
    fx, fy = _local_to_screen(sx, sy, heading, L+4, 0)
    canvas.create_line(sx, sy, fx, fy, fill='#222', width=1.5, tags=tag)
    if label:   # 차량 머리 위 문구
        canvas.create_text(sx, sy - (W + 16), text=label,
                           fill='#5566aa' if ghost else '#222',
                           font=('TkDefaultFont', 9, 'bold'), tags=tag)


# ── 오브젝트 그리기 ───────────────────────────────────────────────────
def _track_perpendicular(wx: float, wy: float):
    """Rounded rectangle 트랙 위 (wx,wy)에서 내향 법선(차선 폭 방향) 단위벡터 반환.
    직선 구간: 수직 방향, 곡선 구간: 원호 중심을 향하는 방향."""
    if abs(wx) <= _STRAIGHT:
        # 직선 구간: 법선은 순수 수직 (y=0 방향 = 내향)
        return (0.0, -1.0) if wy >= 0 else (0.0, 1.0)
    # 곡선 구간: 원호 중심 = (±STRAIGHT, 0)
    cx = _STRAIGHT if wx > 0 else -_STRAIGHT
    dx, dy = wx - cx, wy   # 중심→차량 (외향) 벡터
    length = math.hypot(dx, dy)
    if length < 1e-9:
        return (0.0, -1.0)
    return -dx / length, -dy / length   # 내향 = 차량→중심


def _draw_obj(canvas, obj: TrackObject, tag: str, highlight: bool = False):
    sx, sy = world_to_screen(obj.wx, obj.wy)
    outline_w = 3 if highlight else 2

    if obj.kind == OBJ_OBSTACLE:
        # 1차선 폭(20cm) 장애물 — 트랙 법선 방향으로 배치된 바
        nx, ny = _track_perpendicular(obj.wx, obj.wy)
        # 접선 방향 (깊이용)
        tx, ty = ny, -nx   # 법선을 다시 90° 돌리면 접선
        half_w = 0.10   # 반폭 10cm → 총 20cm
        depth  = 0.025  # 두께 2.5cm
        corners = [
            (obj.wx + nx*half_w + tx*depth, obj.wy + ny*half_w + ty*depth),
            (obj.wx - nx*half_w + tx*depth, obj.wy - ny*half_w + ty*depth),
            (obj.wx - nx*half_w - tx*depth, obj.wy - ny*half_w - ty*depth),
            (obj.wx + nx*half_w - tx*depth, obj.wy + ny*half_w - ty*depth),
        ]
        pts = []
        for cwx, cwy in corners:
            pts.extend(world_to_screen(cwx, cwy))
        fill = '#ff8800' if highlight else '#ff4422'
        canvas.create_polygon(pts, fill=fill, outline='#aa1100', width=outline_w, tags=tag)
        canvas.create_text(sx, sy, text='!', fill='white',
                           font=('TkDefaultFont', 9, 'bold'), tags=tag)

    elif obj.kind == OBJ_STOP_LINE:
        # 2차선 폭(40cm) 검은 정지선 — 트랙 법선 방향 실선
        nx, ny = _track_perpendicular(obj.wx, obj.wy)
        half_w = 0.20   # 반폭 20cm → 총 40cm
        p1 = world_to_screen(obj.wx + nx*half_w, obj.wy + ny*half_w)
        p2 = world_to_screen(obj.wx - nx*half_w, obj.wy - ny*half_w)
        lw = 5 if highlight else 4
        canvas.create_line(*p1, *p2, fill='#ffee00' if highlight else '#111111',
                           width=lw, capstyle='round', tags=tag)

    elif obj.kind == OBJ_TURN_LEFT:
        r = 16
        fill = '#44aaff' if highlight else '#1155cc'
        canvas.create_oval(sx-r, sy-r, sx+r, sy+r,
                           fill=fill, outline='#003388', width=outline_w, tags=tag)
        # 좌회전 화살표: 수직 줄기(위로) + 좌향 꺾임 + 화살촉
        canvas.create_line(sx+5, sy+10, sx+5, sy-2,
                           fill='white', width=3, capstyle='round', tags=tag)
        canvas.create_line(sx+5, sy-2, sx-7, sy-2,
                           fill='white', width=3, capstyle='round', tags=tag)
        canvas.create_polygon([sx-7, sy-2, sx-2, sy-7, sx-2, sy+3],
                               fill='white', outline='', tags=tag)


try:
    import tkinter as tk

    class TrackCanvas(tk.Canvas):
        """타원 트랙 + 차량 + 트랙 오브젝트(장애물/정지선/좌회전 이정표) 캔버스."""

        def __init__(self, parent, **kwargs):
            kwargs.setdefault('width', _CANVAS_W)
            kwargs.setdefault('height', _CANVAS_H)
            kwargs.setdefault('bg', '#e8e8e8')
            super().__init__(parent, **kwargs)

            self._pi_state  = None   # [wx, wy, heading]
            self._sim_state = None
            self._objects: list[TrackObject] = []
            self._place_mode: str | None = None
            self._on_mode_cancel = None   # 콜백: 배치 모드 취소 시 호출
            self._highlighted_objs: set[int] = set()   # 감지 중인 오브젝트 인덱스

            # 웨이포인트 기록 모드
            self._wp_record_mode = False
            self._wp_recording: list[tuple] = []   # 기록 중인 (wx, wy) 리스트
            self._on_wp_done = None   # 완료 콜백: fn(waypoints) 형태

            # 출발점 지정 모드 (좌클릭 1회로 출발점 선택)
            self._start_pick_mode = False
            self._on_start_pick = None   # 콜백: fn(wx, wy)

            self.k_v = 1.0
            self.k_w = 2.0

            # 전방 센서(카메라 FOV + 초음파 거리측정) 시각화 상태
            self._detect_range_m = 2.0     # 초음파 최대 측정거리 (m) — dist_front 슬라이더 연동
            self._detect_on      = False   # 감지(체크박스) 상태
            self._detect_show    = True    # 센서 표시 (차량 있으면 항상)
            self._us_phase       = 0.0     # 초음파 펄스 애니메이션 위상 (0~1)
            self._pi_ghost_label = None    # pi 차량 유령 라벨 (None=일반)
            self._sim_ghost_label = None   # sim 차량 유령 라벨 (None=일반)

            self._draw_track()
            self.bind('<Button-1>',        self._on_left_click)
            self.bind('<Button-3>',        self._on_right_click)
            self.bind('<ButtonRelease-1>', lambda _: None)   # 드래그 방지

        # ── 트랙 배경 ────────────────────────────────────────────────
        def _draw_track(self):
            self.delete('track')
            self.create_rectangle(0, 0, _CANVAS_W, _CANVAS_H,
                                  fill='#c8dcc8', outline='', tags='track')

            def to_flat(pts):
                result = []
                for x, y in pts:
                    sx, sy = world_to_screen(x, y)
                    result.extend([sx, sy])
                return result

            # 도로 면: 외곽 filled grey → 내곽 filled 배경색
            outer_flat = to_flat(_stadium_pts(_STRAIGHT, 1.225))
            inner_flat = to_flat(_stadium_pts(_STRAIGHT, 0.825))
            self.create_polygon(outer_flat, fill='#aaaaaa', outline='', tags='track')
            self.create_polygon(inner_flat, fill='#c8dcc8', outline='', tags='track')

            # 차선 경계선 (외곽/노란/내곽)
            for radius, color, lw in _TRACK_LINES:
                pts_flat = to_flat(_stadium_pts(_STRAIGHT, radius))
                self.create_polygon(pts_flat, fill='', outline=color,
                                    width=lw, tags='track')

        # ── 클릭 이벤트 ──────────────────────────────────────────────
        def _on_left_click(self, event):
            wx = (event.x - _CX) / _SCALE_X
            wy = (_CY - event.y) / _SCALE_Y
            # 출발점 지정 모드 우선 (좌클릭 1회로 완료)
            if self._start_pick_mode:
                self._start_pick_mode = False
                self.config(cursor='')
                if self._on_start_pick:
                    self._on_start_pick(wx, wy)
                return
            # 웨이포인트 기록 모드 우선
            if self._wp_record_mode:
                self._wp_recording.append((wx, wy))
                self._draw_wp_recording()
                return
            if self._place_mode is not None:
                self._objects.append(TrackObject(kind=self._place_mode, wx=wx, wy=wy))
                self._redraw_objects()
                return   # 배치 모드 유지 (연속 배치 가능)
            # 배치 모드 아님 → 차량 시작 위치 설정
            self._pi_state  = [wx, wy, 0.0]
            self._sim_state = [wx, wy, 0.0]
            self._redraw_vehicles()

        def _on_right_click(self, event):
            wx = (event.x - _CX) / _SCALE_X
            wy = (_CY - event.y) / _SCALE_Y
            # 웨이포인트 기록 모드: 우클릭 = 완료
            if self._wp_record_mode:
                self._finish_wp_recording()
                return
            if self._place_mode is not None:
                self.cancel_place_mode()
                return
            # 배치 모드 아님 → 가장 가까운 오브젝트 삭제 (0.25m 이내)
            best_idx, best_d = -1, float('inf')
            for i, obj in enumerate(self._objects):
                d = math.hypot(wx - obj.wx, wy - obj.wy)
                if d < best_d:
                    best_d, best_idx = d, i
            if best_idx >= 0 and best_d < 0.25:
                self._objects.pop(best_idx)
                self._redraw_objects()

        # ── 출발점 지정 모드 API ──────────────────────────────────────
        def enter_start_pick_mode(self, on_pick=None):
            """출발점 지정 모드 진입 — 좌클릭 1회로 출발점 선택."""
            self._start_pick_mode = True
            self._on_start_pick   = on_pick
            self.config(cursor='crosshair')

        def cancel_start_pick_mode(self):
            self._start_pick_mode = False
            self.config(cursor='')

        # ── 웨이포인트 기록 모드 API ──────────────────────────────────
        def enter_wp_record_mode(self, on_done=None):
            """웨이포인트 기록 모드 진입. 좌클릭=추가, 우클릭=완료."""
            self._wp_record_mode = True
            self._wp_recording   = []
            self._on_wp_done     = on_done
            self.config(cursor='crosshair')

        def cancel_wp_record_mode(self):
            """기록 내용 버리고 모드 종료."""
            self._wp_record_mode = False
            self._wp_recording   = []
            self.delete('wp_recording')
            self.config(cursor='')

        def _draw_wp_recording(self):
            """기록 중인 웨이포인트를 임시 파란 점·선으로 표시."""
            self.delete('wp_recording')
            pts = [world_to_screen(p[0], p[1]) for p in self._wp_recording]
            for i in range(len(pts) - 1):
                self.create_line(*pts[i], *pts[i + 1],
                                 fill='#0055ff', width=1, tags='wp_recording')
            r = 4
            for sx, sy in pts:
                self.create_oval(sx - r, sy - r, sx + r, sy + r,
                                 fill='#3388ff', outline='#0033cc',
                                 width=1, tags='wp_recording')

        def _finish_wp_recording(self):
            """우클릭 시 호출 — 기록 확정 후 on_done 콜백."""
            wps = list(self._wp_recording)
            self._wp_record_mode = False
            self._wp_recording   = []
            self.delete('wp_recording')
            self.config(cursor='')
            if self._on_wp_done:
                self._on_wp_done(wps)

        # ── 오브젝트 배치 모드 API ────────────────────────────────────
        def enter_place_mode(self, kind: str, on_cancel=None):
            self._place_mode       = kind
            self._on_mode_cancel   = on_cancel
            self.config(cursor='crosshair')

        def cancel_place_mode(self):
            self._place_mode = None
            self.config(cursor='')
            if self._on_mode_cancel:
                self._on_mode_cancel()

        def clear_objects(self):
            self._objects.clear()
            self.delete('obj')
            self._highlighted_objs.clear()

        # ── 오브젝트 감지 ────────────────────────────────────────────
        def check_objects(self, wx: float, wy: float) -> dict:
            """차량 위치 주변 오브젝트 감지. 트리거된 효과 dict 반환."""
            triggered = {
                'obstacle':   False,
                'dist_m':     None,
                'stop_line':  False,
                'turn_left':  False,
            }
            new_hl: set[int] = set()
            for i, obj in enumerate(self._objects):
                d = math.hypot(wx - obj.wx, wy - obj.wy)
                if d <= _DETECT_R[obj.kind]:
                    new_hl.add(i)
                    if obj.kind == OBJ_OBSTACLE:
                        triggered['obstacle'] = True
                        if triggered['dist_m'] is None or d < triggered['dist_m']:
                            triggered['dist_m'] = d
                    elif obj.kind == OBJ_STOP_LINE:
                        triggered['stop_line'] = True
                    elif obj.kind == OBJ_TURN_LEFT:
                        triggered['turn_left'] = True
            # 하이라이트 변경 시에만 재드로우
            if new_hl != self._highlighted_objs:
                self._highlighted_objs = new_hl
                self._redraw_objects()
            return triggered

        def get_pi_world_pos(self):
            return (self._pi_state[0], self._pi_state[1]) if self._pi_state else None

        # ── 공개 API ─────────────────────────────────────────────────
        def set_start_pos(self, wx, wy, heading=0.0):
            self._pi_state  = [wx, wy, heading]
            self._sim_state = [wx, wy, heading]
            self._redraw_vehicles()

        def clear(self):
            """궤적 + 차량 위치 초기화 (오브젝트·웨이포인트는 유지)."""
            self.delete('trail')
            self.delete('lane_change_path')
            self.delete('dense_path')
            self._pi_state  = None
            self._sim_state = None
            self.delete('pi_vehicle')
            self.delete('sim_vehicle')
            self._highlighted_objs.clear()
            self._redraw_objects()

        def reset_trail(self):
            self.delete('trail')

        def update_pi(self, throttle, steer, dt):
            if self._pi_state is None:
                return
            x, y, h = apply_motion_model(*self._pi_state, throttle, steer, dt,
                                          self.k_v, self.k_w)
            old = world_to_screen(*self._pi_state[:2])
            self._pi_state = [x, y, h]
            new = world_to_screen(x, y)
            self.create_line(*old, *new, fill='#3366cc', width=1, tags='trail')
            self._redraw_vehicles()

        def update_sim(self, throttle, steer, dt):
            if self._sim_state is None:
                return
            x, y, h = apply_motion_model(*self._sim_state, throttle, steer, dt,
                                          self.k_v, self.k_w)
            old = world_to_screen(*self._sim_state[:2])
            self._sim_state = [x, y, h]
            new = world_to_screen(x, y)
            self.create_line(*old, *new, fill='#cc3333', width=1, tags='trail')
            self._redraw_vehicles()

        def _redraw_objects(self):
            self.delete('obj')
            for i, obj in enumerate(self._objects):
                _draw_obj(self, obj, tag='obj', highlight=(i in self._highlighted_objs))
            if self.find_withtag('obj'):
                self.tag_raise('trail', 'obj')  # 궤적이 오브젝트보다 위

        # ── 전방 센서 시각화 (카메라 FOV + 초음파 거리측정) ───────────
        def set_detection(self, range_m, detecting, show=True):
            """초음파 최대 측정거리(range_m, m)·표시 여부 갱신."""
            self._detect_range_m = range_m
            self._detect_on      = detecting
            self._detect_show    = show
            self._redraw_vehicles()

        def _ultrasonic_distance(self):
            """차량 중앙 전방 좁은 빔 내 가장 가까운 장애물까지 거리(m). 없으면 None."""
            if self._pi_state is None:
                return None
            wx, wy, h = self._pi_state
            fx, fy = math.cos(h), math.sin(h)
            maxr = self._detect_range_m or 2.0
            best = None
            for obj in self._objects:
                if obj.kind != OBJ_OBSTACLE:
                    continue
                dx, dy = obj.wx - wx, obj.wy - wy
                fwd = dx * fx + dy * fy                 # 전방 거리
                if fwd <= 0.0 or fwd > maxr:
                    continue
                if abs(-dx * fy + dy * fx) > 0.11:      # 빔축 수직거리 (빔 폭 밖)
                    continue
                if best is None or fwd < best:
                    best = fwd
            return best

        def tick_sensor_anim(self):
            """초음파 펄스 위상 전진 + 센서 재드로우 (연속 전파 애니메이션)."""
            if self._detect_show and self._pi_state is not None:
                self._us_phase = (self._us_phase + 0.12) % 1.0
                self._draw_sensors()

        def _sensor_cone(self, ax, ay, h, half, length, color, stipple):
            apex = world_to_screen(ax, ay)
            e1 = world_to_screen(ax + length * math.cos(h + half), ay + length * math.sin(h + half))
            e2 = world_to_screen(ax + length * math.cos(h - half), ay + length * math.sin(h - half))
            self.create_polygon(apex[0], apex[1], e1[0], e1[1], e2[0], e2[1],
                                fill=color, outline=color, stipple=stipple, width=1, tags='sensor')

        def _draw_sensors(self):
            """카메라(넓은 FOV·파랑) + 초음파(중앙 전방 1개·주황, 반사거리 측정·펄스)."""
            self.delete('sensor')
            if not self._detect_show or self._pi_state is None:
                return
            CAM, US = '#3a7afe', '#ff8c00'
            wx, wy, h = self._pi_state
            fx, fy = math.cos(h), math.sin(h)
            ppx, ppy = -fy, fx                        # 빔축 수직 단위벡터
            ax, ay = wx + fx * 0.09, wy + fy * 0.09   # 차량 앞 기준점

            # 카메라: 넓은 FOV 콘 (고정 깊이)
            self._sensor_cone(ax, ay, h, math.radians(28), 0.50, CAM, 'gray12')

            # 초음파: 중앙 전방 1개 빔 + 반사거리
            us_half = math.radians(6)
            us_d    = self._ultrasonic_distance()
            us_len  = us_d if us_d is not None else (self._detect_range_m or 2.0)
            self._sensor_cone(ax, ay, h, us_half, us_len, US, 'gray25')
            # 전파(핑) 호 — 전방으로 연속 확산
            for k in range(3):
                d = ((self._us_phase + k / 3.0) % 1.0) * us_len
                hw = max(0.006, d * math.tan(us_half))
                x1, y1 = world_to_screen(ax + fx * d + ppx * hw, ay + fy * d + ppy * hw)
                x2, y2 = world_to_screen(ax + fx * d - ppx * hw, ay + fy * d - ppy * hw)
                self.create_line(x1, y1, x2, y2, fill=US, width=2, tags='sensor')
            # 반사파 수신 → 거리 표시
            if us_d is not None:
                hsx, hsy = world_to_screen(ax + fx * us_d, ay + fy * us_d)
                self.create_oval(hsx - 6, hsy - 6, hsx + 6, hsy + 6,
                                 outline='#cc1100', width=2, tags='sensor')
                self.create_text(hsx, hsy - 13, text=f'{us_d * 100:.0f}cm',
                                 fill='#cc1100', font=('TkDefaultFont', 9, 'bold'), tags='sensor')

            self._draw_sensor_legend(CAM, US)

        def _draw_sensor_legend(self, cam, us):
            x, y = 12, 10
            self.create_rectangle(x, y, x + 14, y + 12, fill=cam, outline='#1133aa', tags='sensor')
            self.create_text(x + 18, y + 6, text='카메라 (Camera) — 넓은 시야',
                             anchor='w', fill='#1133aa', font=('TkDefaultFont', 8), tags='sensor')
            self.create_rectangle(x, y + 18, x + 14, y + 30, fill=us, outline='#aa5500', tags='sensor')
            self.create_text(x + 18, y + 24, text='초음파 (Ultrasonic) — 중앙 전방·반사거리',
                             anchor='w', fill='#aa5500', font=('TkDefaultFont', 8), tags='sensor')

        def set_pi_ghost(self, label):
            """pi 차량(수신 데이터 기반)을 유령(반투명)+라벨로 표시. None=일반 표시."""
            self._pi_ghost_label = label
            self._redraw_vehicles()

        def set_sim_ghost(self, label):
            """sim 차량(수신 데이터 기반)을 유령(반투명)+라벨로 표시. None=일반 표시."""
            self._sim_ghost_label = label
            self._redraw_vehicles()

        def _redraw_vehicles(self):
            self.delete('pi_vehicle')
            self.delete('sim_vehicle')
            self._draw_sensors()   # 차량 아래 레이어로 먼저
            if self._pi_state:
                sx, sy = world_to_screen(*self._pi_state[:2])
                ghost = self._pi_ghost_label is not None
                _draw_car(self, sx, sy, self._pi_state[2],
                          body_color='#f0f0f0', wheel_color='#2244cc', tag='pi_vehicle',
                          ghost=ghost, label=self._pi_ghost_label)
            if self._sim_state:
                sx, sy = world_to_screen(*self._sim_state[:2])
                sghost = self._sim_ghost_label is not None
                _draw_car(self, sx, sy, self._sim_state[2],
                          body_color='#ffddcc', wheel_color='#888888', tag='sim_vehicle',
                          ghost=sghost, label=self._sim_ghost_label)
            # 레이어 순서: dense_path < trail < lane_change_path < 차량
            if self.find_withtag('dense_path'):
                self.tag_raise('dense_path')
            if self.find_withtag('lane_change_path'):
                self.tag_raise('lane_change_path')
            self.tag_raise('pi_vehicle')
            self.tag_raise('sim_vehicle')

        # ── 웨이포인트 / 차선변경 경로 ───────────────────────────────────
        def draw_waypoints(self, waypoints: list):
            """웨이포인트를 빨간 원 + 굵은 연결선으로 표시. [(wx, wy), ...]"""
            self.delete('waypoints')
            if not waypoints:
                return
            pts = [world_to_screen(p[0], p[1]) for p in waypoints]
            for i in range(len(pts) - 1):
                self.create_line(*pts[i], *pts[i + 1],
                                 fill='#cc0000', width=2, tags='waypoints')
            r = 5
            for sx, sy in pts:
                self.create_oval(sx - r, sy - r, sx + r, sy + r,
                                 fill='#ff3333', outline='#880000', width=2, tags='waypoints')

        def draw_dense_path(self, path_points: list):
            """보간된 경로를 진한 파란색 점선으로 표시. [(wx, wy), ...]"""
            self.delete('dense_path')
            if len(path_points) < 2:
                return
            pts_flat = []
            for p in path_points:
                pts_flat.extend(world_to_screen(p[0], p[1]))
            self.create_line(pts_flat, fill='#0044cc', width=2,
                             dash=(5, 3), capstyle='round', tags='dense_path')

        def clear_dense_path(self):
            self.delete('dense_path')

        def clear_waypoints(self):
            self.delete('waypoints')

        # ── 오브젝트 직렬화 API ───────────────────────────────────────
        def get_objects_data(self) -> list:
            """오브젝트 목록을 dict 리스트로 반환 (JSON 저장용)."""
            return [{'kind': o.kind, 'wx': o.wx, 'wy': o.wy} for o in self._objects]

        def set_objects_data(self, data: list):
            """dict 리스트로 오브젝트 복원 (JSON 불러오기용)."""
            self._objects = [TrackObject(kind=d['kind'], wx=d['wx'], wy=d['wy'])
                             for d in data]
            self._highlighted_objs.clear()
            self._redraw_objects()

        def draw_lane_change_path(self, path_points: list):
            """차선 변경 계획 경로를 형광 녹색 촘촘 점선으로 표시."""
            self.delete('lane_change_path')
            if len(path_points) < 2:
                return
            pts_flat = []
            for p in path_points:
                pts_flat.extend(world_to_screen(p[0], p[1]))
            self.create_line(pts_flat, fill='#7fff00', width=2,
                             dash=(3, 2), capstyle='round', tags='lane_change_path')

        def clear_lane_change_path(self):
            self.delete('lane_change_path')

except ImportError:
    class TrackCanvas:  # type: ignore[no-redef]
        def __init__(self, *args, **kwargs):
            raise RuntimeError("tkinter을 사용할 수 없는 환경입니다")
