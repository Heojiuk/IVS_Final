"""VILS Simulator — main tkinter application.

Run:
    cd d:/Source/IVS_Final/simulator
    python app.py
"""
import math, os, sys, time
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import _src_path; _src_path.add()

from core_module.v2v import packet_parser, packet_generator, PACKET_LEN, fmt_ms_of_day
from core_module import config
from sim_algorithm.perception import SimPerception
from logger import SessionRecorder
from track_canvas import (TrackCanvas, OBJ_OBSTACLE, OBJ_STOP_LINE, OBJ_TURN_LEFT,
                          _track_perpendicular, _STRAIGHT, stadium_lane_path)
from vils_core import VILSEngine
from scenario import ScenarioStep, PARAM_BOOL
from scenario_window import ScenarioEditor
from data_editor import DataEditorTab
# data_view 는 matplotlib 의존 → 지연 임포트(_import_data_view). 미설치 시에도 앱은 정상 기동.

LOG_ROOT = os.path.join(os.path.dirname(__file__), '..', 'data', 'log')
TICK_MS = 50
DEFAULT_START = (0.0, -1.025, 0.0)   # 타원 하단, 동쪽 방향

_GRP   = 'Group.TLabelframe'
_GRP_L = 'Group.TLabelframe.Label'


def _setup_styles():
    s = ttk.Style()
    s.configure(_GRP,   background='white', relief='groove', padding=4)
    s.configure(_GRP_L, font=('TkDefaultFont', 9, 'bold'), background='white')


def _make_group(parent, title):
    return ttk.LabelFrame(parent, text=title, style=_GRP)


def _import_data_view():
    """data_view(LiveMonitorWindow, BinAnalysisWindow) 지연 임포트.
    matplotlib 미설치 등 ImportError 시 None 반환 → 호출측에서 안내."""
    try:
        from data_view import LiveMonitorWindow, BinAnalysisWindow
        return LiveMonitorWindow, BinAnalysisWindow
    except ImportError:
        return None


_VIEW_DEP_MSG = ('View 기능(plot)은 matplotlib 가 필요합니다.\n'
                 '설치: py -m pip install matplotlib')


def _make_scrollable_left(parent, width=295):
    outer = ttk.Frame(parent, width=width)
    outer.pack(side='left', fill='y')
    outer.pack_propagate(False)

    cv  = tk.Canvas(outer, bg='#ebebeb', highlightthickness=0)
    vsb = ttk.Scrollbar(outer, orient='vertical', command=cv.yview)
    cv.configure(yscrollcommand=vsb.set)
    vsb.pack(side='right', fill='y')
    cv.pack(side='left', fill='both', expand=True)

    inner  = ttk.Frame(cv)
    win_id = cv.create_window((0, 0), window=inner, anchor='nw')

    inner.bind('<Configure>', lambda _: cv.configure(scrollregion=cv.bbox('all')))
    cv.bind('<Configure>',    lambda e: cv.itemconfig(win_id, width=e.width))
    cv.bind('<Enter>',  lambda _: cv.bind_all('<MouseWheel>',
            lambda ev: cv.yview_scroll(int(-1 * ev.delta / 120), 'units')))
    cv.bind('<Leave>',  lambda _: cv.unbind_all('<MouseWheel>'))
    return inner


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title('VILS Simulator')
        self.resizable(True, True)
        self.geometry('1310x650')
        _setup_styles()

        nb = ttk.Notebook(self)
        nb.pack(fill='both', expand=True, padx=4, pady=4)
        self._nb = nb

        self._tabs = {}
        for role in ('follower', 'leader'):
            tab = RoleTab(nb, role)
            nb.add(tab, text=role.capitalize())
            self._tabs[role] = tab

        de_tab = DataEditorTab(nb)
        nb.add(de_tab, text='Data Editor')

        nb.bind('<<NotebookTabChanged>>', self._on_tab_change)

        # ── View 메뉴 (서브시스템 plot + 데이터 뷰) ──────────────────────
        menubar = tk.Menu(self)
        view_menu = tk.Menu(menubar, tearoff=False)
        view_menu.add_command(label='실시간 모니터 (Live Monitor)',
                              command=self._open_live_monitor)
        view_menu.add_command(label='bin 분석 (Bin Analysis)',
                              command=self._open_bin_analysis)
        menubar.add_cascade(label='View', menu=view_menu)
        self.config(menu=menubar)

        self.after(300, self._sensor_anim)   # 초음파 펄스 연속 애니메이션 시작

    def _active_role_tab(self):
        """현재 선택된 역할 탭(leader/follower) 반환. Data Editor 탭이면 None."""
        idx = self._nb.index(self._nb.select())
        tabs = list(self._tabs.values())
        return tabs[idx] if idx < len(tabs) else None

    def _sensor_anim(self):
        """초음파 펄스 연속 애니메이션 — 활성 탭 트랙 센서 갱신 (~11Hz)."""
        tab = self._active_role_tab()
        if tab is not None:
            tab._track.tick_sensor_anim()
        self.after(90, self._sensor_anim)

    def _open_live_monitor(self):
        tab = self._active_role_tab()
        if tab is None:
            messagebox.showinfo('View', 'Leader/Follower 탭에서 열어주세요.', parent=self)
            return
        dv = _import_data_view()
        if dv is None:
            messagebox.showwarning('View', _VIEW_DEP_MSG, parent=self)
            return
        tab.open_live_monitor(dv[0])   # LiveMonitorWindow

    def _open_bin_analysis(self):
        dv = _import_data_view()
        if dv is None:
            messagebox.showwarning('View', _VIEW_DEP_MSG, parent=self)
            return
        dv[1](self, LOG_ROOT)          # BinAnalysisWindow

    def _on_tab_change(self, event):
        nb    = event.widget
        idx   = nb.index(nb.select())
        role  = list(self._tabs.keys())[idx]
        other = 'leader' if role == 'follower' else 'follower'
        if self._tabs[other].is_running:
            nb.select(list(self._tabs.keys()).index(other))
            messagebox.showwarning('VILS',
                f'{other.capitalize()} 탭이 실행 중입니다. 먼저 Stop 해주세요.')


class RoleTab(ttk.Frame):
    def __init__(self, parent, role):
        super().__init__(parent)
        self._role  = role
        self._engine     = None
        self._recorder   = SessionRecorder(LOG_ROOT)
        self._perception = SimPerception()
        self._mode       = tk.StringVar(value='realtime')
        self._playback_packets  = []
        self._playback_raw      = []      # 원본 60B 바이트 (재송신용)
        self._pb_idx            = 0
        self._pb_speed          = tk.DoubleVar(value=1.0)
        self._pb_running        = False
        self._pb_start_pose     = None    # playback 출발 자세 (meta.json 복원)
        # Playback UDP 재송신 (저장 데이터를 원본 tx 타이밍으로 다시 쏨)
        self._pb_udp_sock       = None
        self._pb_peer           = None
        self._pb_tx_ok          = 0
        self._pb_tx_fail        = 0
        self._prev_pi_tx_abs    = None
        self._start_time        = None
        self._scene_vars        = {}
        self._bus_labels        = {}
        self._perception_widgets = []     # playback 시 비활성화할 인지 입력 위젯
        # 시나리오
        self._scenario_steps  : list[ScenarioStep] = []
        self._scenario_cursor : int = 0
        # 오브젝트 감지 상태 (이전 틱에 감지 중이었는지 추적 → 해제 시 복원)
        self._obstacle_active  = False
        self._stop_line_active = False
        self._turn_sign_active = False
        self._lane_change_drawn = False   # 장애물 1회당 1번만 경로 계산
        self._waypoints: list      = []
        self._sim_dense_path: list = []   # 보간된 dense world pos (pure pursuit용)
        self._sim_path: list       = []   # [(throttle, steer)] playback bin 생성용
        self._sim_path_idx: int    = 0
        self._sim_running: bool    = False
        self._sim_lane: int        = 1    # 출발점 지정 시 자동 감지된 주행 차선
        # 장애물 회피 / 차선 변경 상태
        self._avoidance_mode: bool    = False   # 차선 전환 경로 추종 중
        self._avoidance_path: list    = []
        self._avoidance_path_idx: int = 0
        self._lane_paths: dict        = {}      # {1: path1, 2: path2} 양쪽 차선 경로
        self._lane_change_target: int = 1       # 전환 목표 차선
        self._lane_change_cooldown: int = 0     # 재트리거 방지 틱 카운트
        # Simulator UDP TX
        self._sim_udp_sock = None
        self._sim_peer     = None
        self._sim_seq: int = 0
        self._sim_key      = None
        self._sim_tx_ok: int   = 0    # UDP 송신 성공 누적
        self._sim_tx_fail: int = 0    # UDP 송신 실패 누적
        self._sim_last_tx      = None # 마지막 송신 시각 (50ms 고정 주기 게이트)
        # Simulator UDP RX (실차에서 돌아오는 상대 데이터 수신·로깅용)
        self._sim_rx_sock   = None
        self._sim_rx_thread = None
        self._sim_rx_stop   = None
        self._sim_last_rx   = None    # 마지막 수신 V2VState (모니터 통신 표시용)
        # View 메뉴 실시간 모니터 창
        self._live_monitor  = None

        self._build_ui()

    # ── UI 구성 ─────────────────────────────────────────────────────────
    def _build_ui(self):
        mode_bar = ttk.Frame(self)
        mode_bar.pack(fill='x', padx=6, pady=2)
        for val, text in [('realtime', 'Real-time'),
                           ('playback', 'Playback'),
                           ('simulator', 'Simulator')]:
            ttk.Radiobutton(mode_bar, text=text, variable=self._mode,
                            value=val, command=self._on_mode_change
                            ).pack(side='left', padx=4)

        main = ttk.Frame(self)
        main.pack(fill='both', expand=True)

        left = _make_scrollable_left(main, width=380)

        right = ttk.Frame(main)
        right.pack(side='left', fill='both', expand=True)

        btn_bar = ttk.Frame(right)
        btn_bar.pack(fill='x', padx=2, pady=2)
        ttk.Button(btn_bar, text='화면 초기화',  command=self._clear_canvas).pack(side='left', padx=2)
        ttk.Button(btn_bar, text='기본 시작위치', command=self._set_default_pos).pack(side='left', padx=2)

        ttk.Separator(btn_bar, orient='vertical').pack(side='left', fill='y', padx=4, pady=2)

        # 오브젝트 배치 버튼 (전 모드 공통)
        self._obj_btns = {}
        for kind, label in [(OBJ_OBSTACLE,  '🚧 장애물'),
                             (OBJ_STOP_LINE, '🛑 정지선'),
                             (OBJ_TURN_LEFT, '↰ 좌회전')]:
            b = ttk.Button(btn_bar, text=label,
                           command=lambda k=kind: self._enter_obj_mode(k))
            b.pack(side='left', padx=2)
            self._obj_btns[kind] = b
        ttk.Button(btn_bar, text='오브젝트 전체 삭제',
                   command=lambda: self._track.clear_objects()).pack(side='left', padx=4)

        # 배치 모드 상태 표시
        self._obj_mode_var = tk.StringVar(value='')
        tk.Label(btn_bar, textvariable=self._obj_mode_var,
                 fg='#cc4400', font=('TkDefaultFont', 8)).pack(side='left', padx=4)

        self._track = TrackCanvas(right)
        self._track.pack()

        self._build_perception_box(left)
        self._build_decision_box(left)
        self._build_motion_box(left)
        self._build_v2v_box(left)
        self._build_motion_model_box(left)
        self._build_network_box(left)
        self._build_control_bar(left)
        self._build_playback_bar(left)
        self._build_simulator_bar(left)

        self._on_mode_change()

    # ── 인지(Perception) ─────────────────────────────────────────────
    def _build_perception_box(self, parent):
        lf = _make_group(parent, '인지 파라미터 주입 (Perception)')
        lf.pack(fill='x', pady=2, padx=2)

        def add_check(key, label, default=False):
            v = tk.BooleanVar(value=default)
            cb = tk.Checkbutton(lf, text=label, variable=v, bg='white',
                                activebackground='white',
                                command=lambda: self._perception.params.update({key: v.get()}))
            cb.pack(anchor='w')
            self._scene_vars[key] = v
            self._perception_widgets.append(cb)

        def add_slider(key, label, from_, to, default=0.0, cm=False):
            # cm=True → 내부값(m)은 그대로, 표시만 cm 로 (거리 단위)
            def _disp(val):
                return f'{val * 100:.0f}cm' if cm else f'{val:.3f}'
            row = tk.Frame(lf, bg='white')
            row.pack(fill='x')
            tk.Label(row, text=label, width=22, anchor='w', bg='white').pack(side='left')
            v       = tk.DoubleVar(value=default)
            val_lbl = tk.Label(row, text=_disp(default), width=6, anchor='e',
                               fg='#1155cc', bg='white')
            val_lbl.pack(side='right')
            sc = ttk.Scale(row, variable=v, from_=from_, to=to, orient='horizontal',
                           command=lambda _, _v=v, _l=val_lbl: (
                               self._perception.params.update({key: round(_v.get(), 4)}),
                               _l.config(text=_disp(_v.get()))))
            sc.pack(side='left', fill='x', expand=True)
            self._scene_vars[key] = v
            self._perception_widgets.append(sc)

        def add_lane(key, label):
            row = tk.Frame(lf, bg='white')
            row.pack(fill='x')
            tk.Label(row, text=label, width=22, anchor='w', bg='white').pack(side='left')
            v = tk.IntVar(value=1)
            for val in [0, 1, 2]:
                rb = tk.Radiobutton(row, text=str(val), variable=v, value=val,
                                    bg='white', activebackground='white',
                                    command=lambda: self._perception.params.update({key: v.get()}))
                rb.pack(side='left')
                self._perception_widgets.append(rb)
            self._scene_vars[key] = v

        add_check('lane_valid',  '차선유효(lane_valid)',       default=True)
        add_lane ('current_lane','현재차선(current_lane)')
        add_slider('lane_offset_m',      '측방오프셋(offset cm)', -0.5,   0.5, cm=True)
        add_slider('lane_heading_rad',   '헤딩각(heading_rad)',   -0.785, 0.785)
        add_slider('lane_curvature_1pm', '곡률(curvature_1pm)',  -2.0,   2.0)
        add_check('front_clear', '전방비어있음(front_clear)',   default=True)
        self._build_dist_front_row(lf)
        add_check('stop_signal', '정지신호(stop_signal)',       default=False)

    def _build_dist_front_row(self, parent):
        row = tk.Frame(parent, bg='white')
        row.pack(fill='x')
        tk.Label(row, text='전방거리(dist_front cm)', width=22, anchor='w',
                 bg='white').pack(side='left')

        self._dist_detected = tk.BooleanVar(value=False)
        self._dist_val      = tk.DoubleVar(value=2.0)
        self._dist_val_lbl = tk.Label(row, text='', width=6, anchor='e', fg='#1155cc', bg='white')
        self._dist_val_lbl.pack(side='right')
        val_lbl = self._dist_val_lbl

        def on_change(*_):
            v = round(self._dist_val.get(), 2) if self._dist_detected.get() else None
            self._perception.params['dist_front_m'] = v   # 내부는 m (Scene 계약)
            val_lbl.config(text='' if v is None else f'{v * 100:.0f}cm')
            # 전방 감지범위(카메라·초음파) 콘 시각화 — 길이=슬라이더(m), 감지 체크 시 빨강
            self._track.set_detection(self._dist_val.get(), self._dist_detected.get())

        dcb = tk.Checkbutton(row, text='감지', variable=self._dist_detected, bg='white',
                             activebackground='white', command=on_change)
        dcb.pack(side='left')
        dsc = ttk.Scale(row, variable=self._dist_val, from_=0.0, to=5.0, orient='horizontal',
                        command=lambda _: on_change())
        dsc.pack(side='left', fill='x', expand=True)
        self._perception_widgets += [dcb, dsc]

    # ── 판단 출력 (Decision) ──────────────────────────────────────────
    def _build_decision_box(self, parent):
        lf = _make_group(parent, '판단 출력 (Decision)')
        lf.pack(fill='x', pady=2, padx=2)
        for key, label in [('cmd_behavior', '주행명령(cmd_behavior)'),
                            ('mode',         '시스템모드(mode)')]:
            self._add_monitor_row(lf, key, label)

    # ── 모션 출력 (Motion) ────────────────────────────────────────────
    def _build_motion_box(self, parent):
        lf = _make_group(parent, '모션 출력 (Motion)')
        lf.pack(fill='x', pady=2, padx=2)
        for key, label in [('ego_throttle', '자차스로틀(ego_throttle)'),
                            ('ego_steer',    '자차조향(ego_steer)')]:
            self._add_monitor_row(lf, key, label)

    # ── 통신 상태 (V2V) ───────────────────────────────────────────────
    def _build_v2v_box(self, parent):
        lf = _make_group(parent, '통신 상태 (V2V)')
        lf.pack(fill='x', pady=2, padx=2)
        for key, label in [('pi_seq',      '상대seq(pi_seq)'),
                            ('pi_lane',     '상대차선(pi_lane)'),
                            ('pi_throttle', '상대스로틀(pi_throttle)'),
                            ('pi_steer',    '상대조향(pi_steer)'),
                            ('link_state',  '링크상태(link_state)'),
                            ('link_age',    '링크경과(link_age)')]:
            self._add_monitor_row(lf, key, label)

    def _add_monitor_row(self, lf, key, label):
        row = tk.Frame(lf, bg='white')
        row.pack(fill='x')
        tk.Label(row, text=label, width=26, anchor='w', bg='white').pack(side='left')
        v = tk.StringVar(value='—')
        tk.Label(row, textvariable=v, width=13, anchor='e',
                 fg='#1155cc', bg='white').pack(side='right')
        self._bus_labels[key] = v

    # ── 속도 모델 ─────────────────────────────────────────────────────
    def _build_motion_model_box(self, parent):
        lf = _make_group(parent, '속도 모델 (Motion Model)')
        lf.pack(fill='x', pady=2, padx=2)
        for key, label, from_, to, default in [
            ('__kv', '속도배율(k_v)', 0.1, 5.0,  1.0),
            ('__kw', '조향배율(k_w)', 0.1, 10.0, 2.0),
        ]:
            row = tk.Frame(lf, bg='white')
            row.pack(fill='x')
            tk.Label(row, text=label, width=15, anchor='w', bg='white').pack(side='left')
            v = tk.DoubleVar(value=default)
            val_lbl = tk.Label(row, text=f'{default:.2f}', width=5, anchor='e',
                               fg='#1155cc', bg='white')
            val_lbl.pack(side='right')
            ttk.Scale(row, variable=v, from_=from_, to=to, orient='horizontal',
                      command=lambda _, _v=v, _l=val_lbl: _l.config(text=f'{_v.get():.2f}')
                      ).pack(side='left', fill='x', expand=True)
            self._scene_vars[key] = v

    # ── 네트워크 설정 ────────────────────────────────────────────────
    _NET_IPS = {
        'dev':      {'leader': '192.168.202.91', 'follower': '192.168.201.102'},
        'release':  {'leader': '192.168.0.11',    'follower': '192.168.0.12'},
        'loopback': {'leader': '127.0.0.1',       'follower': '127.0.0.1'},
    }

    def _build_network_box(self, parent):
        frm = _make_group(parent, '네트워크 (Network)')
        frm.pack(fill='x', pady=2, padx=2)

        # 모드 라디오
        mode_row = tk.Frame(frm, bg='white')
        mode_row.pack(fill='x', pady=2)
        self._net_mode = tk.StringVar(value=os.environ.get('IVS_MODE', 'dev'))
        for m in ('dev', 'release', 'loopback'):
            tk.Radiobutton(mode_row, text=m, variable=self._net_mode, value=m,
                           bg='white', activebackground='white',
                           command=self._on_net_mode_change).pack(side='left', padx=4)

        G = {'sticky': 'w', 'padx': 4, 'pady': 1}
        ip_frm = tk.Frame(frm, bg='white')
        ip_frm.pack(fill='x')

        tk.Label(ip_frm, text='Leader IP',   bg='white', width=10, anchor='w').grid(row=0, column=0, **G)
        self._leader_ip = tk.StringVar()
        ttk.Entry(ip_frm, textvariable=self._leader_ip, width=18).grid(row=0, column=1, **G)

        tk.Label(ip_frm, text='Follower IP', bg='white', width=10, anchor='w').grid(row=1, column=0, **G)
        self._follower_ip = tk.StringVar()
        ttk.Entry(ip_frm, textvariable=self._follower_ip, width=18).grid(row=1, column=1, **G)

        self._on_net_mode_change()   # 초기값 채우기

    def _on_net_mode_change(self):
        m = self._net_mode.get()
        ips = self._NET_IPS.get(m, self._NET_IPS['dev'])
        self._leader_ip.set(ips['leader'])
        self._follower_ip.set(ips['follower'])

    def _apply_network_settings(self):
        """Start 직전 호출 — 환경변수와 config._IPS를 UI 값으로 덮어씀."""
        m = self._net_mode.get()
        os.environ['IVS_MODE'] = m
        config._IPS[m]['leader']   = self._leader_ip.get().strip()
        config._IPS[m]['follower'] = self._follower_ip.get().strip()

    # ── Simulator 제어 ────────────────────────────────────────────────
    def _build_simulator_bar(self, parent):
        self._sim_frame = _make_group(parent, 'Simulator')
        self._sim_frame.pack(fill='x', pady=2, padx=2)

        # 시나리오 저장/불러오기 (웨이포인트 + 오브젝트)
        sc_row = tk.Frame(self._sim_frame, bg='white')
        sc_row.pack(fill='x', pady=2)
        ttk.Button(sc_row, text='💾 시나리오 저장', command=self._sim_save_scenario
                   ).pack(side='left', padx=2)
        ttk.Button(sc_row, text='📂 시나리오 불러오기', command=self._sim_load_scenario
                   ).pack(side='left', padx=2)

        # 출발점 지정 (클릭 → 차선 자동 감지 → CCW 자동 주행 경로 생성)
        wp_row = tk.Frame(self._sim_frame, bg='white')
        wp_row.pack(fill='x', pady=2)
        ttk.Button(wp_row, text='📍 출발점 지정',
                   command=self._pick_start_point).pack(side='left', padx=2)

        # Start/Stop
        ctrl_row = tk.Frame(self._sim_frame, bg='white')
        ctrl_row.pack(fill='x', pady=2)
        self._sim_start_btn = ttk.Button(ctrl_row, text='▶ Start',
                                         command=self._start_simulator)
        self._sim_start_btn.pack(side='left', padx=4)
        self._sim_stop_btn  = ttk.Button(ctrl_row, text='⏹ Stop',
                                         command=self._stop_simulator, state='disabled')
        self._sim_stop_btn.pack(side='left', padx=2)

        # 스로틀(속도) 슬라이더
        thr_row = tk.Frame(self._sim_frame, bg='white')
        thr_row.pack(fill='x')
        tk.Label(thr_row, text='스로틀(throttle)', width=16, anchor='w',
                 bg='white').pack(side='left')
        self._sim_throttle = tk.DoubleVar(value=0.30)
        self._sim_thr_lbl  = tk.Label(thr_row, text='0.30', width=5, anchor='e',
                                      fg='#1155cc', bg='white')
        self._sim_thr_lbl.pack(side='right')
        ttk.Scale(thr_row, variable=self._sim_throttle, from_=0.05, to=1.0,
                  orient='horizontal',
                  command=lambda _: self._sim_thr_lbl.config(
                      text=f'{self._sim_throttle.get():.2f}')
                  ).pack(side='left', fill='x', expand=True)

        # 상태
        self._sim_status_var = tk.StringVar(value='대기 중 (WP 기록 후 경로 생성)')
        tk.Label(self._sim_frame, textvariable=self._sim_status_var,
                 anchor='w', bg='white', fg='#444', font=('TkDefaultFont', 8)
                 ).pack(fill='x', padx=2, pady=(0, 2))

        # 배속
        spd_row = tk.Frame(self._sim_frame, bg='white')
        spd_row.pack(fill='x')
        tk.Label(spd_row, text='배속', bg='white').pack(side='left')
        self._sim_speed = tk.DoubleVar(value=1.0)
        for spd in [0.5, 1.0, 2.0, 4.0]:
            tk.Radiobutton(spd_row, text=f'{spd}×', variable=self._sim_speed, value=spd,
                           bg='white', activebackground='white').pack(side='left')

    # ── Real-time 제어 ────────────────────────────────────────────────
    def _build_control_bar(self, parent):
        self._ctrl_frame = _make_group(parent, '실시간 제어 (Real-time)')
        self._ctrl_frame.pack(fill='x', pady=2, padx=2)
        row = tk.Frame(self._ctrl_frame, bg='white')
        row.pack(fill='x')
        self._start_btn = ttk.Button(row, text='▶ Start', command=self._start_realtime)
        self._start_btn.pack(side='left', padx=4, pady=2)
        self._stop_btn  = ttk.Button(row, text='⏹ Stop',  command=self._stop_realtime,
                                     state='disabled')
        self._stop_btn.pack(side='left', padx=4, pady=2)
        self._status_var = tk.StringVar(value='대기 중')
        tk.Label(self._ctrl_frame, textvariable=self._status_var,
                 anchor='w', bg='white').pack(fill='x', padx=2)
        self._pkt_var = tk.StringVar(value='패킷: 0')
        tk.Label(self._ctrl_frame, textvariable=self._pkt_var,
                 anchor='w', bg='white').pack(fill='x', padx=2)

    # ── Playback 제어 ─────────────────────────────────────────────────
    def _build_playback_bar(self, parent):
        self._pb_frame = _make_group(parent, '재생 (Playback)')
        self._pb_frame.pack(fill='x', pady=2, padx=2)

        r0 = tk.Frame(self._pb_frame, bg='white')
        r0.pack(fill='x', pady=2)
        ttk.Button(r0, text='📂 파일 열기', command=self._open_bin).pack(side='left', padx=2)

        # 로드된 파일명
        self._pb_fname = tk.StringVar(value='')
        tk.Label(self._pb_frame, textvariable=self._pb_fname,
                 anchor='w', bg='white', fg='#1155cc', font=('Consolas', 8)
                 ).pack(fill='x', padx=2)

        # 시간 범위 표시
        self._pb_range = tk.StringVar(value='—')
        tk.Label(self._pb_frame, textvariable=self._pb_range,
                 anchor='w', bg='white', fg='#444', font=('Consolas', 8)
                 ).pack(fill='x', padx=2)

        # 진행 바
        self._pb_progress = ttk.Progressbar(self._pb_frame, mode='determinate',
                                            maximum=100, value=0)
        self._pb_progress.pack(fill='x', padx=2, pady=2)

        # 현재 시각 / 위치
        self._pb_cur_time = tk.StringVar(value='00:00:00.000  (0 / 0)')
        tk.Label(self._pb_frame, textvariable=self._pb_cur_time,
                 anchor='w', bg='white', fg='#1155cc', font=('Consolas', 8)
                 ).pack(fill='x', padx=2)

        # 재생 버튼
        btn_row = tk.Frame(self._pb_frame, bg='white')
        btn_row.pack(fill='x', pady=2)
        self._reset_btn = ttk.Button(btn_row, text='⏮ 처음', command=self._pb_reset,
                                     state='disabled')
        self._reset_btn.pack(side='left', padx=2)
        self._play_btn  = ttk.Button(btn_row, text='▶ Play', command=self._play,
                                     state='disabled')
        self._play_btn.pack(side='left', padx=2)
        self._pause_btn = ttk.Button(btn_row, text='⏸ Pause', command=self._pause,
                                     state='disabled')
        self._pause_btn.pack(side='left', padx=2)

        # 배속 선택
        spd_row = tk.Frame(self._pb_frame, bg='white')
        spd_row.pack(fill='x')
        tk.Label(spd_row, text='배속(speed)', bg='white').pack(side='left')
        for spd in [0.25, 0.5, 1.0, 2.0, 4.0, 8.0]:
            tk.Radiobutton(spd_row, text=f'{spd}×', variable=self._pb_speed, value=spd,
                           bg='white', activebackground='white').pack(side='left')

    # ── 모드 전환 ─────────────────────────────────────────────────────
    def _on_mode_change(self):
        # Simulator 모드 벗어날 때 웨이포인트 상태 초기화
        if self._mode.get() != 'simulator':
            if self._track._wp_record_mode:
                self._track.cancel_wp_record_mode()
            self._waypoints = []
            self._track.clear_waypoints()

        mode = self._mode.get()
        if mode == 'realtime':
            self._ctrl_frame.pack(fill='x', pady=2, padx=2)
            self._pb_frame.pack_forget()
            self._sim_frame.pack_forget()
        elif mode == 'playback':
            self._pb_frame.pack(fill='x', pady=2, padx=2)
            self._ctrl_frame.pack_forget()
            self._sim_frame.pack_forget()
        else:   # simulator
            self._sim_frame.pack(fill='x', pady=2, padx=2)
            self._ctrl_frame.pack_forget()
            self._pb_frame.pack_forget()

        # playback 은 녹화 데이터 재생 → 인지 파라미터 주입 비활성화 (값은 파싱본을 표시)
        st = 'disabled' if mode == 'playback' else 'normal'
        for w in self._perception_widgets:
            try:
                w.configure(state=st)
            except tk.TclError:
                pass

        # 모드 전환 시 유령 표시 리셋 (start 핸들러가 올바른 슬롯에 다시 설정)
        self._track.set_pi_ghost(None)
        self._track.set_sim_ghost(None)

    # ── 오브젝트 배치 ────────────────────────────────────────────────
    _OBJ_LABELS = {
        OBJ_OBSTACLE:  '🚧 장애물',
        OBJ_STOP_LINE: '🛑 정지선',
        OBJ_TURN_LEFT: '↰ 좌회전',
    }

    def _enter_obj_mode(self, kind: str):
        self._track.enter_place_mode(kind, on_cancel=self._on_obj_mode_cancel)
        self._obj_mode_var.set(
            f'배치 모드: {self._OBJ_LABELS[kind]}  |  클릭=배치  우클릭=취소')
        for k, btn in self._obj_btns.items():
            btn.state(['pressed'] if k == kind else ['!pressed'])

    def _on_obj_mode_cancel(self):
        self._obj_mode_var.set('')
        for btn in self._obj_btns.values():
            btn.state(['!pressed'])

    def _apply_object_effects(self):
        """Pi 차량 위치 기준 오브젝트 감지 → perception params 자동 갱신.
        감지 중일 때 override, 벗어나면 UI 슬라이더 값으로 복원."""
        pos = self._track.get_pi_world_pos()
        if pos is None:
            return
        fx = self._track.check_objects(*pos)

        # ── 장애물 (front_clear / dist_front_m)
        if fx['obstacle']:
            d = round(fx['dist_m'], 2)
            # simulator 모드 차선변경은 틱에서 차선 인지로 트리거; 그 외 모드는 시각화만
            if not self._lane_change_drawn and d <= 0.15:
                if self._mode.get() != 'simulator':
                    self._draw_lane_change_visualization()
                self._lane_change_drawn = True
            self._obstacle_active = True
            self._perception.params['front_clear']  = False
            self._perception.params['dist_front_m'] = d
            self._dist_val_lbl.config(text=f'{d * 100:.0f}cm')
        elif self._obstacle_active:
            self._obstacle_active   = False
            self._lane_change_drawn = False
            if not self._avoidance_mode:   # 회피 경로 추종 중이면 경로 유지
                self._track.clear_lane_change_path()
            fc  = self._scene_vars.get('front_clear')
            self._perception.params['front_clear']  = fc.get() if fc else True
            v = round(self._dist_val.get(), 2) if self._dist_detected.get() else None
            self._perception.params['dist_front_m'] = v
            self._dist_val_lbl.config(text='' if v is None else f'{v * 100:.0f}cm')

        # ── 정지선 (stop_signal)
        if fx['stop_line']:
            self._stop_line_active = True
            self._perception.params['stop_signal'] = True
        elif self._stop_line_active:
            self._stop_line_active = False
            ss = self._scene_vars.get('stop_signal')
            self._perception.params['stop_signal'] = ss.get() if ss else False

        # ── 좌회전 이정표 (lane_curvature_1pm)
        if fx['turn_left']:
            self._turn_sign_active = True
            self._perception.params['lane_curvature_1pm'] = 2.0   # +ve = 좌회전
        elif self._turn_sign_active:
            self._turn_sign_active = False
            curv = self._scene_vars.get('lane_curvature_1pm')
            self._perception.params['lane_curvature_1pm'] = round(curv.get(), 4) if curv else 0.0

    # ── 트랙 캔버스 ─────────────────────────────────────────────────
    def _clear_canvas(self):
        """궤적·차량 초기화. Simulator 모드면 시나리오(WP+오브젝트) 전체 초기화."""
        self._track.clear()
        if self._mode.get() == 'simulator':
            # 웨이포인트 메모리 + 캔버스
            if self._track._wp_record_mode:
                self._track.cancel_wp_record_mode()
                self._wp_rec_btn.state(['!pressed'])
                self._obj_mode_var.set('')
            self._waypoints          = []
            self._sim_dense_path     = []
            self._sim_path           = []
            self._sim_path_idx       = 0
            self._avoidance_mode     = False
            self._avoidance_path     = []
            self._avoidance_path_idx = 0
            self._track.clear_waypoints()
            self._track.clear_dense_path()
            self._track.clear_lane_change_path()
            # 오브젝트 전체 삭제
            self._track.clear_objects()
            self._sim_status_var.set('초기화됨 — WP 기록 후 경로 생성')

    def _set_default_pos(self):
        self._track.clear()
        self._track.set_start_pos(*DEFAULT_START)

    # ── 프로퍼티 ─────────────────────────────────────────────────────
    @property
    def is_running(self):
        return self._engine is not None and self._engine._started

    # ── Real-time ─────────────────────────────────────────────────────
    def _start_realtime(self):
        self._apply_network_settings()
        os.makedirs(LOG_ROOT, exist_ok=True)
        self._recorder.start('Realtime')
        # 받은 상대(실차) 패킷 → suffix 없음 / 자기(PC) 송신 패킷 → _pc
        self._engine = VILSEngine(self._role,
                                   on_packet_cb=lambda raw: self._recorder.log(raw, is_pc=False),
                                   on_hmac_fail_cb=self._recorder.log_hmac_fail,
                                   on_tx_cb=lambda raw: self._recorder.log(raw, is_pc=True))
        self._engine.start(self._perception)
        self._prev_pi_tx_abs = None
        self._start_time     = time.monotonic()
        self._track.reset_trail()
        # 수신 데이터로 주행하는 상대 차량(pi)=유령, 자기 ego(sim)=솔리드
        partner = 'leader' if self._role == 'follower' else 'follower'
        self._track.set_pi_ghost(f'{partner.upper()} (수신)')
        self._track.set_sim_ghost(None)
        self._start_btn.config(state='disabled')
        self._stop_btn.config(state='normal')
        self._status_var.set('실행 중...')
        self._tick()

    def _stop_realtime(self):
        if self._engine:
            self._engine.stop()
            self._engine = None
        self._track.set_pi_ghost(None)   # 유령 표시 해제
        names = self._recorder.stop()
        self._start_btn.config(state='normal')
        self._stop_btn.config(state='disabled')
        self._status_var.set('저장: ' + (', '.join(names) if names else '(없음)'))

    def _tick(self):
        if not self.is_running:
            return
        self._apply_object_effects()   # 오브젝트 감지 → perception params 반영
        self._engine.tick()
        self._sync_kv_kw()
        snap = self._engine.bus_snapshot()
        self._refresh_bus_labels(snap)
        if snap['pi_state']:
            pi = snap['pi_state']
            if self._prev_pi_tx_abs is not None:
                dt_ms = (pi.tx_abs - self._prev_pi_tx_abs) % 86_400_000
                dt = min(dt_ms / 1000.0, 0.5)
                self._track.update_pi(pi.throttle_pwm, pi.steer_pwm, dt)
            self._prev_pi_tx_abs = pi.tx_abs
        if snap['ego']:
            ego = snap['ego']
            self._track.update_sim(ego.throttle_pwm, ego.steer_pwm, TICK_MS / 1000.0)
        ego = snap.get('ego')
        self._push_monitor(time.monotonic() - self._start_time,
                           ego.throttle_pwm if ego else None,
                           ego.steer_pwm if ego else None,
                           ego.behavior if ego else None, snap)
        elapsed = int(time.monotonic() - self._start_time)
        fail = self._recorder.fail_count
        fail_str = f'  HMAC실패: {fail}' if fail else ''
        self._pkt_var.set(f'패킷: {self._recorder.packet_count}  경과: {elapsed}s{fail_str}')
        self.after(TICK_MS, self._tick)

    # ── Playback ──────────────────────────────────────────────────────
    def _open_bin(self):
        path = filedialog.askopenfilename(
            initialdir=LOG_ROOT,
            filetypes=[('Binary', '*.bin'), ('All', '*.*')],
            title='session.bin 선택')
        if not path:
            return
        self._load_bin_from_path(path)

    def _load_bin_from_path(self, path: str, label: str | None = None):
        """bin 파일을 재생 목록으로 로드. label 지정 시 파일명 표시 대신 사용."""
        key = config.load_key()
        self._playback_packets = []
        self._playback_raw     = []
        with open(path, 'rb') as f:
            while True:
                raw = f.read(PACKET_LEN)
                if len(raw) < PACKET_LEN:
                    break
                try:
                    parsed = packet_parser(raw, key)
                except ValueError:
                    continue
                self._playback_packets.append(parsed)
                self._playback_raw.append(raw)   # 원본 바이트 그대로 보관 (재송신용)
        self._pb_idx     = 0
        self._pb_running = False
        self._track.clear()
        # 출발 자세: bin 옆 meta.json 에서 복원 (녹화 시 저장) → 동일 경로 재현
        self._pb_start_pose = None
        meta_path = os.path.join(os.path.dirname(path), 'meta.json')
        if os.path.exists(meta_path):
            try:
                import json
                with open(meta_path, encoding='utf-8') as mf:
                    st = json.load(mf).get('start')
                if st and len(st) == 3:
                    self._pb_start_pose = list(st)
            except Exception:
                self._pb_start_pose = None
        self._pb_set_start()   # 출발 자세(meta) 또는 차선 하단에 배치
        self._pb_fname.set(label if label else os.path.basename(path))
        n = len(self._playback_packets)
        if n > 0:
            t0 = fmt_ms_of_day(self._playback_packets[0].tx_abs)
            t1 = fmt_ms_of_day(self._playback_packets[-1].tx_abs)
            dur = (self._playback_packets[-1].tx_abs - self._playback_packets[0].tx_abs
                   ) % 86_400_000 / 1000.0
            self._pb_range.set(f'{t0}  →  {t1}  ({dur:.1f}s)')
            self._pb_progress['maximum'] = n
            self._pb_progress['value']   = 0
            self._pb_cur_time.set(f'{t0}  (0 / {n})')
            self._play_btn.config(state='normal')
            self._reset_btn.config(state='normal')
        else:
            self._pb_range.set('패킷 없음')

    def _play(self):
        if not self._playback_packets:
            return
        # UDP 재송신 소켓 열기 (저장 데이터를 그대로 다시 쏨)
        self._apply_network_settings()
        import socket as _socket
        cfg = config.for_role(self._role)
        if self._pb_udp_sock is None:
            self._pb_udp_sock = _socket.socket(_socket.AF_INET, _socket.SOCK_DGRAM)
        self._pb_peer = (cfg['peer_ip'], cfg['peer_port'])
        if self._pb_idx == 0:
            self._pb_tx_ok = self._pb_tx_fail = 0
        self._pb_running = True
        self._play_btn.config(state='disabled')
        self._pause_btn.config(state='normal')
        self._pb_step()

    def _pause(self):
        self._pb_running = False
        if self._pb_udp_sock is not None:
            try: self._pb_udp_sock.close()
            except Exception: pass
            self._pb_udp_sock = None
        self._play_btn.config(state='normal')
        self._pause_btn.config(state='disabled')

    def _pb_set_start(self):
        """Playback 차량 시작 위치 — 녹화 시 저장된 출발 자세(meta) 우선, 없으면 차선 하단."""
        if not self._playback_packets:
            return
        if getattr(self, '_pb_start_pose', None):
            self._track.set_start_pos(*self._pb_start_pose)   # 녹화와 동일 자세
            return
        lane = self._playback_packets[0].lane
        r = {1: 0.925, 2: 1.125}.get(lane, 0.925)   # 차선 중앙 반경
        self._track.set_start_pos(0.0, -r, 0.0)

    def _pb_reset(self):
        self._pb_running = False
        if self._pb_udp_sock is not None:
            try: self._pb_udp_sock.close()
            except Exception: pass
            self._pb_udp_sock = None
        self._pb_idx     = 0
        self._track.clear()
        self._pb_set_start()   # 시작점으로 차량 재배치
        self._play_btn.config(state='normal')
        self._pause_btn.config(state='disabled')
        n = len(self._playback_packets)
        self._pb_progress['value'] = 0
        t0 = fmt_ms_of_day(self._playback_packets[0].tx_abs) if n > 0 else '—'
        self._pb_cur_time.set(f'{t0}  (0 / {n})')

    def _pb_step(self):
        if not self._pb_running:
            return
        n = len(self._playback_packets)
        if self._pb_idx >= n:
            self._pause()
            t1 = fmt_ms_of_day(self._playback_packets[-1].tx_abs)
            self._pb_cur_time.set(f'{t1}  ✓ 재생 완료')
            self._pb_progress['value'] = n
            return

        self._apply_object_effects()   # 오브젝트 감지
        self._sync_kv_kw()
        pkt = self._playback_packets[self._pb_idx]
        # 저장된 원본 60B 를 그대로 UDP 재송신 (seq·tx_abs 원본 보존)
        if self._pb_udp_sock is not None and self._pb_idx < len(self._playback_raw):
            try:
                self._pb_udp_sock.sendto(self._playback_raw[self._pb_idx], self._pb_peer)
                self._pb_tx_ok += 1
            except OSError:
                self._pb_tx_fail += 1
        # 모션 적분 dt = 공칭 제어주기(50ms) 고정 — 시뮬레이터/Pi 가 매 틱 dt=0.05 로
        # 적분했으므로 동일하게 0.05 로 재생해야 경로가 정확히 일치한다.
        # (tx_abs(벽시계) 간격을 쓰면 ~55~70ms jitter 가 open-loop 재생에서 누적돼 트랙 이탈)
        dt = TICK_MS / 1000.0
        self._track.update_pi(pkt.throttle_pwm, pkt.steer_pwm, dt)
        self._pb_idx += 1

        # 파싱한 실제 값을 UI 에 반영 (값 변화 확인용)
        self._refresh_playback_labels(pkt)

        # UI 갱신
        self._pb_progress['value'] = self._pb_idx
        cur_t = fmt_ms_of_day(pkt.tx_abs)
        tx_info = f'  TX:{self._pb_tx_ok}' + (f' 실패:{self._pb_tx_fail}' if self._pb_tx_fail else '')
        self._pb_cur_time.set(f'{cur_t}  ({self._pb_idx} / {n}){tx_info}')

        # 다음 패킷까지 원본 tx 간격으로 스케줄 (tx 타이밍 보존), 배속 적용
        if self._pb_idx < n:
            gap_ms = (self._playback_packets[self._pb_idx].tx_abs - pkt.tx_abs) % 86_400_000
            if gap_ms <= 0 or gap_ms > 2000:   # 이상치 폴백
                gap_ms = TICK_MS
        else:
            gap_ms = TICK_MS
        delay = max(1, int(gap_ms / self._pb_speed.get()))
        self.after(delay, self._pb_step)

    def _refresh_playback_labels(self, pkt):
        """재생 중 파싱한 패킷 값을 UI(모션·판단·V2V·인지 차선)에 표시."""
        lbl = self._bus_labels
        lbl['ego_throttle'].set(f'{pkt.throttle_pwm:.3f}')
        lbl['ego_steer'].set(f'{pkt.steer_pwm:.3f}')
        lbl['cmd_behavior'].set(pkt.behavior.name)
        lbl['pi_seq'].set(str(pkt.seq))
        lbl['pi_lane'].set(str(pkt.lane))
        lbl['pi_throttle'].set(f'{pkt.throttle_pwm:.3f}')
        lbl['pi_steer'].set(f'{pkt.steer_pwm:.3f}')
        # 인지 현재차선도 파싱값으로 표시 (입력은 비활성, 표시는 갱신)
        if pkt.lane in (0, 1, 2) and 'current_lane' in self._scene_vars:
            self._scene_vars['current_lane'].set(pkt.lane)

    # ── Simulator 시나리오 저장/불러오기 (웨이포인트 + 오브젝트) ───────
    def _sim_save_scenario(self):
        import json
        if not self._waypoints and not self._track.get_objects_data():
            messagebox.showinfo('저장', '웨이포인트 또는 오브젝트가 없습니다.', parent=self)
            return
        path = filedialog.asksaveasfilename(
            initialdir=LOG_ROOT,
            defaultextension='.json',
            filetypes=[('JSON', '*.json'), ('All', '*.*')],
            title='시나리오 저장')
        if not path:
            return
        data = {
            'waypoints': [{'wx': p[0], 'wy': p[1]} for p in self._waypoints],
            'objects':   self._track.get_objects_data(),
        }
        with open(path, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        self._sim_status_var.set(f'저장됨: {os.path.basename(path)}')

    def _sim_load_scenario(self):
        import json
        path = filedialog.askopenfilename(
            initialdir=LOG_ROOT,
            filetypes=[('JSON', '*.json'), ('All', '*.*')],
            title='시나리오 불러오기')
        if not path:
            return
        try:
            with open(path, encoding='utf-8') as f:
                data = json.load(f)
        except Exception as e:
            messagebox.showerror('오류', str(e), parent=self)
            return
        wps = [(p['wx'], p['wy']) for p in data.get('waypoints', [])]
        self._waypoints = wps
        self._track.draw_waypoints(wps)
        self._track.set_objects_data(data.get('objects', []))
        n_wp = len(wps)
        n_ob = len(data.get('objects', []))
        self._sim_status_var.set(f'불러옴: WP {n_wp}개, 오브젝트 {n_ob}개')

    # ── Simulator 실행 루프 ─────────────────────────────────────────
    # ── 출발점 지정 → CCW 자동 주행 경로 ──────────────────────────────
    def _pick_start_point(self):
        """출발점 지정 모드 진입 — 클릭한 위치의 차선을 따라 CCW 주행 경로를 만든다."""
        self._track.enter_start_pick_mode(on_pick=self._on_start_point_picked)
        self._sim_status_var.set('출발점 지정: 트랙 위 한 곳을 클릭하세요 (시계 반대방향 주행)')

    def _on_start_point_picked(self, wx, wy):
        """클릭 위치 → 차선 자동 감지 → 양쪽 차선 CCW dense 경로 생성. 장애물 시 반대 차선 전환용."""
        lane = self._detect_vehicle_lane(wx, wy)
        self._sim_lane = lane
        self._perception.params['current_lane'] = lane
        # 두 차선 경로 모두 생성 (같은 출발점 기준 정렬) → 장애물 감지 시 반대 차선으로 영구 전환
        self._lane_paths = {
            1: stadium_lane_path(wx, wy, 1, step=0.05),
            2: stadium_lane_path(wx, wy, 2, step=0.05),
        }
        path = self._lane_paths[lane]
        self._sim_dense_path = path
        self._sim_path_idx   = 0
        self._lane_change_cooldown = 0
        x0, y0 = path[0]
        x1, y1 = path[1]
        h0 = math.atan2(y1 - y0, x1 - x0)
        self._track.set_start_pos(x0, y0, h0)
        self._track.draw_dense_path(path)
        self._sim_status_var.set(
            f'출발점 설정 (차선 {lane}, {len(path)}점) — ▶ Start 가능')

    def _start_simulator(self):
        if not self._sim_dense_path:
            messagebox.showwarning('Simulator', '먼저 📍 출발점 지정을 눌러주세요.', parent=self)
            return
        # 경로 첫 점으로 차량을 재배치 (헤딩 = 진행 방향)
        x0, y0 = self._sim_dense_path[0]
        if len(self._sim_dense_path) > 1:
            x1, y1 = self._sim_dense_path[1]
            h0 = math.atan2(y1 - y0, x1 - x0)
        else:
            h0 = 0.0
        self._track.set_start_pos(x0, y0, h0)
        self._sim_path_idx       = 0
        self._sim_running        = True
        self._obstacle_active    = False
        self._stop_line_active   = False
        self._lane_change_drawn  = False
        self._avoidance_mode     = False
        self._avoidance_path     = []
        self._avoidance_path_idx = 0
        # UDP TX 소켓 열기
        self._apply_network_settings()
        import socket as _socket
        cfg = config.for_role(self._role)
        if self._sim_udp_sock:
            try: self._sim_udp_sock.close()
            except Exception: pass
        self._sim_udp_sock = _socket.socket(_socket.AF_INET, _socket.SOCK_DGRAM)
        self._sim_peer     = (cfg['peer_ip'], cfg['peer_port'])
        self._sim_key      = config.load_key()
        self._sim_seq      = 0
        self._sim_tx_ok    = 0
        self._sim_tx_fail  = 0
        self._sim_last_tx  = None
        # 60B 로깅 시작 (자기 송신=_pc, 실차 수신=suffix 없음)
        self._sim_last_rx = None
        self._recorder.start('Simulator')
        # playback 시 동일 경로 재현용 출발 자세 저장 (bin 옆 meta.json)
        self._recorder.set_meta({'start': [x0, y0, h0]})
        self._start_sim_rx(cfg)   # 실차에서 돌아오는 상대 데이터 수신·로깅
        # 자기 주행 차량(pi)=솔리드, 수신 상대 차량(sim)=유령
        partner = 'leader' if self._role == 'follower' else 'follower'
        self._track.set_pi_ghost(None)
        self._track.set_sim_ghost(f'{partner.upper()} (수신)')
        self._sim_start_btn.config(state='disabled')
        self._sim_stop_btn.config(state='normal')
        self._sim_status_var.set(f'실행 중... → {self._sim_peer}')
        self._simulator_tick()

    def _start_sim_rx(self, cfg):
        """시뮬레이터 모드 RX 스레드 — 실차(상대)에서 돌아오는 패킷을 받아 검증·로깅."""
        import socket as _socket, threading
        self._sim_rx_stop = threading.Event()
        self._sim_rx_sock = _socket.socket(_socket.AF_INET, _socket.SOCK_DGRAM)
        self._sim_rx_sock.setsockopt(_socket.SOL_SOCKET, _socket.SO_REUSEADDR, 1)
        try:
            self._sim_rx_sock.bind(('0.0.0.0', cfg['rx_port']))
        except OSError as e:
            print(f'[sim] RX bind 실패 :{cfg["rx_port"]}: {e}', file=sys.stderr)
            self._sim_rx_sock.close()
            self._sim_rx_sock = None
            return
        self._sim_rx_sock.settimeout(0.5)
        self._sim_rx_thread = threading.Thread(target=self._sim_rx_loop,
                                               name='sim-rx', daemon=True)
        self._sim_rx_thread.start()

    def _sim_rx_loop(self):
        import socket as _socket
        from core_module.v2v import packet_parser
        while not self._sim_rx_stop.is_set():
            try:
                data, _addr = self._sim_rx_sock.recvfrom(2048)
            except _socket.timeout:
                continue
            except OSError:
                break   # 소켓 close → 종료
            try:
                state = packet_parser(data, self._sim_key)   # HMAC 검증
            except ValueError:
                self._recorder.log_hmac_fail(data)
                continue
            self._sim_last_rx = state                # 모니터 통신 표시용
            self._recorder.log(data, is_pc=False)   # 실차 상대 데이터 (suffix 없음)

    def _stop_sim_rx(self):
        if self._sim_rx_stop:
            self._sim_rx_stop.set()
        if self._sim_rx_sock:
            try: self._sim_rx_sock.close()
            except Exception: pass
        if self._sim_rx_thread and self._sim_rx_thread.is_alive():
            self._sim_rx_thread.join(timeout=1.0)
        self._sim_rx_sock = self._sim_rx_thread = self._sim_rx_stop = None

    def _stop_simulator(self):
        self._sim_running    = False
        self._avoidance_mode = False
        if self._sim_udp_sock:
            try: self._sim_udp_sock.close()
            except Exception: pass
            self._sim_udp_sock = None
        self._stop_sim_rx()
        self._track.set_sim_ghost(None)   # 유령 표시 해제
        names = self._recorder.stop()   # 60B 로그 마감 + 폴더 리네임
        self._sim_start_btn.config(state='normal')
        self._sim_stop_btn.config(state='disabled')
        n = len(self._sim_dense_path)
        saved = ', '.join(names) if names else '(없음)'
        self._sim_status_var.set(f'정지 ({self._sim_path_idx} / {n} 스텝) — 저장: {saved}')

    # ── Pure Pursuit 피드백 컨트롤러 ─────────────────────────────────
    _PP_LOOKAHEAD = 5   # 전방 몇 스텝 앞 점을 목표로 할지

    def _simulator_tick(self):
        if not self._sim_running:
            return

        path = self._sim_dense_path
        n    = len(path)

        pos = self._track._pi_state
        if pos is None:
            self._stop_simulator()
            return
        cx, cy, heading = pos

        self._apply_object_effects()   # 오브젝트 감지 → perception params 갱신
        self._sync_kv_kw()
        kw = self._scene_vars['__kw'].get() if '__kw' in self._scene_vars else 2.0
        dt = TICK_MS / 1000.0

        # ── 차선 변경 트리거: 현재 차선 전방 장애물 감지 시 반대 차선으로 영구 전환 ──
        if self._lane_change_cooldown > 0:
            self._lane_change_cooldown -= 1
        if (not self._avoidance_mode and self._lane_change_cooldown == 0
                and self._obstacle_ahead_in_lane() is not None):
            self._start_lane_change()

        # ── 경로 선택: 회피 중 → 회피 경로 / 평시 → 원본 경로 ────────
        if self._avoidance_mode and self._avoidance_path:
            av   = self._avoidance_path
            av_n = len(av)

            # 통과한 회피 경로점 건너뜀
            while self._avoidance_path_idx < av_n - 1:
                d = math.hypot(av[self._avoidance_path_idx][0] - cx,
                               av[self._avoidance_path_idx][1] - cy)
                if d < 0.08:
                    self._avoidance_path_idx += 1
                else:
                    break

            if (self._avoidance_path_idx >= av_n - 1
                    and math.hypot(av[-1][0] - cx, av[-1][1] - cy) < 0.12):
                # 차선 전환 완료 → 활성 경로를 반대 차선으로 교체하고 크루즈 복귀
                self._sim_lane = self._lane_change_target
                self._perception.params['current_lane'] = self._sim_lane
                self._sim_dense_path = self._lane_paths[self._sim_lane]
                path = self._sim_dense_path
                n    = len(path)
                best, min_d = 0, float('inf')
                for j in range(n):
                    d = math.hypot(path[j][0] - cx, path[j][1] - cy)
                    if d < min_d:
                        min_d, best = d, j
                self._sim_path_idx   = best
                self._avoidance_mode = False
                self._avoidance_path_idx = 0
                self._avoidance_path = []
                self._track.clear_lane_change_path()
                self._lane_change_cooldown = 25   # 재트리거 방지 (~1.25s)

            if self._avoidance_mode:
                la_idx = min(self._avoidance_path_idx + self._PP_LOOKAHEAD, av_n - 1)
                tx, ty = av[la_idx]
            else:
                la_idx = min(self._sim_path_idx + self._PP_LOOKAHEAD, n - 1)
                tx, ty = path[la_idx]
        else:
            # ── 원본 경로 추종 (닫힌 루프 → 연속 CCW 주행, wrap-around) ──
            skips = 0
            while skips < n:
                j = self._sim_path_idx % n
                dist = math.hypot(path[j][0] - cx, path[j][1] - cy)
                if dist < 0.08:
                    self._sim_path_idx = (self._sim_path_idx + 1) % n
                    skips += 1
                else:
                    break

            la_idx = (self._sim_path_idx + self._PP_LOOKAHEAD) % n
            tx, ty = path[la_idx]

        # ── Pure Pursuit → steer / throttle ──────────────────────────
        dx, dy   = tx - cx, ty - cy
        target_h = math.atan2(dy, dx)
        dh       = target_h - heading
        while dh >  math.pi: dh -= 2 * math.pi
        while dh < -math.pi: dh += 2 * math.pi

        steer    = max(-0.5, min(0.5, dh / (kw * dt)))
        throttle = max(-1.0, min(1.0, self._sim_throttle.get()))

        from messages import DriveBehavior, EgoState, Role
        behavior = DriveBehavior.CRUISE
        if self._avoidance_mode:
            behavior = DriveBehavior.LANE_CHANGE
        if self._stop_line_active:
            throttle = 0.0
            behavior = DriveBehavior.STOP

        self._track.update_pi(throttle, steer, dt)

        # 수신한 상대(예: follower) 데이터로 유령 차량(sim) 구동
        if self._sim_last_rx is not None:
            rx = self._sim_last_rx
            self._track.update_sim(rx.throttle_pwm, rx.steer_pwm, dt)

        # ── 60B 패킷: 실제 Pi와 동일하게 50ms(20Hz) 고정 주기로 생성·로깅·송신 ──
        #    (배속은 화면 시각화에만 적용 — V2V 출력 레이트는 실차와 같아야 함)
        now = time.monotonic()
        if self._sim_last_tx is None or (now - self._sim_last_tx) >= 0.049:
            self._sim_last_tx = now
            self._sim_seq = (self._sim_seq + 1) & 0xFFFF
            role_enum = Role.LEADER if self._role == 'leader' else Role.FOLLOWER
            cur_lane  = self._perception.params.get('current_lane', self._sim_lane)
            try:
                pkt = packet_generator(
                    EgoState(stamp=now, throttle_pwm=throttle,
                             steer_pwm=steer, behavior=behavior),
                    lane=cur_lane, role=role_enum,
                    seq=self._sim_seq, key=self._sim_key,
                )
                self._recorder.log(pkt, is_pc=True)   # 자기(PC) leader 데이터 → leader_pc.bin
            except Exception as e:
                pkt = None
                print(f'[sim] 패킷 생성 실패 seq={self._sim_seq}: {type(e).__name__}: {e}',
                      file=sys.stderr)
            if pkt is not None and self._sim_udp_sock is not None:
                try:
                    self._sim_udp_sock.sendto(pkt, self._sim_peer)
                    self._sim_tx_ok += 1
                except OSError as e:
                    self._sim_tx_fail += 1
                    if self._sim_tx_fail <= 3 or self._sim_tx_fail % 50 == 0:
                        print(f'[sim] UDP 송신 실패 x{self._sim_tx_fail} → {self._sim_peer}: {e}',
                              file=sys.stderr)

        # ── 버스 라벨 업데이트 ─────────────────────────────────────────
        lbl = self._bus_labels
        lbl['ego_throttle'].set(f'{throttle:.3f}')
        lbl['ego_steer'].set(f'{steer:.3f}')
        lbl['cmd_behavior'].set(behavior.name)

        self._push_monitor(self._sim_seq * 0.05, throttle, steer, behavior)

        tx_info = (f' TX성공:{self._sim_tx_ok}' +
                   (f' 실패:{self._sim_tx_fail}' if self._sim_tx_fail else '')) if self._sim_udp_sock else ''
        log_info = f' 로그:{self._recorder.packet_count}'
        if self._avoidance_mode:
            self._sim_status_var.set(
                f'회피 중 {self._avoidance_path_idx}/{len(self._avoidance_path)}{tx_info}{log_info}')
        else:
            self._sim_status_var.set(f'{self._sim_path_idx} / {n} 스텝{tx_info}{log_info}')

        delay = max(1, int(TICK_MS / self._sim_speed.get()))
        self.after(delay, self._simulator_tick)

    # ── 웨이포인트 ───────────────────────────────────────────────────
    def _toggle_wp_record_mode(self):
        """웨이포인트 기록 모드 ON/OFF."""
        if self._track._wp_record_mode:
            self._track.cancel_wp_record_mode()
            self._wp_rec_btn.state(['!pressed'])
            self._obj_mode_var.set('')
        else:
            self._track.enter_wp_record_mode(on_done=self._on_wp_record_done)
            self._wp_rec_btn.state(['pressed'])
            self._obj_mode_var.set('📍 웨이포인트 기록 중  |  좌클릭=추가  우클릭=완료')

    def _on_wp_record_done(self, waypoints: list):
        """기록 완료 콜백 — 저장 후 캔버스에 표시."""
        self._wp_rec_btn.state(['!pressed'])
        self._obj_mode_var.set('')
        if not waypoints:
            return
        import json
        path = filedialog.asksaveasfilename(
            initialdir=LOG_ROOT,
            defaultextension='.json',
            filetypes=[('JSON', '*.json'), ('All', '*.*')],
            title='웨이포인트 저장')
        if path:
            with open(path, 'w') as f:
                json.dump({'waypoints': [{'wx': p[0], 'wy': p[1]} for p in waypoints]}, f, indent=2)
        self._waypoints = waypoints
        self._track.draw_waypoints(waypoints)

    def _clear_waypoints(self):
        self._waypoints      = []
        self._sim_dense_path = []
        self._sim_path       = []
        self._track.clear_waypoints()
        self._track.clear_dense_path()

    def _generate_bin_from_waypoints(self):
        """웨이포인트 → 경로 보간 → throttle/steer 역산 → bin → 재생 목록 로드."""
        import tempfile
        from core_module.v2v import packet_generator
        from core_module.config import load_key
        from messages import EgoState, DriveBehavior, Role

        if len(self._waypoints) < 2:
            messagebox.showwarning('웨이포인트', '웨이포인트가 2개 이상 필요합니다.', parent=self)
            return

        kv = self._scene_vars['__kv'].get() if '__kv' in self._scene_vars else 1.0
        kw = self._scene_vars['__kw'].get() if '__kw' in self._scene_vars else 2.0
        dt = TICK_MS / 1000.0
        step = kv * dt   # 스텝당 이동 거리(m), 기본 0.05 m

        def _adiff(a, b):
            d = a - b
            while d >  math.pi: d -= 2 * math.pi
            while d < -math.pi: d += 2 * math.pi
            return d

        # ── 1. 웨이포인트 사이 선형 보간 → dense path ─────────────────
        dense = [self._waypoints[0]]
        for i in range(len(self._waypoints) - 1):
            x0, y0 = self._waypoints[i]
            x1, y1 = self._waypoints[i + 1]
            seg = math.hypot(x1 - x0, y1 - y0)
            n = max(1, round(seg / step))
            for j in range(1, n + 1):
                t = j / n
                dense.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t))

        # ── 2. 각 점의 heading 계산 ───────────────────────────────────
        headings = [math.atan2(dense[i+1][1] - dense[i][1],
                               dense[i+1][0] - dense[i][0])
                    for i in range(len(dense) - 1)]
        headings.append(headings[-1])

        # ── 3. throttle / steer 역산 → bin 버퍼 작성 ─────────────────
        key = load_key()
        buf = bytearray()
        t_base_ms = 0   # tx_abs: 0ms 기준 (재생 시 dt 계산에 사용)
        for i in range(len(dense) - 1):
            ds = math.hypot(dense[i+1][0] - dense[i][0],
                            dense[i+1][1] - dense[i][1])
            throttle = max(0.0, min(1.0, ds / (kv * dt)))
            dh = _adiff(headings[i + 1], headings[i])
            steer = max(-1.0, min(1.0, dh / (kw * dt)))
            ego = EgoState(
                stamp        = i * dt,
                throttle_pwm = throttle,
                steer_pwm    = steer,
                behavior     = DriveBehavior.CRUISE,
            )
            pkt = packet_generator(ego, lane=1, role=Role.LEADER,
                                   seq=i & 0xFFFF, key=key)
            buf.extend(pkt)

        # ── 4. sim_dense_path 저장 + 캔버스에 경로 표시 ─────────────
        self._sim_dense_path = list(dense)
        self._sim_path_idx   = 0
        self._track.draw_dense_path(dense)

        # sim_path 저장 (Playback bin 생성용)
        self._sim_path = [(t, s) for t, s in zip(
            [max(0.0, min(1.0, math.hypot(dense[i+1][0]-dense[i][0],
                                          dense[i+1][1]-dense[i][1]) / (kv * dt)))
             for i in range(len(dense) - 1)],
            [max(-1.0, min(1.0, _adiff(headings[i+1], headings[i]) / (kw * dt)))
             for i in range(len(dense) - 1)],
        )]
        self._sim_path_idx = 0

        # ── 5. 임시 bin 파일 저장 후 재생 목록 로드 (Playback 모드용) ─
        tmp = tempfile.NamedTemporaryFile(suffix='_wp.bin', delete=False, dir=LOG_ROOT)
        tmp.write(buf)
        tmp.close()
        n_wp  = len(self._waypoints)
        n_stp = len(dense) - 1
        label = f'[WP {n_wp}개 → {n_stp}스텝]'
        if self._mode.get() == 'playback':
            self._load_bin_from_path(tmp.name, label=label)
        else:
            self._sim_status_var.set(f'경로 생성 완료: {n_stp}스텝 — ▶ Start 가능')

    def _load_waypoints(self):
        """웨이포인트 JSON 파일 불러와 캔버스에 빨간 원·선으로 표시."""
        import json
        path = filedialog.askopenfilename(
            initialdir=LOG_ROOT,
            filetypes=[('JSON', '*.json'), ('All', '*.*')],
            title='웨이포인트 불러오기')
        if not path:
            return
        try:
            with open(path) as f:
                data = json.load(f)
            wps = [(p['wx'], p['wy']) for p in data.get('waypoints', [])]
            self._waypoints = wps
            self._track.draw_waypoints(wps)
        except Exception as e:
            messagebox.showerror('웨이포인트 오류', str(e), parent=self)

    def _compute_lane_change_path(self, wx, wy, heading):
        """현재 위치·헤딩에서 인접 차선으로의 코사인-보간 경로 반환."""
        nx, ny = _track_perpendicular(wx, wy)
        cur_lane = self._perception.params.get('current_lane', 1)
        # 법선은 항상 inward(중심 방향). 차선 1(내측)→2(외측): 법선 반대(-), 2→1: 법선 방향(+)
        lateral_shift = -0.20 if cur_lane == 1 else 0.20
        N = 40
        fwd_total = 0.50   # 전진 거리 (m)
        cos_h, sin_h = math.cos(heading), math.sin(heading)
        path = []
        for i in range(N + 1):
            t = i / N
            fwd = fwd_total * t
            lat = lateral_shift * (1 - math.cos(math.pi * t)) / 2   # S커브
            path.append((wx + cos_h * fwd + nx * lat,
                         wy + sin_h * fwd + ny * lat))
        return path

    def _draw_lane_change_visualization(self):
        """Pi 현재 위치 기준 차선 변경 계획 경로를 캔버스에 표시."""
        pos = self._track._pi_state
        if pos is None:
            return
        wx, wy, heading = pos
        path = self._compute_lane_change_path(wx, wy, heading)
        self._track.draw_lane_change_path(path)

    def _detect_vehicle_lane(self, wx, wy) -> int:
        """차량 세계좌표로 현재 차선 자동 감지. 1=내측(황선 안쪽), 2=외측(황선 바깥)."""
        cx = _STRAIGHT if wx > _STRAIGHT else (-_STRAIGHT if wx < -_STRAIGHT else wx)
        r  = math.hypot(wx - cx, wy)   # 곡률 중심 기준 반경
        return 2 if r > 1.025 else 1   # 1.025 = 노란선 반경

    _LANE_CHANGE_TRIGGER_M = 0.70   # 현재 차선 전방 이 거리 안 장애물이면 차선 변경(전환 완료 여유 확보)

    def _obstacle_ahead_in_lane(self):
        """현재 차선(_sim_lane) 전방 _LANE_CHANGE_TRIGGER_M 안에 장애물이 있으면 거리 반환, 없으면 None."""
        pos = self._track._pi_state
        if pos is None:
            return None
        cx, cy, h = pos
        fx, fy = math.cos(h), math.sin(h)
        best = None
        for obj in self._track._objects:
            if obj.kind != OBJ_OBSTACLE:
                continue
            dx, dy = obj.wx - cx, obj.wy - cy
            if dx * fx + dy * fy <= 0:          # 차량 뒤쪽 → 무시
                continue
            d = math.hypot(dx, dy)
            if d > self._LANE_CHANGE_TRIGGER_M:  # 너무 멀음
                continue
            if self._detect_vehicle_lane(obj.wx, obj.wy) != self._sim_lane:  # 다른 차선
                continue
            if best is None or d < best:
                best = d
        return best

    def _start_lane_change(self):
        """현재 차선 → 반대 차선으로 영구 전환(복귀 없음). 전환 경로 추종 모드로 전환."""
        pos = self._track._pi_state
        if pos is None:
            return
        self._lane_change_target = 2 if self._sim_lane == 1 else 1
        self._avoidance_path     = self._compute_lane_change_transition()
        self._avoidance_path_idx = 0
        self._avoidance_mode     = True
        self._track.draw_lane_change_path(self._avoidance_path)

    def _compute_lane_change_transition(self):
        """현재 차선 dense 경로를 반대 차선으로 측방 이동하는 일방향 S커브 경로(복귀 없음).
        트랙 곡률을 따라가므로 곡선에서도 트랙을 벗어나지 않는다."""
        path = self._sim_dense_path
        n = len(path)
        if n < 2:
            return []
        cx, cy, _ = self._track._pi_state
        # 현재→반대 차선 측방 오프셋: 내측(1)→외측 = -inward, 외측(2)→내측 = +inward
        s = -0.20 if self._sim_lane == 1 else 0.20
        start = min(range(n), key=lambda i: (path[i][0] - cx) ** 2 + (path[i][1] - cy) ** 2)
        N = 12   # ×0.05m ≈ 0.6m 전환 구간 (트리거 0.7m 안에 완료)
        av = []
        for k in range(N + 1):
            px, py = path[(start + k) % n]
            nx, ny = _track_perpendicular(px, py)
            prof = (1 - math.cos(math.pi * k / N)) / 2   # 0→1 S커브 (복귀 없음)
            av.append((px + nx * s * prof, py + ny * s * prof))
        return av

    # ── 시나리오 ─────────────────────────────────────────────────────
    def _open_scenario_editor(self):
        ed = ScenarioEditor(self, self._scenario_steps)
        self.wait_window(ed)
        n = len(self._scenario_steps)
        self._sc_status.set(f'시나리오: {n}단계' if n else '시나리오: 없음')

    def _apply_scenario_step(self, step: ScenarioStep):
        """시나리오 단계를 실행 — perception params + UI 슬라이더 동기화."""
        v = step.value
        if step.param in ('__kv', '__kw'):
            if step.param in self._scene_vars:
                self._scene_vars[step.param].set(float(v))
        elif step.param == 'dist_front_m':
            if v is None:
                self._dist_detected.set(False)
            else:
                self._dist_detected.set(True)
                self._dist_val.set(float(v))
            self._perception.params['dist_front_m'] = v
        elif step.param in PARAM_BOOL:
            b = bool(v)
            self._perception.params[step.param] = b
            if step.param in self._scene_vars:
                self._scene_vars[step.param].set(b)
        elif step.param == 'current_lane':
            i = int(v)
            self._perception.params['current_lane'] = i
            if 'current_lane' in self._scene_vars:
                self._scene_vars['current_lane'].set(i)
        else:
            f = float(v)
            self._perception.params[step.param] = round(f, 4)
            if step.param in self._scene_vars:
                self._scene_vars[step.param].set(f)

    # ── 공통 유틸 ────────────────────────────────────────────────────
    def _sync_kv_kw(self):
        kv = self._scene_vars.get('__kv')
        kw = self._scene_vars.get('__kw')
        if kv:
            self._track.k_v = kv.get()
        if kw:
            self._track.k_w = kw.get()

    def _refresh_bus_labels(self, snap):
        lbl = self._bus_labels
        if snap.get('command'):
            lbl['cmd_behavior'].set(snap['command'].behavior.name)
        if snap.get('mode'):
            lbl['mode'].set(snap['mode'].mode.name)
        if snap.get('ego'):
            lbl['ego_throttle'].set(f"{snap['ego'].throttle_pwm:.3f}")
            lbl['ego_steer'].set(f"{snap['ego'].steer_pwm:.3f}")
        if snap.get('pi_state'):
            pi = snap['pi_state']
            lbl['pi_seq'].set(str(pi.seq))
            lbl['pi_lane'].set(str(pi.lane))
            lbl['pi_throttle'].set(f'{pi.throttle_pwm:.3f}')
            lbl['pi_steer'].set(f'{pi.steer_pwm:.3f}')
        if snap.get('link'):
            lbl['link_state'].set(snap['link'].state.name)
            lbl['link_age'].set(f"{snap['link'].age_rx:.0f}ms")

    # ── View 메뉴: 실시간 모니터 ──────────────────────────────────────
    def open_live_monitor(self, LiveMonitorWindow):
        """이 역할 탭의 실시간 버스 모니터 창을 연다 (이미 열려있으면 앞으로).
        LiveMonitorWindow 클래스는 호출측(App)에서 지연 임포트해 전달."""
        if self._live_monitor is not None and self._live_monitor.winfo_exists():
            self._live_monitor.lift()
            return
        self._live_monitor = LiveMonitorWindow(
            self, title=self._role.capitalize(),
            on_close=lambda: setattr(self, '_live_monitor', None))

    def _build_monitor_sample(self, throttle, steer, behavior, snap=None):
        """4개 서브시스템(인지·판단·모션·통신) 현재값을 모니터용 dict 로 모은다."""
        p = self._perception.params
        s = {
            'current_lane': p.get('current_lane'),
            'offset_m':     p.get('lane_offset_m'),
            'heading_rad':  p.get('lane_heading_rad'),
            'curvature':    p.get('lane_curvature_1pm'),
            'front_clear':  p.get('front_clear'),
            'stop_signal':  p.get('stop_signal'),
            'throttle':     throttle,
            'steer':        steer,
            'behavior':     int(behavior) if behavior is not None else None,
        }
        if snap:   # realtime: 엔진 버스 스냅샷
            if snap.get('mode') is not None:
                s['mode'] = int(snap['mode'].mode)
            pi = snap.get('pi_state')
            link = snap.get('link')
        else:      # simulator: 마지막 RX 한 상대 데이터
            pi, link = self._sim_last_rx, None
        if pi is not None:
            s.update(pi_throttle=pi.throttle_pwm, pi_steer=pi.steer_pwm,
                     pi_lane=pi.lane, pi_seq=pi.seq)
        if link is not None:
            s.update(link_age=link.age_rx, link_state=link.state.name)
        return s

    def _push_monitor(self, t, throttle, steer, behavior, snap=None):
        m = self._live_monitor
        if m is not None and m.winfo_exists():
            m.add_sample(t, self._build_monitor_sample(throttle, steer, behavior, snap))


if __name__ == '__main__':
    os.makedirs(LOG_ROOT, exist_ok=True)
    App().mainloop()
