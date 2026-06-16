"""VILS Simulator — main tkinter application.

Run:
    cd d:/Source/IVS_Final/simulator
    python app.py
"""
import os, sys, time
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import _src_path; _src_path.add()

from core_module.v2v import packet_parser, PACKET_LEN, fmt_ms_of_day
from core_module import config
from sim_perception import SimPerception
from logger import SessionRecorder
from track_canvas import TrackCanvas
from vils_core import VILSEngine

LOG_ROOT = os.path.join(os.path.dirname(__file__), '..', 'data', 'log')
TICK_MS = 50


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title('VILS Simulator')
        self.resizable(False, False)

        nb = ttk.Notebook(self)
        nb.pack(fill='both', expand=True, padx=4, pady=4)

        self._tabs = {}
        for role in ('follower', 'leader'):
            tab = RoleTab(nb, role)
            nb.add(tab, text=role.capitalize())
            self._tabs[role] = tab

        nb.bind('<<NotebookTabChanged>>', self._on_tab_change)

    def _on_tab_change(self, event):
        nb = event.widget
        idx = nb.index(nb.select())
        role = list(self._tabs.keys())[idx]
        other = 'leader' if role == 'follower' else 'follower'
        other_tab = self._tabs[other]
        if other_tab.is_running:
            nb.select(list(self._tabs.keys()).index(other))
            messagebox.showwarning('VILS', f'{other.capitalize()} 탭이 실행 중입니다. 먼저 Stop 해주세요.')


class RoleTab(ttk.Frame):
    def __init__(self, parent, role):
        super().__init__(parent)
        self._role = role
        self._engine = None
        self._recorder = SessionRecorder(LOG_ROOT)
        self._perception = SimPerception()
        self._mode = tk.StringVar(value='realtime')
        self._playback_packets = []
        self._pb_idx = 0
        self._pb_speed = tk.DoubleVar(value=1.0)
        self._pb_running = False
        self._prev_pi_tx_abs = None
        self._start_time = None

        self._build_ui()

    def _build_ui(self):
        mode_bar = ttk.Frame(self)
        mode_bar.pack(fill='x', padx=6, pady=2)
        ttk.Radiobutton(mode_bar, text='Real-time', variable=self._mode, value='realtime',
                        command=self._on_mode_change).pack(side='left')
        ttk.Radiobutton(mode_bar, text='Playback', variable=self._mode, value='playback',
                        command=self._on_mode_change).pack(side='left', padx=10)

        main = ttk.Frame(self)
        main.pack(fill='both', expand=True)

        left = ttk.Frame(main, width=280)
        left.pack(side='left', fill='y', padx=4)
        left.pack_propagate(False)

        right = ttk.Frame(main)
        right.pack(side='left', fill='both', expand=True)

        self._track = TrackCanvas(right)
        self._track.pack()

        self._build_scene_panel(left)
        self._build_bus_monitor(left)
        self._build_control_bar(left)
        self._build_playback_bar(left)

        self._on_mode_change()

    def _build_scene_panel(self, parent):
        lf = ttk.LabelFrame(parent, text='Scene (SimPerception)')
        lf.pack(fill='x', pady=2)
        self._scene_vars = {}

        def add_check(key, label, default=False):
            v = tk.BooleanVar(value=default)
            ttk.Checkbutton(lf, text=label, variable=v,
                            command=lambda: self._perception.params.update({key: v.get()})).pack(anchor='w')
            self._scene_vars[key] = v

        def add_slider(key, label, from_, to, default=0.0):
            row = ttk.Frame(lf)
            row.pack(fill='x')
            ttk.Label(row, text=label, width=16, anchor='w').pack(side='left')
            v = tk.DoubleVar(value=default)
            s = ttk.Scale(row, variable=v, from_=from_, to=to, orient='horizontal',
                          command=lambda _: self._perception.params.update({key: round(v.get(), 4)}))
            s.pack(side='left', fill='x', expand=True)
            lbl = ttk.Label(row, textvariable=v, width=6)
            lbl.pack(side='left')
            self._scene_vars[key] = v

        def add_lane(key, label):
            row = ttk.Frame(lf)
            row.pack(fill='x')
            ttk.Label(row, text=label, width=16, anchor='w').pack(side='left')
            v = tk.IntVar(value=0)
            for val in [0, 1, 2]:
                ttk.Radiobutton(row, text=str(val), variable=v, value=val,
                                command=lambda: self._perception.params.update({key: v.get()})).pack(side='left')
            self._scene_vars[key] = v

        add_check('lane_valid', 'lane_valid', default=False)
        add_lane('current_lane', 'current_lane')
        add_slider('lane_offset_m', 'offset_m', -0.5, 0.5)
        add_slider('lane_heading_rad', 'heading_rad', -0.785, 0.785)
        add_slider('lane_curvature_1pm', 'curvature_1pm', -2.0, 2.0)
        add_check('front_clear', 'front_clear', default=True)
        add_slider('dist_front_m', 'dist_front_m', 0.0, 5.0, default=2.0)
        add_check('stop_signal', 'stop_signal', default=False)

        ttk.Separator(lf).pack(fill='x', pady=2)
        add_slider('__kv', 'k_v', 0.1, 5.0, default=1.0)
        add_slider('__kw', 'k_w', 0.1, 10.0, default=2.0)

    def _build_bus_monitor(self, parent):
        lf = ttk.LabelFrame(parent, text='Bus Monitor')
        lf.pack(fill='x', pady=2)
        self._bus_labels = {}
        for key in ['cmd_behavior', 'mode', 'ego_throttle', 'ego_steer',
                    'pi_seq', 'pi_lane', 'pi_throttle', 'pi_steer', 'link_state', 'link_age']:
            row = ttk.Frame(lf)
            row.pack(fill='x')
            ttk.Label(row, text=key, width=14, anchor='w').pack(side='left')
            v = tk.StringVar(value='—')
            ttk.Label(row, textvariable=v, width=12, anchor='e').pack(side='right')
            self._bus_labels[key] = v

    def _build_control_bar(self, parent):
        self._ctrl_frame = ttk.LabelFrame(parent, text='Real-time')
        self._ctrl_frame.pack(fill='x', pady=2)
        row = ttk.Frame(self._ctrl_frame)
        row.pack(fill='x')
        self._start_btn = ttk.Button(row, text='Start', command=self._start_realtime)
        self._start_btn.pack(side='left', padx=4)
        self._stop_btn = ttk.Button(row, text='Stop', command=self._stop_realtime, state='disabled')
        self._stop_btn.pack(side='left', padx=4)
        self._status_var = tk.StringVar(value='대기 중')
        ttk.Label(self._ctrl_frame, textvariable=self._status_var).pack(anchor='w', padx=4)
        self._pkt_var = tk.StringVar(value='패킷: 0')
        ttk.Label(self._ctrl_frame, textvariable=self._pkt_var).pack(anchor='w', padx=4)

    def _build_playback_bar(self, parent):
        self._pb_frame = ttk.LabelFrame(parent, text='Playback')
        self._pb_frame.pack(fill='x', pady=2)
        ttk.Button(self._pb_frame, text='파일 열기', command=self._open_bin).pack(fill='x', padx=4, pady=2)
        row = ttk.Frame(self._pb_frame)
        row.pack(fill='x')
        self._play_btn = ttk.Button(row, text='Play', command=self._play, state='disabled')
        self._play_btn.pack(side='left', padx=4)
        self._pause_btn = ttk.Button(row, text='Pause', command=self._pause, state='disabled')
        self._pause_btn.pack(side='left')
        speed_row = ttk.Frame(self._pb_frame)
        speed_row.pack(fill='x')
        ttk.Label(speed_row, text='속도').pack(side='left')
        for spd in [1.0, 2.0, 4.0]:
            ttk.Radiobutton(speed_row, text=f'{int(spd)}×', variable=self._pb_speed,
                            value=spd).pack(side='left')
        self._pb_status = tk.StringVar(value='파일 없음')
        ttk.Label(self._pb_frame, textvariable=self._pb_status).pack(anchor='w', padx=4)

    def _on_mode_change(self):
        if self._mode.get() == 'realtime':
            self._ctrl_frame.pack(fill='x', pady=2)
            self._pb_frame.pack_forget()
        else:
            self._pb_frame.pack(fill='x', pady=2)
            self._ctrl_frame.pack_forget()

    @property
    def is_running(self):
        return self._engine is not None and self._engine._started

    def _start_realtime(self):
        os.makedirs(LOG_ROOT, exist_ok=True)
        self._recorder.start()
        self._engine = VILSEngine(self._role, on_packet_cb=self._on_rx_packet)
        self._engine.start(self._perception)
        self._prev_pi_tx_abs = None
        self._start_time = time.monotonic()
        self._track.reset_trail()
        self._start_btn.config(state='disabled')
        self._stop_btn.config(state='normal')
        self._status_var.set('실행 중...')
        self._tick()

    def _stop_realtime(self):
        if self._engine:
            self._engine.stop()
            self._engine = None
        name, path = self._recorder.stop()
        self._start_btn.config(state='normal')
        self._stop_btn.config(state='disabled')
        self._status_var.set(f'저장: {name}')

    def _on_rx_packet(self, raw_bytes):
        self._recorder.on_packet(raw_bytes)

    def _tick(self):
        if not self.is_running:
            return
        self._engine.tick()
        self._update_kv_kw()
        snap = self._engine.bus_snapshot()
        self._refresh_bus_monitor(snap)
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
        elapsed = int(time.monotonic() - self._start_time)
        self._pkt_var.set(f'패킷: {self._recorder.packet_count}  경과: {elapsed}s')
        self.after(TICK_MS, self._tick)

    def _open_bin(self):
        path = filedialog.askopenfilename(
            initialdir=LOG_ROOT,
            filetypes=[('Binary', '*.bin'), ('All', '*.*')],
            title='session.bin 선택')
        if not path:
            return
        key = config.load_key()
        self._playback_packets = []
        with open(path, 'rb') as f:
            while True:
                raw = f.read(PACKET_LEN)
                if len(raw) < PACKET_LEN:
                    break
                try:
                    self._playback_packets.append(packet_parser(raw, key))
                except ValueError:
                    continue
        self._pb_idx = 0
        self._pb_running = False
        self._track.reset_trail()
        n = len(self._playback_packets)
        self._pb_status.set(f'{n}개 패킷 로드')
        if n > 0:
            self._play_btn.config(state='normal')

    def _play(self):
        if not self._playback_packets:
            return
        self._pb_running = True
        self._play_btn.config(state='disabled')
        self._pause_btn.config(state='normal')
        self._pb_step()

    def _pause(self):
        self._pb_running = False
        self._play_btn.config(state='normal')
        self._pause_btn.config(state='disabled')

    def _pb_step(self):
        if not self._pb_running:
            return
        if self._pb_idx >= len(self._playback_packets):
            self._pause()
            self._pb_status.set('재생 완료')
            return
        pkt = self._playback_packets[self._pb_idx]
        if self._pb_idx > 0:
            prev = self._playback_packets[self._pb_idx - 1]
            dt_ms = (pkt.tx_abs - prev.tx_abs) % 86_400_000
            dt = min(dt_ms / 1000.0, 0.5)
        else:
            dt = TICK_MS / 1000.0
        self._track.update_pi(pkt.throttle_pwm, pkt.steer_pwm, dt)
        self._pb_idx += 1
        self._pb_status.set(f'{self._pb_idx}/{len(self._playback_packets)}')
        speed = self._pb_speed.get()
        delay = max(1, int(TICK_MS / speed))
        self.after(delay, self._pb_step)

    def _update_kv_kw(self):
        kv = self._scene_vars.get('__kv')
        kw = self._scene_vars.get('__kw')
        if kv:
            self._track.k_v = kv.get()
        if kw:
            self._track.k_w = kw.get()

    def _refresh_bus_monitor(self, snap):
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


if __name__ == '__main__':
    os.makedirs(LOG_ROOT, exist_ok=True)
    App().mainloop()
