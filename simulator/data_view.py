"""View 메뉴용 데이터 뷰 — 서브시스템별 plot + 데이터 테이블.

  LiveMonitorWindow : 주행 중(Realtime/Simulator) 버스 데이터를 실시간 롤링 plot + 현재값 테이블.
                      인지·판단·모션·통신 4개 서브시스템.
  BinAnalysisWindow : 저장된 60B .bin 을 열어 세션 전체를 시계열 plot + 요약 테이블.
                      (bin 은 전송 패킷만 담으므로 모션 throttle/steer/behavior + 통신 lane/seq/role/time)
"""
import os
import struct
from collections import deque
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

import _src_path; _src_path.add()
from messages import DriveBehavior, Mode, LinkState
from core_module import config

import matplotlib
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# plot 제목·범례의 한글 렌더 (Windows 기본 한글 폰트). 없으면 DejaVu 로 폴백.
matplotlib.rcParams['font.sans-serif'] = ['Malgun Gothic', 'DejaVu Sans']
matplotlib.rcParams['font.family'] = 'sans-serif'
matplotlib.rcParams['axes.unicode_minus'] = False


# ── 실시간 모니터 ─────────────────────────────────────────────────────
# 서브시스템 → 시계열로 그릴 수치 신호 [(key, 표시명)]
_PLOT_SIGNALS = {
    '인지 (Perception)': [('offset_m', 'offset(m)'),
                          ('heading_rad', 'heading(rad)'),
                          ('curvature', 'curv(1/m)')],
    '판단 (Decision)':   [('behavior', 'behavior'), ('mode', 'mode')],
    '모션 (Motion)':     [('throttle', 'throttle'), ('steer', 'steer')],
    '통신 (V2V)':        [('pi_throttle', 'leader thr'),
                          ('pi_steer', 'leader str'),
                          ('link_age', 'link age(ms)')],
}
# 테이블에 표시할 모든 값 [(key, 표시명)] — 상태값(문자열) 포함
_TABLE_FIELDS = [
    ('current_lane', '현재차선'), ('offset_m', 'offset(m)'),
    ('heading_rad', 'heading'), ('curvature', 'curvature'),
    ('front_clear', 'front_clear'), ('stop_signal', 'stop_signal'),
    ('behavior', 'behavior'), ('mode', 'mode'),
    ('throttle', 'throttle'), ('steer', 'steer'),
    ('pi_throttle', 'leader thr'), ('pi_steer', 'leader str'),
    ('pi_lane', 'leader lane'), ('pi_seq', 'leader seq'),
    ('link_state', 'link_state'), ('link_age', 'link_age(ms)'),
]
_MAXLEN = 200          # 롤링 윈도우 길이 (~10s @ 50ms)
_REDRAW_EVERY = 5      # 몇 샘플마다 다시 그릴지 (250ms)


class LiveMonitorWindow(tk.Toplevel):
    """주행 중 버스 데이터를 실시간 plot + 테이블로 보여주는 창."""

    def __init__(self, parent, title='실시간 모니터', on_close=None):
        super().__init__(parent)
        self.title(f'{title} — Live Monitor')
        self.geometry('900x640')
        self._on_close = on_close
        self._t = deque(maxlen=_MAXLEN)
        self._buf = {k: deque(maxlen=_MAXLEN)
                     for sigs in _PLOT_SIGNALS.values() for k, _ in sigs}
        self._n = 0

        # 좌: 2x2 plot / 우: 현재값 테이블
        body = ttk.Frame(self); body.pack(fill='both', expand=True)
        self._fig = Figure(figsize=(6.4, 6.0), dpi=80)
        self._axes = {}
        for i, group in enumerate(_PLOT_SIGNALS):
            self._axes[group] = self._fig.add_subplot(2, 2, i + 1)
        self._canvas = FigureCanvasTkAgg(self._fig, master=body)
        self._canvas.get_tk_widget().pack(side='left', fill='both', expand=True)

        tbl = ttk.LabelFrame(body, text='현재값 (Bus)')
        tbl.pack(side='left', fill='y', padx=4, pady=4)
        self._tvars = {}
        for key, label in _TABLE_FIELDS:
            row = tk.Frame(tbl); row.pack(fill='x', padx=2)
            tk.Label(row, text=label, width=12, anchor='w').pack(side='left')
            v = tk.StringVar(value='—')
            tk.Label(row, textvariable=v, width=10, anchor='e',
                     fg='#1155cc').pack(side='right')
            self._tvars[key] = v

        self.protocol('WM_DELETE_WINDOW', self._close)

    @staticmethod
    def _fmt(v):
        if v is None:
            return '—'
        if isinstance(v, float):
            return f'{v:.3f}'
        return str(v)

    def add_sample(self, t, sample):
        """주행 틱마다 호출 — sample: {신호키: 값} dict."""
        if not self.winfo_exists():
            return
        self._t.append(t)
        for key in self._buf:
            v = sample.get(key)
            self._buf[key].append(v if isinstance(v, (int, float)) else None)
        for key, var in self._tvars.items():
            var.set(self._fmt(sample.get(key)))
        self._n += 1
        if self._n % _REDRAW_EVERY == 0:
            self._redraw()

    def _redraw(self):
        ts = list(self._t)
        for group, ax in self._axes.items():
            ax.clear()
            ax.set_title(group, fontsize=9)
            ax.grid(True, alpha=0.3)
            for key, label in _PLOT_SIGNALS[group]:
                ys = list(self._buf[key])
                xs = [x for x, y in zip(ts, ys) if y is not None]
                yv = [y for y in ys if y is not None]
                if yv:
                    ax.plot(xs, yv, label=label, linewidth=1)
            ax.legend(fontsize=7, loc='upper left')
            ax.tick_params(labelsize=7)
        self._fig.tight_layout()
        self._canvas.draw_idle()

    def _close(self):
        if self._on_close:
            self._on_close()
        self.destroy()


# ── bin 분석 ──────────────────────────────────────────────────────────
_FMT = '!BBBHdIBBffx'          # v2v 본문 포맷 (28B); 뒤 32B 는 HMAC
_HDR = struct.calcsize(_FMT)   # 28
_PKT = _HDR + 32               # 60


def _parse_bin(path):
    """60B 패킷 누적 .bin → dict of 시계열 (HMAC 무시, 본문만 디코드)."""
    data = open(path, 'rb').read()
    out = {'seq': [], 'tx_abs': [], 'role': [], 'lane': [],
           'behavior': [], 'throttle': [], 'steer': []}
    n = len(data) // _PKT
    for i in range(n):
        body = data[i * _PKT: i * _PKT + _HDR]
        ver, typ, role, seq, t_tx, tx_abs, lane, beh, thr, st = struct.unpack(_FMT, body)
        out['seq'].append(seq); out['tx_abs'].append(tx_abs)
        out['role'].append(role); out['lane'].append(lane)
        out['behavior'].append(beh); out['throttle'].append(thr); out['steer'].append(st)
    return out


class BinAnalysisWindow(tk.Toplevel):
    """저장된 .bin 을 열어 세션 전체를 시계열 plot + 요약."""

    def __init__(self, parent, initial_dir):
        super().__init__(parent)
        self.title('bin 분석 — Bin Analysis')
        self.geometry('900x640')
        self._initial_dir = initial_dir

        bar = ttk.Frame(self); bar.pack(fill='x', padx=4, pady=4)
        ttk.Button(bar, text='📂 .bin 열기', command=self._open).pack(side='left')
        self._info = tk.StringVar(value='파일을 여세요.')
        ttk.Label(bar, textvariable=self._info, foreground='#1155cc').pack(side='left', padx=8)

        self._fig = Figure(figsize=(8.5, 5.6), dpi=80)
        self._axes = [self._fig.add_subplot(2, 2, i + 1) for i in range(4)]
        self._canvas = FigureCanvasTkAgg(self._fig, master=self)
        self._canvas.get_tk_widget().pack(fill='both', expand=True)

    def _open(self):
        path = filedialog.askopenfilename(
            initialdir=self._initial_dir,
            filetypes=[('Binary', '*.bin'), ('All', '*.*')],
            title='세션 .bin 열기', parent=self)
        if not path:
            return
        try:
            s = _parse_bin(path)
        except Exception as e:
            messagebox.showerror('bin 분석', f'파싱 실패: {e}', parent=self)
            return
        n = len(s['seq'])
        if n == 0:
            self._info.set('패킷 없음 (0B)')
            return
        role = {1: 'leader', 2: 'follower'}.get(s['role'][0], '?')
        self._info.set(f'{os.path.basename(path)}  |  {n}패킷  role={role}  '
                       f'seq {s["seq"][0]}~{s["seq"][-1]}')
        x = list(range(n))
        specs = [('throttle (모션)', s['throttle']),
                 ('steer (모션)', s['steer']),
                 ('behavior (판단/모션)', s['behavior']),
                 ('lane (통신)', s['lane'])]
        for ax, (title, ys) in zip(self._axes, specs):
            ax.clear()
            ax.plot(x, ys, linewidth=1, color='#1155cc')
            ax.set_title(title, fontsize=9)
            ax.set_xlabel('packet #', fontsize=8)
            ax.grid(True, alpha=0.3)
            ax.tick_params(labelsize=7)
        self._fig.tight_layout()
        self._canvas.draw_idle()
