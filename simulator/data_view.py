"""View 메뉴용 데이터 뷰 — 서브시스템별 plot + 데이터 테이블.

  LiveMonitorWindow : 주행 중(Realtime/Simulator) 버스 데이터를 실시간 plot + 현재값 테이블.
                      플롯은 패널 단위(인지·판단·모션·통신) — 개별 추가/제거, 2열 리사이즈 레이아웃,
                      별도 창 분리(dock), 패널마다 줌/팬/이미지저장 툴바.
                      메모리: 지속 Line2D + set_data (clear+replot 안 함).
  BinAnalysisWindow : 저장된 60B .bin 을 열어 세션 전체를 시계열 plot (줌/저장 툴바).
"""
import os
import struct
from collections import deque
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

import _src_path; _src_path.add()
from messages import DriveBehavior, Mode, LinkState   # noqa: F401 (테이블 의미 참조용)
from core_module import config   # noqa: F401

import matplotlib
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk

# plot 한글 렌더 (Windows 기본 한글 폰트). 없으면 DejaVu 로 폴백.
matplotlib.rcParams['font.sans-serif'] = ['Malgun Gothic', 'DejaVu Sans']
matplotlib.rcParams['font.family'] = 'sans-serif'
matplotlib.rcParams['axes.unicode_minus'] = False


# 서브시스템(그룹) → 시계열로 그릴 수치 신호 [(key, 표시명)]
_GROUPS = {
    '인지 (Perception)': [('offset_m', 'offset(m)'),
                          ('heading_rad', 'heading(rad)'),
                          ('curvature', 'curv(1/m)')],
    '판단 (Decision)':   [('behavior', 'behavior'), ('mode', 'mode')],
    '모션 (Motion)':     [('throttle', 'throttle'), ('steer', 'steer')],
    '통신 (V2V)':        [('pi_throttle', 'leader thr'),
                          ('pi_steer', 'leader str'),
                          ('link_age', 'link age(ms)')],
}
# 현재값 테이블 [(key, 표시명)] — 상태값(문자열) 포함
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
_MAXLEN = 200          # 롤링 윈도우 길이 (~10s @ 50ms) — 메모리 상한
_REDRAW_EVERY = 5      # 몇 샘플마다 다시 그릴지 (250ms)


# ── 플롯 패널 ───────────────────────────────────────────────────────────
class _PlotPanel(ttk.Frame):
    """한 그룹의 실시간 plot 패널. 모니터의 공유 버퍼를 set_data 로 갱신(메모리 효율).
    줌/팬/이미지저장 툴바 포함. 메인 그리드 ↔ 별도 창 어디서든 동일하게 동작."""

    def __init__(self, parent, monitor, group, on_detach=None, on_close=None):
        super().__init__(parent, relief='groove', borderwidth=1)
        self._monitor = monitor
        self._group = group
        signals = _GROUPS[group]

        bar = ttk.Frame(self); bar.pack(fill='x')
        ttk.Label(bar, text=group, font=('TkDefaultFont', 9, 'bold')).pack(side='left', padx=4)
        if on_close is not None:
            ttk.Button(bar, text='✕', width=3,
                       command=lambda: on_close(self)).pack(side='right', padx=1)
        if on_detach is not None:
            ttk.Button(bar, text='⧉ 별도창',
                       command=lambda: on_detach(self)).pack(side='right', padx=1)
        self._autofit = tk.BooleanVar(value=True)
        ttk.Checkbutton(bar, text='자동맞춤', variable=self._autofit).pack(side='right', padx=4)

        self._fig = Figure(figsize=(4.2, 2.3), dpi=80)
        self._ax = self._fig.add_subplot(111)
        self._ax.set_title(group, fontsize=8)
        self._ax.grid(True, alpha=0.3)
        self._ax.tick_params(labelsize=7)
        self._lines = {}
        for key, label in signals:                  # 지속 Line2D 1회 생성 → 이후 set_data
            (ln,) = self._ax.plot([], [], label=label, linewidth=1)
            self._lines[key] = ln
        self._ax.legend(fontsize=7, loc='upper left')
        self._fig.tight_layout()                     # 1회만 (매 프레임 X)

        self._canvas = FigureCanvasTkAgg(self._fig, master=self)
        self._canvas.get_tk_widget().pack(fill='both', expand=True)
        tb = ttk.Frame(self); tb.pack(fill='x')
        self._toolbar = NavigationToolbar2Tk(self._canvas, tb, pack_toolbar=False)
        self._toolbar.update()
        self._toolbar.pack(fill='x')

    def refresh(self):
        """공유 버퍼에서 데이터를 읽어 라인 갱신 (객체 재생성 없음)."""
        if not self.winfo_exists():
            return
        ts = list(self._monitor._t)
        for key, ln in self._lines.items():
            ys = list(self._monitor._buf.get(key, ()))
            pts = [(x, y) for x, y in zip(ts, ys) if y is not None]
            if pts:
                xs, yv = zip(*pts)
                ln.set_data(xs, yv)
            else:
                ln.set_data([], [])
        # 자동맞춤 ON 이고 사용자가 줌/팬 도구를 안 쓰는 동안만 자동 스케일
        if self._autofit.get() and not self._toolbar.mode:
            self._ax.relim()
            self._ax.autoscale_view()
        self._canvas.draw_idle()


# ── 실시간 모니터 ───────────────────────────────────────────────────────
class LiveMonitorWindow(tk.Toplevel):
    """주행 중 버스 데이터를 패널형 실시간 plot + 현재값 테이블로 표시."""

    def __init__(self, parent, title='실시간 모니터', on_close=None):
        super().__init__(parent)
        self.title(f'{title} — Live Monitor')
        self.geometry('1040x720')
        self._on_close = on_close
        self._t = deque(maxlen=_MAXLEN)
        self._buf = {k: deque(maxlen=_MAXLEN)
                     for sigs in _GROUPS.values() for k, _ in sigs}
        self._n = 0
        self._panels = []     # [(panel, column_pane)]
        self._detached = []   # [(Toplevel, panel)]

        # 상단 툴바
        bar = ttk.Frame(self); bar.pack(fill='x', padx=4, pady=3)
        ttk.Label(bar, text='＋ 플롯 추가:').pack(side='left')
        self._add_var = tk.StringVar(value=list(_GROUPS)[0])
        ttk.Combobox(bar, textvariable=self._add_var, values=list(_GROUPS),
                     state='readonly', width=18).pack(side='left', padx=4)
        ttk.Button(bar, text='추가', command=self._add_selected).pack(side='left')
        self._show_table = tk.BooleanVar(value=True)
        ttk.Checkbutton(bar, text='값 테이블', variable=self._show_table,
                        command=self._toggle_table).pack(side='right')

        body = ttk.Frame(self); body.pack(fill='both', expand=True)
        # 플롯 영역: 2열 리사이즈 그리드 (가로 PanedWindow 안에 세로 PanedWindow 2개)
        self._pw = ttk.PanedWindow(body, orient='horizontal')
        self._pw.pack(side='left', fill='both', expand=True)
        self._columns = [ttk.PanedWindow(self._pw, orient='vertical'),
                         ttk.PanedWindow(self._pw, orient='vertical')]
        for col in self._columns:
            self._pw.add(col, weight=1)

        # 현재값 테이블 (우측)
        self._tbl = ttk.LabelFrame(body, text='현재값 (Bus)')
        self._tbl.pack(side='left', fill='y', padx=4, pady=4)
        self._tvars = {}
        for key, label in _TABLE_FIELDS:
            row = ttk.Frame(self._tbl); row.pack(fill='x', padx=2)
            ttk.Label(row, text=label, width=12, anchor='w').pack(side='left')
            v = tk.StringVar(value='—')
            ttk.Label(row, textvariable=v, width=10, anchor='e',
                      foreground='#1155cc').pack(side='right')
            self._tvars[key] = v

        for group in _GROUPS:   # 기본 4개 패널 (2열로 분배)
            self._add_panel(group)

        self.protocol('WM_DELETE_WINDOW', self._close)

    # ── 패널 추가/제거/분리 ──────────────────────────────────────────
    def _add_panel(self, group):
        col = min(self._columns, key=lambda c: len(c.panes()))   # 적은 쪽 열에
        panel = _PlotPanel(col, self, group,
                           on_detach=self._detach_panel, on_close=self._remove_panel)
        col.add(panel, weight=1)
        self._panels.append((panel, col))
        return panel

    def _add_selected(self):
        self._add_panel(self._add_var.get())

    def _remove_panel(self, panel):
        for i, (p, col) in enumerate(self._panels):
            if p is panel:
                try:
                    col.forget(panel)
                except tk.TclError:
                    pass
                panel.destroy()           # Figure/canvas GC → 메모리 해제
                self._panels.pop(i)
                return

    def _detach_panel(self, panel):
        group = panel._group
        self._remove_panel(panel)         # 메인에서 제거
        win = tk.Toplevel(self)
        win.title(f'{group} — 별도 창')
        win.geometry('560x440')
        dp = _PlotPanel(win, self, group, on_detach=None, on_close=None)
        dp.pack(fill='both', expand=True)

        def _redock():
            self._detached[:] = [(w, d) for (w, d) in self._detached if w is not win]
            try:
                win.destroy()
            except tk.TclError:
                pass
            self._add_panel(group)        # 메인 그리드로 복귀

        ttk.Button(win, text='⊟ 메인으로 도킹', command=_redock).pack(side='bottom', fill='x')
        win.protocol('WM_DELETE_WINDOW', _redock)
        self._detached.append((win, dp))

    def _toggle_table(self):
        if self._show_table.get():
            self._tbl.pack(side='left', fill='y', padx=4, pady=4)
        else:
            self._tbl.pack_forget()

    # ── 데이터 입력/갱신 ─────────────────────────────────────────────
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
            for p, _ in self._panels:
                p.refresh()
            for _, dp in self._detached:
                dp.refresh()

    def _close(self):
        for win, _ in list(self._detached):
            try:
                win.destroy()
            except tk.TclError:
                pass
        self._detached.clear()
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
    """저장된 .bin 을 열어 세션 전체를 시계열 plot (줌/팬/이미지저장 툴바)."""

    def __init__(self, parent, initial_dir):
        super().__init__(parent)
        self.title('bin 분석 — Bin Analysis')
        self.geometry('900x660')
        self._initial_dir = initial_dir

        bar = ttk.Frame(self); bar.pack(fill='x', padx=4, pady=4)
        ttk.Button(bar, text='📂 .bin 열기', command=self._open).pack(side='left')
        self._info = tk.StringVar(value='파일을 여세요.')
        ttk.Label(bar, textvariable=self._info, foreground='#1155cc').pack(side='left', padx=8)

        self._fig = Figure(figsize=(8.5, 5.6), dpi=80)
        self._axes = [self._fig.add_subplot(2, 2, i + 1) for i in range(4)]
        self._canvas = FigureCanvasTkAgg(self._fig, master=self)
        self._canvas.get_tk_widget().pack(fill='both', expand=True)
        tb = ttk.Frame(self); tb.pack(fill='x')
        self._toolbar = NavigationToolbar2Tk(self._canvas, tb, pack_toolbar=False)
        self._toolbar.update()
        self._toolbar.pack(fill='x')

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
