"""시나리오 편집기 — 재생 중 특정 시각에 파라미터를 자동 변경하는 단계 목록을 편집/저장/불러온다."""
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

from scenario import ScenarioStep, PARAM_LABELS, PARAM_BOOL, PARAM_INT, \
    coerce_value, save_scenario, load_scenario


class ScenarioEditor(tk.Toplevel):
    """시나리오 단계 목록을 편집하는 팝업 창.

    steps: RoleTab 이 소유하는 list[ScenarioStep] — 이 창에서 직접 수정한다.
    """

    def __init__(self, parent, steps: list):
        super().__init__(parent)
        self.title('시나리오 편집 (Scenario Editor)')
        self.resizable(True, True)
        self._steps = steps
        self._build_ui()
        self._refresh_list()
        self.grab_set()   # 모달

    # ── UI ────────────────────────────────────────────────────────────
    def _build_ui(self):
        # 단계 목록 (고정 높이 — 확장 안 함)
        list_frm = ttk.LabelFrame(self, text='단계 목록 (Steps)')
        list_frm.pack(fill='x', padx=8, pady=4)

        self._lb = tk.Listbox(list_frm, height=7, width=60,
                              selectmode='single', font=('Consolas', 9))
        sb = ttk.Scrollbar(list_frm, orient='vertical', command=self._lb.yview)
        self._lb.configure(yscrollcommand=sb.set)
        sb.pack(side='right', fill='y', pady=2)
        self._lb.pack(side='left', fill='x', expand=True, padx=2, pady=2)

        # 컨텍스트 메뉴
        self._ctx = tk.Menu(self, tearoff=False)
        self._ctx.add_command(label='선택 삭제', command=self._remove_selected)
        self._lb.bind('<Button-3>', self._show_ctx)

        # 추가 폼
        add_frm = ttk.LabelFrame(self, text='단계 추가')
        add_frm.pack(fill='x', padx=8, pady=4)

        row1 = ttk.Frame(add_frm)
        row1.pack(fill='x', pady=2)
        ttk.Label(row1, text='시각(ms)', width=10).pack(side='left')
        self._t_var = tk.StringVar(value='1000')
        ttk.Entry(row1, textvariable=self._t_var, width=10).pack(side='left', padx=4)
        ttk.Label(row1, text='→  재생 시작 후 경과 ms (50ms 단위 권장)').pack(side='left')

        row2 = ttk.Frame(add_frm)
        row2.pack(fill='x', pady=2)
        ttk.Label(row2, text='파라미터', width=10).pack(side='left')
        self._param_var = tk.StringVar()
        param_keys = list(PARAM_LABELS.keys())
        param_cb = ttk.Combobox(row2, textvariable=self._param_var, width=28,
                                values=param_keys, state='readonly')
        param_cb.set(param_keys[0])
        param_cb.pack(side='left', padx=4)
        self._plabel = tk.StringVar()
        ttk.Label(row2, textvariable=self._plabel, width=28, foreground='gray').pack(side='left')
        param_cb.bind('<<ComboboxSelected>>', self._on_param_select)

        row3 = ttk.Frame(add_frm)
        row3.pack(fill='x', pady=2)
        ttk.Label(row3, text='값', width=10).pack(side='left')
        self._val_var = tk.StringVar(value='0.0')
        self._val_hint = tk.StringVar(value='(float)')
        ttk.Entry(row3, textvariable=self._val_var, width=14).pack(side='left', padx=4)
        ttk.Label(row3, textvariable=self._val_hint, foreground='gray').pack(side='left')

        self._on_param_select()   # val_var/val_hint 생성 후 초기화

        btn_frm = ttk.Frame(add_frm)
        btn_frm.pack(fill='x', pady=4)
        ttk.Button(btn_frm, text='+ 추가',    command=self._add_step).pack(side='left', padx=4)
        ttk.Button(btn_frm, text='시간순 정렬', command=self._sort_steps).pack(side='left', padx=4)
        ttk.Button(btn_frm, text='선택 삭제',  command=self._remove_selected).pack(side='left', padx=4)
        ttk.Button(btn_frm, text='전체 삭제',  command=self._clear_steps).pack(side='left', padx=4)

        # 저장/불러오기
        io_frm = ttk.Frame(self)
        io_frm.pack(fill='x', padx=8, pady=(0, 8))
        ttk.Button(io_frm, text='💾 JSON 저장',    command=self._save).pack(side='left', padx=4)
        ttk.Button(io_frm, text='📂 JSON 불러오기', command=self._load).pack(side='left', padx=4)
        self._status = tk.StringVar(value='')
        ttk.Label(io_frm, textvariable=self._status, foreground='#555').pack(side='left', padx=8)

    def _on_param_select(self, _=None):
        key = self._param_var.get()
        self._plabel.set(PARAM_LABELS.get(key, ''))
        if key in PARAM_BOOL:
            hint = '(True / False)'
            self._val_var.set('True')
        elif key in PARAM_INT:
            hint = '(정수: 0·1·2)'
            self._val_var.set('1')
        else:
            hint = '(소수)'
            self._val_var.set('0.0')
        self._val_hint.set(hint)

    def _add_step(self):
        try:
            t_ms  = int(self._t_var.get())
            param = self._param_var.get()
            value = coerce_value(param, self._val_var.get())
        except (ValueError, KeyError) as e:
            messagebox.showerror('입력 오류', str(e), parent=self)
            return
        self._steps.append(ScenarioStep(t_ms=t_ms, param=param, value=value))
        self._refresh_list()
        self._status.set(f'추가됨: {t_ms}ms/{param}={value}')
        # 다음 시각 자동 증가 (+ 500ms)
        self._t_var.set(str(t_ms + 500))

    def _remove_selected(self):
        sel = self._lb.curselection()
        if not sel:
            return
        idx = sel[0]
        del self._steps[idx]
        self._refresh_list()

    def _sort_steps(self):
        self._steps.sort(key=lambda s: s.t_ms)
        self._refresh_list()
        self._status.set('시간순 정렬 완료')

    def _clear_steps(self):
        if messagebox.askyesno('전체 삭제', '모든 단계를 삭제할까요?', parent=self):
            self._steps.clear()
            self._refresh_list()

    def _refresh_list(self):
        self._lb.delete(0, 'end')
        for i, s in enumerate(self._steps):
            self._lb.insert('end', f'  {i+1:3d}.  {s.label()}')

    def _show_ctx(self, event):
        try:
            self._lb.selection_clear(0, 'end')
            self._lb.selection_set(self._lb.nearest(event.y))
            self._ctx.tk_popup(event.x_root, event.y_root)
        finally:
            self._ctx.grab_release()

    def _save(self):
        if not self._steps:
            messagebox.showinfo('저장', '단계가 없습니다.', parent=self)
            return
        path = filedialog.asksaveasfilename(
            defaultextension='.json',
            filetypes=[('JSON', '*.json'), ('All', '*.*')],
            title='시나리오 저장', parent=self)
        if path:
            save_scenario(self._steps, path)
            self._status.set(f'저장됨: {path}')

    def _load(self):
        path = filedialog.askopenfilename(
            filetypes=[('JSON', '*.json'), ('All', '*.*')],
            title='시나리오 불러오기', parent=self)
        if path:
            try:
                loaded = load_scenario(path)
            except Exception as e:
                messagebox.showerror('오류', str(e), parent=self)
                return
            self._steps.clear()
            self._steps.extend(loaded)
            self._refresh_list()
            self._status.set(f'{len(loaded)}개 단계 로드됨')
