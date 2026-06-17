# VILS 시뮬레이터 구현 계획

> **에이전트 작업자용:** 필수 서브스킬: superpowers:subagent-driven-development(권장) 또는 superpowers:executing-plans 를 사용하여 태스크 단위로 구현. 단계는 체크박스(`- [ ]`) 형식으로 추적.

**목표:** `simulator/` 에 실제 Pi 차량의 V2V 파트너 역할을 하는 VILS 도구 구축 — Scene 주입 UI, 실시간 트랙 시각화, 바이너리 로깅, 재생 기능 포함.

**아키텍처:** `sim_perception.py`가 UI 제어 Scene을 버스에 전달; `vils_core.py`가 실제 `src/` decision+motion+v2v 모듈을 사용하여 50ms 스케줄러 루프 실행; `logger.py`가 바이너리 로깅을 위한 60B 패킷 인터셉트; `track_canvas.py`가 트랙 그리기 및 두 차량 애니메이션; `app.py`가 모든 컴포넌트를 탭 구성 tkinter 창에 연결.

**기술 스택:** Python 3 표준 라이브러리만 — tkinter, threading, socket, struct, hmac, csv, argparse

---

## 파일 구성

| 파일 | 역할 |
|------|------|
| `simulator/app.py` | 메인 진입점, 탭 구성 tkinter UI (Follower/Leader + Real-time/Playback) |
| `simulator/sim_perception.py` | SimPerception: UI 파라미터 dict → Scene 발행자 |
| `simulator/logger.py` | SessionRecorder: 폴더 명명, bin 파일 I/O, `RecordableV2VModule` |
| `simulator/vils_core.py` | VILSEngine: src 모듈을 감싸는 50ms 루프, 버스 스냅샷 |
| `simulator/track_canvas.py` | TrackCanvas: tkinter Canvas 서브클래스, 트랙 그리기, 차량 애니메이션, 움직임 모델 |
| `simulator/converter.py` | CLI: bin → CSV 후처리 |
| `simulator/tests/test_logger.py` | 단위 테스트: 인덱스 스캔, 폴더 명명, bin 왕복 |
| `simulator/tests/test_converter.py` | 단위 테스트: CSV 출력, 잘못된 패킷 건너뛰기 |
| `simulator/tests/test_track.py` | 단위 테스트: 월드→스크린 변환, 움직임 모델 |

---

## 태스크 1: 스캐폴드 및 src import 검증

**파일:**
- 생성: `simulator/tests/__init__.py`
- 생성: `simulator/_src_path.py`

- [ ] **단계 1: 디렉토리 구조 생성**

```
mkdir simulator/tests
touch simulator/tests/__init__.py
```

- [ ] **단계 2: `simulator/_src_path.py` 생성**

```python
import os, sys

def add():
    src = os.path.join(os.path.dirname(__file__), '..', 'src')
    src = os.path.normpath(src)
    if src not in sys.path:
        sys.path.insert(0, src)
```

모든 simulator 파일은 `import _src_path; _src_path.add()`로 시작.

- [ ] **단계 3: import 스모크 테스트 작성**

`simulator/tests/test_imports.py` 생성:

```python
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import _src_path; _src_path.add()

from core_module.bus import MessageBus, Topics
from core_module.v2v import packet_generator, packet_parser, PACKET_LEN, fmt_ms_of_day
from algorithm.decision import DecisionModule
from algorithm.motion_planning import MotionModule
from messages import EgoState, Scene, V2VState, Role, DriveBehavior

def test_imports_work():
    bus = MessageBus()
    assert bus.read(Topics.SCENE) is None

if __name__ == '__main__':
    test_imports_work()
    print('OK')
```

- [ ] **단계 4: 스모크 테스트 실행**

```
cd d:/Source/IVS_Final/simulator
python tests/test_imports.py
```

예상: `OK`

- [ ] **단계 5: 커밋**

```bash
git add simulator/
git commit -m "feat(simulator): scaffold + src import path helper"
```

---

## 태스크 2: sim_perception.py

**파일:**
- 생성: `simulator/sim_perception.py`
- 생성: `simulator/tests/test_sim_perception.py`

- [ ] **단계 1: 실패 테스트 작성**

`simulator/tests/test_sim_perception.py` 생성:

```python
import os, sys, time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import _src_path; _src_path.add()

from core_module.bus import MessageBus, Topics
from sim_perception import SimPerception

def test_step_publishes_scene():
    bus = MessageBus()
    sp = SimPerception()
    sp.params['lane_valid'] = True
    sp.params['current_lane'] = 2
    sp.params['lane_offset_m'] = 0.1
    sp.step(bus)
    scene = bus.read(Topics.SCENE)
    assert scene is not None
    assert scene.lane_valid is True
    assert scene.current_lane == 2
    assert abs(scene.lane_offset_m - 0.1) < 1e-9

def test_defaults_are_safe():
    bus = MessageBus()
    SimPerception().step(bus)
    scene = bus.read(Topics.SCENE)
    assert scene.lane_valid is False
    assert scene.front_clear is True
    assert scene.dist_front_m is None

if __name__ == '__main__':
    test_step_publishes_scene()
    test_defaults_are_safe()
    print('OK')
```

- [ ] **단계 2: 테스트 실행 — FAIL 확인**

```
cd d:/Source/IVS_Final/simulator
python tests/test_sim_perception.py
```

예상: `ModuleNotFoundError: No module named 'sim_perception'`

- [ ] **단계 3: `simulator/sim_perception.py` 구현**

```python
import time
import _src_path; _src_path.add()

from core_module.bus import Topics
from messages import Scene


class SimPerception:
    """UI 제어 가짜 인지 모듈. 50ms step() 호출마다 버스에 발행."""

    def __init__(self):
        self.params = {
            'lane_valid': False,
            'current_lane': 0,
            'lane_offset_m': 0.0,
            'lane_heading_rad': 0.0,
            'lane_curvature_1pm': 0.0,
            'front_clear': True,
            'dist_front_m': None,
            'stop_signal': False,
        }

    def step(self, bus):
        p = self.params
        scene = Scene(
            stamp=time.monotonic(),
            lane_valid=bool(p['lane_valid']),
            current_lane=int(p['current_lane']),
            lane_offset_m=float(p['lane_offset_m']),
            lane_heading_rad=float(p['lane_heading_rad']),
            lane_curvature_1pm=float(p['lane_curvature_1pm']),
            front_clear=bool(p['front_clear']),
            dist_front_m=p['dist_front_m'],
            stop_signal=bool(p['stop_signal']),
        )
        bus.publish(Topics.SCENE, scene)
```

- [ ] **단계 4: 테스트 실행 — PASS 확인**

```
cd d:/Source/IVS_Final/simulator
python tests/test_sim_perception.py
```

예상: `OK`

- [ ] **단계 5: 커밋**

```bash
git add simulator/sim_perception.py simulator/tests/test_sim_perception.py
git commit -m "feat(simulator): SimPerception — UI param dict to Scene publisher"
```

---

## 태스크 3: logger.py

**파일:**
- 생성: `simulator/logger.py`
- 생성: `simulator/tests/test_logger.py`

- [ ] **단계 1: 실패 테스트 작성**

`simulator/tests/test_logger.py` 생성:

```python
import os, sys, struct, tempfile, shutil
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import _src_path; _src_path.add()

from logger import SessionRecorder, next_index

LOG_ROOT = os.path.join(os.path.dirname(__file__), '..', '..', 'data', 'log')


def test_next_index_empty(tmp_path):
    assert next_index(str(tmp_path)) == 1


def test_next_index_with_existing(tmp_path):
    os.makedirs(os.path.join(tmp_path, '03_2026-06-16 100000'))
    os.makedirs(os.path.join(tmp_path, '07_2026-06-16 110000_2026-06-16 110500'))
    assert next_index(str(tmp_path)) == 8


def test_session_writes_bin_and_renames(tmp_path):
    rec = SessionRecorder(str(tmp_path))
    rec.start()

    dummy_60b = bytes(60)
    rec.on_packet(dummy_60b, '2026-06-16 140000')
    rec.on_packet(dummy_60b, '2026-06-16 140000')

    folder, path = rec.stop('2026-06-16 140100')
    assert os.path.exists(path)
    assert os.path.getsize(path) == 120  # 2 × 60B
    assert '01_2026-06-16 140000_2026-06-16 140100' in folder


if __name__ == '__main__':
    import tempfile
    with tempfile.TemporaryDirectory() as d:
        test_next_index_empty(d)
        test_next_index_with_existing(d)
        test_session_writes_bin_and_renames(d)
    print('OK')
```

- [ ] **단계 2: 테스트 실행 — FAIL 확인**

```
cd d:/Source/IVS_Final/simulator
python tests/test_logger.py
```

예상: `ModuleNotFoundError: No module named 'logger'`

- [ ] **단계 3: `simulator/logger.py` 구현**

```python
import os, re, threading, socket, datetime
import _src_path; _src_path.add()

from core_module.v2v import V2VModule, packet_parser, PACKET_LEN


def next_index(log_root):
    """log_root에서 XX_ 접두사 폴더를 스캔하여 max+1 반환 (최소 1)."""
    if not os.path.isdir(log_root):
        return 1
    max_idx = 0
    for name in os.listdir(log_root):
        m = re.match(r'^(\d+)_', name)
        if m:
            max_idx = max(max_idx, int(m.group(1)))
    return max_idx + 1


def _now_str():
    return datetime.datetime.now().strftime('%Y-%m-%d %H%M%S')


class SessionRecorder:
    """하나의 녹화 세션 관리: 폴더 생성, bin 파일, stop 시 이름 변경."""

    def __init__(self, log_root):
        self._root = log_root
        self._idx = None
        self._tmp_folder = None
        self._bin_path = None
        self._fh = None
        self._first_rx_time = None
        self._count = 0
        self._lock = threading.Lock()

    def start(self):
        os.makedirs(self._root, exist_ok=True)
        self._idx = next_index(self._root)
        now_str = _now_str()
        self._tmp_folder = os.path.join(self._root, f'{self._idx:02d}_{now_str}')
        os.makedirs(self._tmp_folder, exist_ok=True)
        self._bin_path = os.path.join(self._tmp_folder, 'session.bin')
        self._first_rx_time = None
        self._count = 0

    def on_packet(self, raw_bytes, first_rx_str=None):
        """검증된 60B 패킷마다 호출. first_rx_str은 테스트 주입용."""
        with self._lock:
            if self._first_rx_time is None:
                self._first_rx_time = first_rx_str or _now_str()
                self._fh = open(self._bin_path, 'wb')
            if self._fh:
                self._fh.write(raw_bytes)
                self._count += 1

    def stop(self, stop_str=None):
        """파일 닫기, 폴더 이름 변경. (new_folder_name, bin_path) 반환."""
        with self._lock:
            if self._fh:
                self._fh.close()
                self._fh = None
        stop_time = stop_str or _now_str()
        start_time = self._first_rx_time or _now_str()
        new_name = f'{self._idx:02d}_{start_time}_{stop_time}'
        new_folder = os.path.join(self._root, new_name)
        new_bin = os.path.join(new_folder, 'session.bin')
        if os.path.exists(self._tmp_folder):
            os.rename(self._tmp_folder, new_folder)
        return new_name, new_bin

    @property
    def packet_count(self):
        with self._lock:
            return self._count

    @property
    def is_recording(self):
        with self._lock:
            return self._fh is not None


class RecordableV2VModule(V2VModule):
    """검증된 수신 패킷마다 on_packet_cb(raw_bytes)를 호출하는 V2VModule 서브클래스."""

    def __init__(self, role, on_packet_cb=None):
        super().__init__(role)
        self._on_packet_cb = on_packet_cb

    def set_record_callback(self, cb):
        self._on_packet_cb = cb

    def _rx_loop(self):
        import time
        while not self._stop.is_set():
            try:
                data, _addr = self._rx.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                if self._stop.is_set():
                    break
                continue

            try:
                state = packet_parser(data, self._key)
            except ValueError:
                continue

            if self._on_packet_cb is not None:
                self._on_packet_cb(data)

            now = time.monotonic()
            with self._lock:
                last = self._rx_seq
                if last is not None:
                    delta = (state.seq - last) & 0xFFFF
                    if delta == 0 or delta >= 0x8000:
                        continue
                self._rx_seq = state.seq
                self._last_rx = now
            state.t_rx = now
            self._bus.publish(self._rx_topic, state)
```

- [ ] **단계 4: 테스트 실행 — PASS 확인**

```
cd d:/Source/IVS_Final/simulator
python tests/test_logger.py
```

예상: `OK`

- [ ] **단계 5: 커밋**

```bash
git add simulator/logger.py simulator/tests/test_logger.py
git commit -m "feat(simulator): SessionRecorder + RecordableV2VModule"
```

---

## 태스크 4: converter.py

**파일:**
- 생성: `simulator/converter.py`
- 생성: `simulator/tests/test_converter.py`

- [ ] **단계 1: 실패 테스트 작성**

`simulator/tests/test_converter.py` 생성:

```python
import os, sys, csv, tempfile, struct
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import _src_path; _src_path.add()

from core_module.v2v import packet_generator, PACKET_LEN
from messages import EgoState, Role, DriveBehavior
from converter import convert_bin_to_csv

KEY = b'test-key-32-bytes-padded-with-xx'


def _make_bin(tmp_path, n=3):
    path = os.path.join(tmp_path, 'session.bin')
    with open(path, 'wb') as f:
        for i in range(n):
            ego = EgoState(stamp=float(i), throttle_pwm=0.1*i, steer_pwm=-0.05*i, behavior=DriveBehavior.FOLLOW)
            f.write(packet_generator(ego, lane=1, role=Role.LEADER, seq=i, key=KEY))
    return path


def test_convert_produces_csv(tmp_path):
    bin_path = _make_bin(tmp_path)
    csv_path = os.path.join(tmp_path, 'out.csv')
    count = convert_bin_to_csv(bin_path, csv_path, key=KEY)
    assert count == 3
    with open(csv_path) as f:
        rows = list(csv.DictReader(f))
    assert len(rows) == 3
    assert rows[0]['seq'] == '0'
    assert abs(float(rows[2]['throttle_pwm']) - 0.2) < 1e-5


def test_convert_skips_bad_packets(tmp_path):
    bin_path = os.path.join(tmp_path, 'bad.bin')
    with open(bin_path, 'wb') as f:
        ego = EgoState(stamp=1.0, throttle_pwm=0.5, behavior=DriveBehavior.FOLLOW)
        f.write(packet_generator(ego, 1, Role.LEADER, 1, KEY))
        f.write(bytes(60))  # 쓰레기 패킷
    count = convert_bin_to_csv(bin_path, os.path.join(tmp_path, 'out.csv'), key=KEY)
    assert count == 1


if __name__ == '__main__':
    import tempfile
    with tempfile.TemporaryDirectory() as d:
        test_convert_produces_csv(d)
        test_convert_skips_bad_packets(d)
    print('OK')
```

- [ ] **단계 2: 테스트 실행 — FAIL 확인**

```
cd d:/Source/IVS_Final/simulator
python tests/test_converter.py
```

예상: `ModuleNotFoundError: No module named 'converter'`

- [ ] **단계 3: `simulator/converter.py` 구현**

```python
"""bin → CSV 후처리기.

사용법:
    python converter.py session.bin [output.csv] [--no-verify]
"""
import os, sys, csv, argparse, struct
import _src_path; _src_path.add()

from core_module.v2v import packet_parser, PACKET_LEN, fmt_ms_of_day
from core_module import config
from messages import Role, DriveBehavior, V2VState

COLUMNS = ['seq', 'tx_abs', 'tx_time', 'role', 'lane', 'behavior', 'throttle_pwm', 'steer_pwm']
_BODY_FMT = '!BBBHdIBBffx'
_BODY_LEN = struct.calcsize(_BODY_FMT)


def convert_bin_to_csv(bin_path, csv_path, key=None, verify=True):
    """bin_path(60B 패킷)를 파싱하여 csv_path에 저장. 유효 패킷 수 반환."""
    if key is None:
        key = config.load_key()

    count = 0
    with open(bin_path, 'rb') as fin, open(csv_path, 'w', newline='') as fout:
        writer = csv.DictWriter(fout, fieldnames=COLUMNS)
        writer.writeheader()
        while True:
            raw = fin.read(PACKET_LEN)
            if len(raw) < PACKET_LEN:
                break
            try:
                if verify:
                    state = packet_parser(raw, key)
                else:
                    _, _, role, seq, t, tx_abs, lane, beh, thr, st = struct.unpack(_BODY_FMT, raw[:_BODY_LEN])
                    state = V2VState(t_tx=t, tx_abs=tx_abs, role=Role(role), seq=seq,
                                     lane=lane, throttle_pwm=thr, steer_pwm=st,
                                     behavior=DriveBehavior(beh))
            except Exception:
                continue
            writer.writerow({
                'seq': state.seq,
                'tx_abs': state.tx_abs,
                'tx_time': fmt_ms_of_day(state.tx_abs),
                'role': state.role.name,
                'lane': state.lane,
                'behavior': state.behavior.name,
                'throttle_pwm': f'{state.throttle_pwm:.6f}',
                'steer_pwm': f'{state.steer_pwm:.6f}',
            })
            count += 1
    return count


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='session.bin을 CSV로 변환')
    parser.add_argument('bin_file')
    parser.add_argument('csv_file', nargs='?')
    parser.add_argument('--no-verify', action='store_true')
    args = parser.parse_args()

    out = args.csv_file or args.bin_file.replace('.bin', '.csv')
    n = convert_bin_to_csv(args.bin_file, out, verify=not args.no_verify)
    print(f'{n}개 패킷 저장 → {out}')
```

- [ ] **단계 4: 테스트 실행 — PASS 확인**

```
cd d:/Source/IVS_Final/simulator
python tests/test_converter.py
```

예상: `OK`

- [ ] **단계 5: 커밋**

```bash
git add simulator/converter.py simulator/tests/test_converter.py
git commit -m "feat(simulator): converter — bin to CSV CLI"
```

---

## 태스크 5: track_canvas.py

**파일:**
- 생성: `simulator/track_canvas.py`
- 생성: `simulator/tests/test_track.py`

트랙 월드 좌표 (미터, 중앙=원점, x→오른쪽, y→위):
- 내곽 초록 타원: rx=1.05m, ry=0.825m
- 노란 타원 (차선 구분선): rx=1.25m, ry=1.025m
- 외곽 초록 타원: rx=1.45m, ry=1.225m

- [ ] **단계 1: 실패 테스트 작성**

`simulator/tests/test_track.py` 생성:

```python
import os, sys, math
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from track_canvas import world_to_screen, apply_motion_model


def test_world_to_screen_center():
    sx, sy = world_to_screen(0.0, 0.0, cx=450, cy=320, scale=220)
    assert sx == 450
    assert sy == 320


def test_world_to_screen_right():
    sx, sy = world_to_screen(1.0, 0.0, cx=450, cy=320, scale=220)
    assert abs(sx - 670) < 1   # 450 + 220
    assert sy == 320


def test_world_to_screen_up_inverted():
    # y=1.0 월드 → 스크린 y 감소 (스크린 y는 반전)
    sx, sy = world_to_screen(0.0, 1.0, cx=450, cy=320, scale=220)
    assert sx == 450
    assert abs(sy - 100) < 1   # 320 - 220


def test_motion_model_straight():
    x, y, h = apply_motion_model(0.0, 0.0, 0.0, throttle=1.0, steer=0.0, dt=1.0, k_v=1.0, k_w=1.0)
    assert abs(x - 1.0) < 1e-9
    assert abs(y) < 1e-9
    assert abs(h) < 1e-9


def test_motion_model_turn():
    x, y, h = apply_motion_model(0.0, 0.0, 0.0, throttle=0.0, steer=1.0, dt=1.0, k_v=1.0, k_w=math.pi/2)
    assert abs(h - math.pi/2) < 1e-9
    assert abs(x) < 1e-9   # 스로틀 없음 → 위치 변화 없음


if __name__ == '__main__':
    test_world_to_screen_center()
    test_world_to_screen_right()
    test_world_to_screen_up_inverted()
    test_motion_model_straight()
    test_motion_model_turn()
    print('OK')
```

- [ ] **단계 2: 테스트 실행 — FAIL 확인**

```
cd d:/Source/IVS_Final/simulator
python tests/test_track.py
```

예상: `ModuleNotFoundError: No module named 'track_canvas'`

- [ ] **단계 3: `simulator/track_canvas.py` 구현**

```python
"""TrackCanvas: 트랙 + 두 차량 애니메이션을 위한 tkinter Canvas 위젯."""
import math
try:
    import tkinter as tk
    _TK_AVAILABLE = True
except ImportError:
    _TK_AVAILABLE = False

# 트랙 형상 (월드 미터, 중앙=원점)
_TRACK_OVALS = [
    (1.45, 1.225, '#22aa22', 3),   # 외곽 초록
    (1.25, 1.025, '#ddcc00', 3),   # 노란 (차선 구분)
    (1.05, 0.825, '#22aa22', 3),   # 내곽 초록
]
_CANVAS_W, _CANVAS_H = 900, 580
_SCALE = 210   # px/m
_CX, _CY = _CANVAS_W // 2, _CANVAS_H // 2


def world_to_screen(wx, wy, cx=_CX, cy=_CY, scale=_SCALE):
    """월드 좌표 (m, y-위) → 스크린 좌표 (px, y-아래)."""
    return cx + wx * scale, cy - wy * scale


def apply_motion_model(x, y, heading, throttle, steer, dt, k_v, k_w):
    """단일 스텝 단순 비례 움직임 모델.
    월드 미터/라디안 단위 (new_x, new_y, new_heading) 반환."""
    heading = heading + steer * k_w * dt
    x = x + throttle * k_v * math.cos(heading) * dt
    y = y + throttle * k_v * math.sin(heading) * dt
    return x, y, heading


if _TK_AVAILABLE:
    def _triangle_points(cx, cy, heading, size=12):
        """탑뷰 차량 삼각형 폴리곤 (스크린 좌표)."""
        pts = []
        for angle, dist in [(0, size), (2.356, size * 0.6), (-2.356, size * 0.6)]:
            a = heading + angle
            pts += [cx + dist * math.cos(a), cy - dist * math.sin(a)]
        return pts

    class TrackCanvas(tk.Canvas):
        """타원 트랙을 그리고 Pi 차량 + Sim 차량을 애니메이션."""

        def __init__(self, parent, **kwargs):
            kwargs.setdefault('width', _CANVAS_W)
            kwargs.setdefault('height', _CANVAS_H)
            kwargs.setdefault('bg', '#f5f5f5')
            super().__init__(parent, **kwargs)

            self._pi_state = None
            self._sim_state = None

            self._pi_tag = 'pi_vehicle'
            self._sim_tag = 'sim_vehicle'
            self._trail_tag = 'trail'

            self._draw_track()
            self.bind('<Button-1>', self._on_click)

            self.k_v = 1.0
            self.k_w = 2.0

        def _draw_track(self):
            self.delete('track')
            for rx, ry, color, width in _TRACK_OVALS:
                sx1, sy1 = world_to_screen(-rx, ry)
                sx2, sy2 = world_to_screen(rx, -ry)
                self.create_oval(sx1, sy1, sx2, sy2, outline=color, width=width, tags='track')

        def _on_click(self, event):
            wx = (event.x - _CX) / _SCALE
            wy = (_CY - event.y) / _SCALE
            self._pi_state = [wx, wy, 0.0]
            self._sim_state = [wx, wy, 0.0]
            self._redraw_vehicles()

        def set_start_pos(self, wx, wy, heading=0.0):
            self._pi_state = [wx, wy, heading]
            self._sim_state = [wx, wy, heading]
            self._redraw_vehicles()

        def reset_trail(self):
            self.delete(self._trail_tag)

        def update_pi(self, throttle, steer, dt):
            if self._pi_state is None:
                return
            x, y, h = apply_motion_model(*self._pi_state, throttle, steer, dt, self.k_v, self.k_w)
            old_sx, old_sy = world_to_screen(*self._pi_state[:2])
            self._pi_state = [x, y, h]
            new_sx, new_sy = world_to_screen(x, y)
            self.create_line(old_sx, old_sy, new_sx, new_sy, fill='#4488ff', width=1, tags=self._trail_tag)
            self._redraw_vehicles()

        def update_sim(self, throttle, steer, dt):
            if self._sim_state is None:
                return
            x, y, h = apply_motion_model(*self._sim_state, throttle, steer, dt, self.k_v, self.k_w)
            old_sx, old_sy = world_to_screen(*self._sim_state[:2])
            self._sim_state = [x, y, h]
            new_sx, new_sy = world_to_screen(x, y)
            self.create_line(old_sx, old_sy, new_sx, new_sy, fill='#ff4444', width=1, tags=self._trail_tag)
            self._redraw_vehicles()

        def _redraw_vehicles(self):
            self.delete(self._pi_tag)
            self.delete(self._sim_tag)
            if self._pi_state:
                sx, sy = world_to_screen(*self._pi_state[:2])
                pts = _triangle_points(sx, sy, self._pi_state[2])
                self.create_polygon(pts, fill='#2255cc', outline='white', width=1, tags=self._pi_tag)
            if self._sim_state:
                sx, sy = world_to_screen(*self._sim_state[:2])
                pts = _triangle_points(sx, sy, self._sim_state[2])
                self.create_polygon(pts, fill='#cc2222', outline='white', width=1, tags=self._sim_tag)
```

- [ ] **단계 4: 테스트 실행 — PASS 확인**

```
cd d:/Source/IVS_Final/simulator
python tests/test_track.py
```

예상: `OK`

- [ ] **단계 5: 커밋**

```bash
git add simulator/track_canvas.py simulator/tests/test_track.py
git commit -m "feat(simulator): TrackCanvas widget + motion model"
```

---

## 태스크 6: vils_core.py

**파일:**
- 생성: `simulator/vils_core.py`

- [ ] **단계 1: `simulator/vils_core.py` 구현**

```python
"""VILSEngine: 실제 src 모듈을 사용하는 50ms 스케줄러 루프."""
import _src_path; _src_path.add()

from core_module.bus import MessageBus, Topics
from algorithm.decision import DecisionModule
from algorithm.motion_planning import MotionModule
from messages import Role
from logger import RecordableV2VModule


class VILSEngine:
    """src 모듈을 감싸고 tick()마다 한 스텝 실행.

    role: 'follower' 또는 'leader' (시뮬레이터 역할, Pi의 역할과 반대)
    on_packet_cb: 검증된 수신 패킷(60B)마다 raw bytes와 함께 호출
    """

    def __init__(self, role, on_packet_cb=None):
        src_role = Role.FOLLOWER if role == 'follower' else Role.LEADER
        self._bus = MessageBus()
        self._v2v = RecordableV2VModule(role, on_packet_cb)
        self._decision = DecisionModule(src_role)
        self._motion = MotionModule(src_role)
        self._started = False
        self._sim_perception = None

    def set_record_callback(self, cb):
        self._v2v.set_record_callback(cb)

    def start(self, sim_perception):
        """V2V RX 스레드 기동 및 인지 모듈 참조 저장."""
        self._sim_perception = sim_perception
        self._v2v.start(self._bus)
        self._started = True

    def tick(self):
        """UI 스레드에서 50ms마다 1회 호출."""
        if not self._started:
            return
        self._sim_perception.step(self._bus)
        self._decision.step(self._bus)
        self._motion.step(self._bus)
        self._v2v.step(self._bus)

    def stop(self):
        if self._started:
            self._v2v.stop()
            self._started = False

    def bus_snapshot(self):
        """UI 표시용 현재 버스 값 dict 반환."""
        def safe_read(topic):
            try:
                return self._bus.read(topic)
            except Exception:
                return None
        return {
            'command':  safe_read(Topics.COMMAND),
            'mode':     safe_read(Topics.MODE),
            'ego':      safe_read(Topics.EGO_STATE),
            'pi_state': safe_read(Topics.LEADER_STATE) or safe_read(Topics.FOLLOWER_STATE),
            'link':     safe_read(Topics.LINK_STATUS),
        }
```

- [ ] **단계 2: 스모크 테스트 (네트워크 불필요)**

```python
# 빠른 정상 확인 — simulator/ 디렉토리에서 실행
import sys; sys.path.insert(0, '.')
from vils_core import VILSEngine
from sim_perception import SimPerception

# 생성만 테스트 (start/stop 없으므로 소켓 미사용)
e = VILSEngine('follower')
print('VILSEngine constructed OK')
```

실행: `cd d:/Source/IVS_Final/simulator && python -c "...위 코드..."`
예상: `VILSEngine constructed OK`

- [ ] **단계 3: 커밋**

```bash
git add simulator/vils_core.py
git commit -m "feat(simulator): VILSEngine — 50ms scheduler loop with src modules"
```

---

## 태스크 7: app.py — 메인 UI

**파일:**
- 생성: `simulator/app.py`

- [ ] **단계 1: `simulator/app.py` 구현**

(전체 코드는 설계 문서 §5 참조 — 탭 구조, Scene 제어판, 버스 모니터, 실시간/재생 모드)

- [ ] **단계 2: 문법 검사 (디스플레이 불필요)**

```
cd d:/Source/IVS_Final/simulator
python -c "import ast; ast.parse(open('app.py').read()); print('syntax OK')"
```

예상: `syntax OK`

- [ ] **단계 3: 커밋**

```bash
git add simulator/app.py
git commit -m "feat(simulator): main VILS app — tabbed UI, real-time + playback"
```

- [ ] **단계 4: 종단간 스모크 테스트**

```
cd d:/Source/IVS_Final/simulator
python app.py
```

확인 사항:
1. 창이 열리고 Follower/Leader 탭 표시
2. 트랙(초록/노란 타원) 렌더링 확인
3. 트랙 위 마우스 클릭 → 파란/빨간 삼각형 생성
4. Playback 탭 선택 → "파일 열기" 버튼 표시
5. 창 닫기 시 에러 없음

---

## 셀프 리뷰 체크리스트

- [x] **§4 녹화 흐름**: Task 3 `SessionRecorder` 구현. `RecordableV2VModule`이 검증된 패킷만 기록.
- [x] **§5 Scene 제어판**: Task 7 `_build_scene_panel()` — 8개 필드 전부 위젯화.
- [x] **§5 버스 모니터**: Task 7 `_refresh_bus_monitor()` — 10개 레이블 50ms 갱신.
- [x] **§5 탭 구조**: `[Follower|Leader]` → `[Real-time|Playback]` 라디오 버튼.
- [x] **§6 트랙 치수**: Task 5 `_TRACK_OVALS` — inner(1.05/0.825), yellow(1.25/1.025), outer(1.45/1.225) m.
- [x] **§6 움직임 모델**: Task 5 `apply_motion_model()` — heading += steer×k_w×dt, pos += throttle×k_v×(cos/sin)×dt.
- [x] **§6 Real-time/Playback 모드**: Task 7 `_tick()` / `_pb_step()`.
- [x] **§7 converter**: Task 4 — `--no-verify` 플래그 포함.
- [x] **`tx_abs` 기반 dt**: `_tick()`과 `_pb_step()` 모두 `% 86_400_000` 자정 랩 처리.
- [x] **탭 전환 방지**: `_on_tab_change()` — 실행 중 탭 전환 차단.
