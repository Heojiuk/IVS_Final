import os, re, threading, socket, datetime
import _src_path; _src_path.add()

from core_module.v2v import V2VModule, packet_parser, PACKET_LEN


def next_index(log_root):
    """Scan log_root for XX_ prefixed folders and return max+1 (min 1)."""
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


def _role_of(raw_bytes):
    """60B 패킷의 role 바이트(offset 2: ver·type·role…) → 'leader'/'follower'/'unknown'."""
    if len(raw_bytes) < 3:
        return 'unknown'
    return {1: 'leader', 2: 'follower'}.get(raw_bytes[2], 'unknown')


class SessionRecorder:
    """한 녹화 세션을 관리 — 모드별 상위 폴더(Realtime/Simulator) 아래,
    패킷 role + PC 여부로 역할 스트림마다 하위 폴더 + 동일명 .bin 을 만든다.

      <log_root>/<mode>/NN_<시작>_<종료>_<role>[_pc]/NN_<시작>_<종료>_<role>[_pc].bin

    한 세션에 여러 역할 스트림 공존 가능 (예: 시뮬=leader → leader_pc + follower).
    스트림 키는 role + ('_pc' if 자기(PC)생성) — 받은 실차 데이터는 suffix 없음.
    종료시각은 stop() 에서 알게 되므로 임시 폴더(시작시각만)로 만들고 stop 에서 리네임한다.
    """

    def __init__(self, log_root):
        self._base = log_root        # data/log
        self._root = None            # data/log/<mode>
        self._idx = None
        self._start_str = None       # 첫 패킷 시각 (모든 스트림 공유)
        self._streams = {}           # key -> {'fh','count','tmp','name'}
        self._meta = {}              # 세션 메타(예: 출발 자세) → meta.json 저장
        self._lock = threading.Lock()

    def start(self, mode='Realtime'):
        """녹화 시작. mode = 상위 폴더명 ('Realtime' | 'Simulator')."""
        self._root = os.path.join(self._base, mode)
        os.makedirs(self._root, exist_ok=True)
        self._idx = next_index(self._root)
        self._start_str = None
        self._streams = {}
        self._meta = {}

    def set_meta(self, d):
        """세션 메타 설정 — stop() 시 각 스트림 폴더에 meta.json 으로 저장.
        예: {'start': [x, y, heading]} → playback 시 동일 출발 자세 복원."""
        self._meta = dict(d)

    def _stream(self, key):
        """역할 스트림(폴더+bin)을 지연 생성해 반환. 호출자가 lock 보유."""
        if key not in self._streams:
            if self._start_str is None:
                self._start_str = _now_str()
            name = f'{self._idx:02d}_{self._start_str}_{key}'
            tmp = os.path.join(self._root, name)
            os.makedirs(tmp, exist_ok=True)
            fh = open(os.path.join(tmp, name + '.bin'), 'wb')
            self._streams[key] = {'fh': fh, 'count': 0, 'tmp': tmp, 'name': name}
        return self._streams[key]

    def log(self, raw_bytes, is_pc=False):
        """검증된 60B 패킷 1개 기록. is_pc=True → 이 시뮬레이터(PC)가 생성한 데이터."""
        with self._lock:
            if self._idx is None:
                return
            key = _role_of(raw_bytes) + ('_pc' if is_pc else '')
            s = self._stream(key)
            s['fh'].write(raw_bytes)
            s['count'] += 1

    def log_hmac_fail(self, raw_bytes):
        """HMAC 검증 실패 패킷 → hmac_failed 스트림에 누적."""
        with self._lock:
            if self._idx is None:
                return
            s = self._stream('hmac_failed')
            s['fh'].write(raw_bytes)
            s['count'] += 1

    def stop(self, stop_str=None):
        """모든 스트림 파일을 닫고 폴더·bin 을 종료시각 포함 이름으로 리네임.
        생성된 폴더명 리스트 반환."""
        stop = stop_str or _now_str()
        names = []
        with self._lock:
            for key, s in self._streams.items():
                s['fh'].close()
                new_name = f'{self._idx:02d}_{self._start_str}_{stop}_{key}'
                old_bin = os.path.join(s['tmp'], s['name'] + '.bin')
                new_bin = os.path.join(s['tmp'], new_name + '.bin')
                if os.path.exists(old_bin):
                    os.rename(old_bin, new_bin)
                new_folder = os.path.join(self._root, new_name)
                if os.path.exists(s['tmp']):
                    os.rename(s['tmp'], new_folder)
                if self._meta:   # 출발 자세 등 메타를 bin 옆에 저장
                    import json
                    try:
                        with open(os.path.join(new_folder, 'meta.json'), 'w',
                                  encoding='utf-8') as mf:
                            json.dump(self._meta, mf, ensure_ascii=False)
                    except OSError:
                        pass
                names.append(new_name)
            self._streams = {}
            self._idx = None
        return names

    @property
    def packet_count(self):
        with self._lock:
            return sum(s['count'] for k, s in self._streams.items() if k != 'hmac_failed')

    @property
    def fail_count(self):
        with self._lock:
            s = self._streams.get('hmac_failed')
            return s['count'] if s else 0

    @property
    def is_recording(self):
        with self._lock:
            return self._idx is not None


class _TxTap:
    """TX 소켓 래퍼 — sendto 직전 raw bytes를 콜백으로 흘려 자기 송신 패킷을 로깅 가능케 함.
    V2VModule.step 은 self._tx.sendto(...) 만, stop 은 self._tx.close() 만 호출하므로 덕타이핑 충분."""

    def __init__(self, sock, on_tx_cb=None):
        self._sock = sock
        self._on_tx_cb = on_tx_cb

    def sendto(self, data, addr):
        if self._on_tx_cb is not None:
            self._on_tx_cb(data)
        return self._sock.sendto(data, addr)

    def close(self):
        return self._sock.close()


class RecordableV2VModule(V2VModule):
    """V2VModule subclass — 검증된 수신 패킷마다 on_packet_cb(raw), 자기 송신 패킷마다 on_tx_cb(raw)."""

    def __init__(self, role, on_packet_cb=None, on_hmac_fail_cb=None, on_tx_cb=None):
        super().__init__(role)
        self._on_packet_cb     = on_packet_cb
        self._on_hmac_fail_cb  = on_hmac_fail_cb
        if on_tx_cb is not None:
            self._tx = _TxTap(self._tx, on_tx_cb)   # 송신 패킷 캡처용 래핑

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
                if self._on_hmac_fail_cb is not None:
                    self._on_hmac_fail_cb(data)
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
