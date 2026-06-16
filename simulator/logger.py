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


class SessionRecorder:
    """Manages one recording session: folder creation, bin file, rename on stop."""

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
        """Called for each verified 60B packet. first_rx_str for test injection only."""
        with self._lock:
            if self._first_rx_time is None:
                self._first_rx_time = first_rx_str or _now_str()
                self._fh = open(self._bin_path, 'wb')
            if self._fh:
                self._fh.write(raw_bytes)
                self._count += 1

    def stop(self, stop_str=None):
        """Close file, rename folder. Returns (new_folder_name, bin_path)."""
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
    """V2VModule subclass that calls on_packet_cb(raw_bytes) after each verified RX packet."""

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
