"""통신 (DD-COM) — V2V 송수신. 이 한 파일이 통신 담당의 전부.

  TX: 스케줄러가 매 50ms step() 호출 → 자차 PWM 명령+행동을 STATE 패킷으로 송신 + link_status 갱신.
  RX: 별도 스레드 → 수신·HMAC 검증·디코드 → 버스에 V2VState·link_status 기록.

STATE 패킷 = 본문 22B + HMAC-SHA256 32B = 54B.
  본문 '!BBBHdffB' = ver type role seq t_tx throttle_pwm steer_pwm behavior
  (CONS-05: 속도·요 측정 불가 → PWM 명령 throttle_pwm·steer_pwm + 행동 behavior 만 전송)
"""
import socket
import struct
import hmac
import hashlib
import threading
import time

import config
from bus import Topics
from contracts import V2VState, LinkStatus, LinkState, Role, DriveBehavior

# ── STATE 패킷 코덱 ───────────────────────────────────────────────────
_FMT = "!BBBHdffB"
_HDR = struct.calcsize(_FMT)        # 22
PACKET_LEN = _HDR + 32              # 54
_VER, _STATE = 1, 1


def pack_state(ego, role, seq, key):
    """EgoState(throttle_pwm·steer_pwm·behavior) → 54B STATE 패킷."""
    body = struct.pack(_FMT, _VER, _STATE, int(role), seq & 0xFFFF,
                       ego.stamp, ego.throttle_pwm, ego.steer_pwm, int(ego.behavior))
    return body + hmac.new(key, body, hashlib.sha256).digest()


def unpack_state(pkt, key):
    """54B 패킷 → V2VState. 길이/HMAC/버전 불일치 시 ValueError(폐기)."""
    if len(pkt) != PACKET_LEN:
        raise ValueError(f"길이 오류 {len(pkt)}≠{PACKET_LEN}")
    body, mac = pkt[:_HDR], pkt[_HDR:]
    if not hmac.compare_digest(mac, hmac.new(key, body, hashlib.sha256).digest()):
        raise ValueError("HMAC 불일치 — 위변조/키 불일치 패킷 폐기")
    ver, typ, role, seq, t, thr, st, beh = struct.unpack(_FMT, body)
    if ver != _VER or typ != _STATE:
        raise ValueError(f"미지원 패킷 ver={ver} type={typ}")
    return V2VState(t_tx=t, role=Role(role), seq=seq,
                    throttle_pwm=thr, steer_pwm=st, behavior=DriveBehavior(beh))


# ── 통신 모듈 ─────────────────────────────────────────────────────────
class CommModule:
    def __init__(self, role):
        cfg = config.for_role(role)
        self._role = Role.LEADER if role == "leader" else Role.FOLLOWER
        self._key = config.load_key()
        self._peer = (cfg["peer_ip"], cfg["peer_port"])
        # 후행 ← 선행 STATE → LEADER_STATE / 선행 ← 후행 STATE → FOLLOWER_STATE
        self._rx_topic = Topics.LEADER_STATE if role == "follower" else Topics.FOLLOWER_STATE
        self._tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._rx.bind(("0.0.0.0", cfg["rx_port"]))
        self._rx.settimeout(0.5)
        self._seq = 0
        self._last_rx = None       # 마지막 수신 monotonic 시각
        self._rx_seq = 0
        self._bus = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._rx_loop, name="v2v-rx", daemon=True)

    # 스케줄러 호출 — 송신(TX)
    def step(self, bus):
        ego = bus.read(Topics.EGO_STATE)               # 입력 IF-B4 (throttle_pwm·steer_pwm·behavior)
        if ego is not None:
            self._seq = (self._seq + 1) & 0xFFFF
            try:
                self._tx.sendto(pack_state(ego, self._role, self._seq, self._key), self._peer)
            except OSError:
                pass                                   # TODO: 송신 실패 카운트/경보
        bus.publish(Topics.LINK_STATUS, self._link_status())   # IF-B6

    # 별도 스레드 — 수신(RX), 비동기
    def _rx_loop(self):
        while not self._stop.is_set():
            try:
                data, _addr = self._rx.recvfrom(256)
            except socket.timeout:
                continue
            try:
                state = unpack_state(data, self._key)
            except ValueError:
                continue                               # 위변조/길이 오류 폐기
            now = time.monotonic()
            self._last_rx = now
            self._rx_seq = state.seq
            state.t_rx = now                           # 수신 시각 기록 (IF-B5)
            self._bus.publish(self._rx_topic, state)               # IF-B5
            self._bus.publish(Topics.LINK_STATUS, self._link_status())  # IF-B6

    def _link_status(self):
        now = time.monotonic()
        if self._last_rx is None:
            return LinkStatus(stamp=now, state=LinkState.LOST, age_rx=9999.0, last_seq=self._rx_seq)
        age = (now - self._last_rx) * 1000.0
        if age < config.LINK_STALE_MS:
            state = LinkState.ALIVE
        elif age < config.LINK_LOST_MS:
            state = LinkState.STALE
        else:
            state = LinkState.LOST
        return LinkStatus(stamp=now, state=state, age_rx=age, last_seq=self._rx_seq)

    # 생명주기 (main 이 호출, 스케줄러 아님)
    def start(self, bus):
        self._bus = bus
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._tx.close()
        self._rx.close()
