"""통신 (DD-COM) — V2V 송수신. 이 한 파일이 통신 담당의 전부.

  TX: 스케줄러가 매 50ms step() 호출 → 자차 PWM 명령+행동을 STATE 패킷으로 송신 + link_status 갱신.
  RX: 별도 스레드 → 수신·HMAC 검증·디코드 → 버스에 V2VState·link_status 기록.

STATE 패킷 = 본문 28B + HMAC-SHA256 32B = 60B.
  본문 '!BBBHdIBBffx' = ver type role seq t_tx(monotonic) tx_abs(절대 HH:mm:ss.fff) · [인지]lane · [판단]behavior · [모션]throttle_pwm steer_pwm · rsv
  (CONS-05: 속도·요 측정 불가 → PWM 명령 throttle_pwm·steer_pwm + 행동 behavior + 차로 lane 만 전송)
"""

import socket
import struct
import hmac
import hashlib
import threading
import time
import sys

from core_module import config
from core_module.bus import Topics
from messages import V2VState, LinkStatus, LinkState, Role, DriveBehavior

# ── STATE 패킷 코덱 ───────────────────────────────────────────────────
# 본문 28B: [헤더] ver·type·role·seq·t_tx(monotonic)·tx_abs(절대시각 ms-of-day) · [인지] lane · [판단] behavior · [모션] throttle·steer · rsv(1)
_FMT = "!BBBHdIBBffx"  # I=tx_abs(자정 기준 ms), x=reserved 1B(송신 0·수신 무시)
_HDR = struct.calcsize(_FMT)  # 28
PACKET_LEN = _HDR + 32  # 60
_VER, _STATE = 1, 1


def _tx_abs():
    """현재 시스템 시각을 자정 기준 ms(0~86_399_999)로 — 패킷에 절대 송신시각(HH:mm:ss.fff) 싣는 용도.  파라미터 없음"""
    t = time.time()
    lt = time.localtime(t)
    return ((lt.tm_hour * 3600 + lt.tm_min * 60 + lt.tm_sec) * 1000 + int((t % 1) * 1000)) % 86_400_000


def fmt_ms_of_day(ms):
    """ms-of-day(0~86_399_999) → 'HH:MM:SS.fff' 문자열 (로그·검증 표시용).  ms=자정 기준 밀리초"""
    h, ms = divmod(int(ms) % 86_400_000, 3_600_000)
    m, ms = divmod(ms, 60_000)
    s, ms = divmod(ms, 1000)
    return f"{h:02d}:{m:02d}:{s:02d}.{ms:03d}"


def packet_generator(ego, lane, role, seq, key):
    """EgoState(+차로)를 60B STATE 패킷(bytes)으로 직렬화한다(본문 28B + HMAC 32B).
    송신 절대시각(tx_abs=자정 기준 ms)은 이 함수에서 자동으로 찍는다(leader/follower 공통).
    ego=자차상태(throttle_pwm·steer_pwm·behavior), lane=현재 차로(1·2·0), role=송신차량(Role 1선행/2후행), seq=일련번호(0~65535), key=HMAC PSK
    """
    body = struct.pack(
        _FMT,
        _VER,
        _STATE,
        int(role),
        seq & 0xFFFF,
        ego.stamp,  # t_tx (monotonic, 상대적)
        _tx_abs(),  # tx_abs (절대 HH:mm:ss.fff = 자정 기준 ms)
        int(lane),  # [인지]
        int(ego.behavior),  # [판단]
        ego.throttle_pwm,  # [모션]
        ego.steer_pwm,
    )
    return body + hmac.new(key, body, hashlib.sha256).digest()


def packet_parser(pkt, key):
    """수신 60B 패킷을 검증·역직렬화해 V2VState로 만든다. 길이/HMAC/버전 불일치 시 ValueError(폐기).
    pkt=수신 바이트열(60B), key=HMAC PSK(송신측과 동일)"""
    if len(pkt) != PACKET_LEN:
        raise ValueError(f"길이 오류 {len(pkt)}≠{PACKET_LEN}")
    body, mac = pkt[:_HDR], pkt[_HDR:]
    if not hmac.compare_digest(mac, hmac.new(key, body, hashlib.sha256).digest()):
        raise ValueError("HMAC 불일치 — 위변조/키 불일치 패킷 폐기")
    ver, typ, role, seq, t, tx_abs, lane, beh, thr, st = struct.unpack(_FMT, body)
    if ver != _VER or typ != _STATE:
        raise ValueError(f"미지원 패킷 ver={ver} type={typ}")
    return V2VState(
        t_tx=t,
        tx_abs=tx_abs,
        role=Role(role),
        seq=seq,
        lane=lane,
        throttle_pwm=thr,
        steer_pwm=st,
        behavior=DriveBehavior(beh),
    )


# ── 통신 모듈 (V2V) ───────────────────────────────────────────────────
class V2VModule:
    def __init__(self, role):
        """통신 모듈 초기화 — 역할별 포트로 송수신 소켓을 열고 RX 스레드를 준비한다.  role='leader'|'follower'(자차 역할)"""
        cfg = config.for_role(role)
        self._role = Role.LEADER if role == "leader" else Role.FOLLOWER
        self._key = config.load_key()
        self._peer = (cfg["peer_ip"], cfg["peer_port"])
        # 후행 ← 선행 STATE → LEADER_STATE / 선행 ← 후행 STATE → FOLLOWER_STATE
        self._rx_topic = (
            Topics.LEADER_STATE if role == "follower" else Topics.FOLLOWER_STATE
        )
        self._tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._rx.bind(("0.0.0.0", cfg["rx_port"]))
        self._rx.settimeout(0.5)
        self._seq = 0
        self._tx_fail = 0  # TX 송신 실패 누적
        self._lock = threading.Lock()  # _last_rx · _rx_seq 크로스스레드 보호
        self._last_rx = None  # 마지막 수신 monotonic 시각
        self._rx_seq = None  # 마지막 채택 seq (None=아직 수신 없음)
        self._bus = None
        self._stop = threading.Event()
        self._thread = threading.Thread(
            target=self._rx_loop, name="v2v-rx", daemon=True
        )

    # 스케줄러 호출 — 송신(TX)
    def step(self, bus):
        """50ms 주기 — 자차 ego_state(+scene 차로)를 STATE 패킷으로 송신하고 link_status를 갱신.  bus=메시지버스"""
        ego = bus.read(Topics.EGO_STATE)  # 입력 IF-B4 (throttle_pwm·steer_pwm·behavior)
        if ego is not None:
            scene = bus.read(Topics.SCENE)  # 입력 IF-B1 (current_lane)
            lane = scene.current_lane if scene is not None else 0
            self._seq = (self._seq + 1) & 0xFFFF
            try:
                self._tx.sendto(
                    packet_generator(ego, lane, self._role, self._seq, self._key),
                    self._peer,
                )
            except OSError as e:
                self._tx_fail += 1  # 송신 실패 관측성 (첫 1회 + 100회마다 1줄)
                if self._tx_fail == 1 or self._tx_fail % 100 == 0:
                    print(
                        f"[v2v] TX send failed x{self._tx_fail} to {self._peer}: {e}",
                        file=sys.stderr,
                    )
        bus.publish(Topics.LINK_STATUS, self._link_status())  # IF-B6

    # 별도 스레드 — 수신(RX), 비동기
    def _rx_loop(self):
        """수신 스레드 루프 — 상대 STATE를 받아 검증·디코드 후 버스에 V2VState를 기록한다.  파라미터 없음
        link_status는 step()(main 스레드)에서만 게시 — 이중 게시로 인한 역행 방지."""
        while not self._stop.is_set():
            try:
                data, _addr = self._rx.recvfrom(
                    2048
                )  # UDP 최대 크기보다 넉넉히 큰 버퍼 (실제 패킷은 60B = PACKET_LEN)
            except socket.timeout:
                continue
            except OSError:
                if self._stop.is_set():
                    break  # stop()의 소켓 close → RX 스레드 정상 종료
                continue  # 일시적 수신 오류(큰/오염 데이터그램 등) → 계속 수신

            try:
                state = packet_parser(data, self._key)
            except ValueError:
                continue  # 길이/HMAC/버전 불일치 폐기

            now = time.monotonic()
            with self._lock:
                # seq 순차성: 더 최신 패킷만 채택 (16bit wrap-aware). 중복·과거(재정렬/replay) 폐기.
                last = self._rx_seq
                if last is not None:
                    delta = (state.seq - last) & 0xFFFF
                    if delta == 0 or delta >= 0x8000:
                        continue  # 폐기 — _last_rx 미갱신(옛 패킷이 link 못 살림)
                self._rx_seq = state.seq
                self._last_rx = now
            state.t_rx = now  # 수신 시각 기록 (IF-B5)
            self._bus.publish(self._rx_topic, state)  # IF-B5

    def _link_status(self):
        """마지막 수신 경과시간으로 링크 상태(ALIVE/STALE/LOST)를 판정해 LinkStatus를 반환한다.  파라미터 없음"""
        now = time.monotonic()
        with self._lock:
            last_rx = self._last_rx  # 마지막 수신 시각 (None=아직 수신 없음)
            rx_seq = (
                self._rx_seq if self._rx_seq is not None else 0
            )  # 마지막 채택 seq (None=아직 수신 없음 → 0으로 간주)
        if last_rx is None:
            return LinkStatus(
                stamp=now, state=LinkState.LOST, age_rx=9999.0, last_seq=rx_seq
            )
        age = (now - last_rx) * 1000.0  # ms 단위
        if age < config.LINK_STALE_MS:  # 50ms 미만 → ALIVE
            state = LinkState.ALIVE  # 50ms 이상 200ms 미만 → STALE, 200ms 이상 → LOST
        elif age < config.LINK_LOST_MS:  # 200ms 이상 500ms 미만 → STALE
            state = LinkState.STALE  # 200ms 이상 → LOST
        else:
            state = LinkState.LOST  # 500ms 이상 → LOST
        return LinkStatus(stamp=now, state=state, age_rx=age, last_seq=rx_seq)

    # 생명주기 (main 이 호출, 스케줄러 아님)
    def start(self, bus):
        """RX 스레드를 기동한다 (main 이 1회 호출).  bus=수신 결과를 기록할 메시지버스"""
        self._bus = bus
        self._thread.start()

    def stop(self):
        """RX 스레드를 멈추고 송수신 소켓을 닫는다 (종료 시 1회).  파라미터 없음"""
        self._stop.set()
        self._tx.close()
        self._rx.close()
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)  # RX 스레드 정상 종료 대기
