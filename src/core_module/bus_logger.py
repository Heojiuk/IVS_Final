"""버스 로거 — dev 모드에서 매 50ms 사이클의 전 토픽 스냅샷을 바이너리로 기록 (디버깅용).

목적: "11:50에 판단이 LANE_CHANGE를 냈을 때 인지가 무슨 scene을 넘겼나"처럼
      같은 사이클의 인지·판단·모션·통신 데이터를 한 줄로 묶어 사후 추적.

설계:
  - Scheduler 마지막 모듈로 자동 삽입(dev 모드만) → 그 사이클에 갓 쓰인 scene·command·…를 모두 캡처.
  - 고정폭 레코드 바이너리(.buslog) → pack/write 만, 지연 거의 0. flush 매 사이클(syscall ~µs, durability).
  - 파일이 곧 포맷의 진실원: writer(BusLoggerModule)와 reader(read_file)가 같은 RECORD_FMT 공유.
  - 후처리(CSV/xlsx/pdf)는 simulator/tools/parse_buslog.py 가 read_file()을 불러 수행.

파일 레이아웃:
  [헤더 16B]  magic(8s) version(B) role(B) record_size(H) reserved(I)
  [레코드 ×N] 아래 RECORD_FMT(고정폭) 연속

None 처리: 토픽 미발행(초기 몇 사이클)은 valid_mask 비트=0 으로 표시, 값은 0.
           dist_front_cm(미감지=None)은 NaN 으로 저장 → reader가 None 복원.
"""
import os
import struct
import datetime
import atexit

from core_module.bus import Topics
from messages import DriveBehavior, Mode, ModeCause, LinkState, Role

# ── 파일/레코드 포맷 (writer·reader 공유 = 단일 진실원) ───────────────────────
MAGIC = b"IVSBUS01"           # 8바이트 식별자
VERSION = 1
HEADER_FMT = "!8sBBHI"        # magic·version·role·record_size·reserved = 16B
HEADER_SIZE = struct.calcsize(HEADER_FMT)

# 레코드 필드 순서 = RECORD_FMT 순서 = FIELD_NAMES 순서 (셋이 항상 일치해야 함)
RECORD_FMT = "!IIB" "BBBBffff" "BB" "BB" "ffB" "HBBff" "BfH"
RECORD_SIZE = struct.calcsize(RECORD_FMT)   # 61B
FIELD_NAMES = [
    "tick", "t_abs_ms", "valid_mask",
    # SCENE (인지)
    "lane_valid", "current_lane", "front_clear", "stop_signal",
    "lane_offset_cm", "lane_heading_rad", "lane_curvature_1pm", "dist_front_cm",
    # COMMAND (판단)
    "behavior", "target_lane",
    # MODE (판단)
    "mode", "cause",
    # EGO_STATE (모션)
    "throttle_pwm", "steer_pwm", "ego_behavior",
    # PEER_STATE (V2V 수신 — leader면 follower_state, follower면 leader_state)
    "peer_seq", "peer_lane", "peer_behavior", "peer_throttle", "peer_steer",
    # LINK_STATUS
    "link_state", "link_age_ms", "link_last_seq",
]

# valid_mask 비트
_BIT_SCENE, _BIT_CMD, _BIT_MODE, _BIT_EGO, _BIT_PEER, _BIT_LINK = (1 << i for i in range(6))

LOG_DIR = os.path.normpath(
    os.path.join(os.path.dirname(__file__), "..", "..", "data", "log", "DevBus")
)


def _now_ms_of_day():
    """현재 벽시계 시각을 자정 기준 ms(0~86_399_999)로. V2V tx_abs·bin 분석과 동일 도메인 → 타임라인 정렬."""
    n = datetime.datetime.now()
    return ((n.hour * 60 + n.minute) * 60 + n.second) * 1000 + n.microsecond // 1000


# ── Writer (스케줄러 모듈) ────────────────────────────────────────────────────
class BusLoggerModule:
    """스케줄러 마지막 모듈 — 매 사이클 전 토픽을 읽어 고정폭 레코드 1개를 파일에 append.

    role: Role enum(자차 역할) 또는 None. 파일명·헤더·읽을 peer 토픽 결정.
    파일 열기 실패는 호출측(Scheduler)이 격리 → 로깅 못해도 주행은 계속.
    """

    def __init__(self, role=None, log_dir=None):
        self._role = role
        role_id = int(role) if role is not None else 0
        self._role_name = {Role.LEADER: "leader", Role.FOLLOWER: "follower"}.get(role, "node")
        # leader 버스엔 follower_state, follower 버스엔 leader_state 가 채워짐
        self._peer_topic = (
            Topics.LEADER_STATE if role == Role.FOLLOWER else Topics.FOLLOWER_STATE
        )

        d = log_dir or LOG_DIR
        os.makedirs(d, exist_ok=True)                 # 없으면 생성, 있으면 통과(에러X)
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = os.path.join(d, f"{self._role_name}_{ts}.buslog")
        self._f = open(self.path, "wb")               # 버퍼드 — flush는 step마다
        self._f.write(struct.pack(HEADER_FMT, MAGIC, VERSION, role_id, RECORD_SIZE, 0))
        self._f.flush()
        self._tick = 0
        atexit.register(self.close)                   # 정상 종료(Ctrl+C 포함) 시 닫기
        print(f"[buslog] dev 버스 로깅 시작 -> {self.path}")

    def step(self, bus):
        if self._f is None:
            return
        scene = bus.read(Topics.SCENE)
        cmd   = bus.read(Topics.COMMAND)
        mode  = bus.read(Topics.MODE)
        ego   = bus.read(Topics.EGO_STATE)
        peer  = bus.read(self._peer_topic)
        link  = bus.read(Topics.LINK_STATUS)

        mask = ((_BIT_SCENE if scene is not None else 0)
                | (_BIT_CMD  if cmd  is not None else 0)
                | (_BIT_MODE if mode is not None else 0)
                | (_BIT_EGO  if ego  is not None else 0)
                | (_BIT_PEER if peer is not None else 0)
                | (_BIT_LINK if link is not None else 0))

        dist = (scene.dist_front_cm if (scene and scene.dist_front_cm is not None)
                else float("nan"))
        vals = [
            self._tick, _now_ms_of_day(), mask,
            # scene
            int(bool(scene.lane_valid)) if scene else 0,
            scene.current_lane if scene else 0,
            int(bool(scene.front_clear)) if scene else 0,
            int(bool(scene.stop_signal)) if scene else 0,
            scene.lane_offset_cm if scene else 0.0,
            scene.lane_heading_rad if scene else 0.0,
            scene.lane_curvature_1pm if scene else 0.0,
            dist,
            # command
            int(cmd.behavior) if cmd else 0,
            cmd.target_lane if cmd else 0,
            # mode
            int(mode.mode) if mode else 0,
            int(mode.cause) if mode else 0,
            # ego
            ego.throttle_pwm if ego else 0.0,
            ego.steer_pwm if ego else 0.0,
            int(ego.behavior) if ego else 0,
            # peer
            peer.seq if peer else 0,
            peer.lane if peer else 0,
            int(peer.behavior) if peer else 0,
            peer.throttle_pwm if peer else 0.0,
            peer.steer_pwm if peer else 0.0,
            # link
            int(link.state) if link else 0,
            link.age_rx if link else 0.0,
            link.last_seq if link else 0,
        ]
        try:
            self._f.write(struct.pack(RECORD_FMT, *vals))
            self._f.flush()                            # ~µs syscall — durability
        except (OSError, ValueError) as e:
            print(f"[buslog] write 실패 - 로깅 중단(주행 계속): {e!r}")
            self.close()
            return
        self._tick += 1

    def close(self):
        """파일을 닫는다(중복 호출 안전). atexit/Scheduler 종료 시 호출."""
        if self._f is not None:
            try:
                self._f.flush()
                self._f.close()
            except OSError:
                pass
            self._f = None


# ── Reader (후처리에서 import) ────────────────────────────────────────────────
_ENUM = {
    "behavior": DriveBehavior, "ego_behavior": DriveBehavior, "peer_behavior": DriveBehavior,
    "mode": Mode, "cause": ModeCause, "link_state": LinkState,
}
_VALID_BITS = {
    "scene": _BIT_SCENE, "command": _BIT_CMD, "mode_topic": _BIT_MODE,
    "ego": _BIT_EGO, "peer": _BIT_PEER, "link": _BIT_LINK,
}


def decode_record(raw):
    """RECORD_SIZE 바이트 → dict. enum은 이름 문자열로, NaN dist는 None으로 복원."""
    vals = struct.unpack(RECORD_FMT, raw)
    rec = dict(zip(FIELD_NAMES, vals))
    # dist_front_cm NaN → None
    if rec["dist_front_cm"] != rec["dist_front_cm"]:   # NaN 자기불일치
        rec["dist_front_cm"] = None
    # enum int → 이름
    for k, enum_cls in _ENUM.items():
        try:
            rec[k] = enum_cls(rec[k]).name
        except ValueError:
            pass   # 미지정 값은 정수 유지
    # bool 복원
    for k in ("lane_valid", "front_clear", "stop_signal"):
        rec[k] = bool(rec[k])
    return rec


def read_file(path):
    """.buslog 파일 → (header_dict, [record_dict, ...]).

    header_dict = {version, role, role_name, record_size, count}
    잘린 마지막 레코드는 버린다(부분 write 안전).
    """
    with open(path, "rb") as f:
        head = f.read(HEADER_SIZE)
        if len(head) < HEADER_SIZE:
            raise ValueError(f"헤더 부족: {len(head)}B < {HEADER_SIZE}B")
        magic, version, role_id, rec_size, _ = struct.unpack(HEADER_FMT, head)
        if magic != MAGIC:
            raise ValueError(f"매직 불일치: {magic!r} (.buslog 아님)")
        body = f.read()

    role_name = {1: "leader", 2: "follower"}.get(role_id, "node")
    n = len(body) // rec_size
    records = [decode_record(body[i * rec_size:(i + 1) * rec_size]) for i in range(n)]
    header = {"version": version, "role": role_id, "role_name": role_name,
              "record_size": rec_size, "count": n}
    return header, records
