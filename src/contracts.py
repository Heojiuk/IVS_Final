"""토픽 데이터 형식 (ICD IF-B1~B6) — 6명 모두가 공유하는 인터페이스.

각 모듈은 여기 정의된 dataclass만 버스에 publish / read 한다.
- 필드명·타입·단위가 곧 모듈 간 약속이다. 임의로 바꾸지 말 것.
- 변경이 필요하면 보고 → ICD와 함께 수정 (ASPICE 추적성).
- writer 담당이 자기 토픽 dataclass를 소유:
    scene=인지, maneuver/mode=판단, ego_state=주행, v2v/*·link=통신

주의: 버스 read 결과는 초기 사이클에 None일 수 있다. 소비 측은 None 가드 필수.
"""
from dataclasses import dataclass, field
from enum import IntEnum
from typing import List, Optional


# ── 열거형 (패킷에 정수로 실리므로 IntEnum) ───────────────────────────
class Mode(IntEnum):           # decision/mode (IF-B3) · STATE.mode
    NORMAL = 0                 # 정상
    DEGRADED = 1               # 서행 — 통신 두절 등
    ESTOP = 2                  # 비상정지


class ManeuverType(IntEnum):   # decision/maneuver.type (IF-B2)
    CRUISE = 0                 # 차선 추종 순항
    FOLLOW = 1                 # V2V/CACC 거리 추종
    AVOID = 2                  # 장애물 회피
    STOP = 3                   # 정지(정지선/비상)
    LANE_CHANGE = 4            # 차선 변경


class LinkState(IntEnum):      # v2v/link_status.state (IF-B6)
    ALIVE = 0                  # 수신 정상
    STALE = 1                  # 지연(> LINK_STALE_MS 미수신)
    LOST = 2                   # 두절(> LINK_LOST_MS 미수신) → 안전 폴백


class Role(IntEnum):
    LEADER = 0                 # 선행
    FOLLOWER = 1               # 후행


# ── 토픽 페이로드 ─────────────────────────────────────────────────────
@dataclass
class Detection:               # YOLO 검출 1건 (정규화 좌표 0~1)
    cls: int = 0               # 클래스 id
    conf: float = 0.0          # 신뢰도
    cx: float = 0.0            # 중심 x
    cy: float = 0.0            # 중심 y
    w: float = 0.0             # 폭
    h: float = 0.0             # 높이


@dataclass
class Scene:                   # perception/scene  IF-B1  (인지 → 판단·주행)
    stamp: float = 0.0
    lane_valid: bool = False             # 차선 검출 유효
    lane_offset_m: float = 0.0           # 차선 중심 횡오차 (+우)
    lane_heading_rad: float = 0.0        # 차선 대비 헤딩오차
    front_clear: bool = True             # 전방 가용(카메라+초음파 융합)
    dist_front_m: Optional[float] = None # 전방 초음파 거리(m), 미검 시 None
    stop_signal: bool = False            # 정지선/STOP 검출
    objects: List[Detection] = field(default_factory=list)


@dataclass
class Maneuver:                # decision/maneuver  IF-B2  (판단 → 주행·통신)
    stamp: float = 0.0
    type: ManeuverType = ManeuverType.CRUISE
    target_speed: float = 0.0            # 목표 속도(정규화 0~1, 설계 확정 TODO)
    target_lane: int = 0                 # 차선 변경 목표(상대 ±1)
    gap_target_m: float = 0.0            # 추종(FOLLOW) 목표 차간거리


@dataclass
class ModeCmd:                 # decision/mode  IF-B3  (판단 → 전 모듈)
    stamp: float = 0.0
    mode: Mode = Mode.NORMAL


@dataclass
class EgoState:                # motion/ego_state  IF-B4  (주행 → 통신) == STATE 4 float
    stamp: float = 0.0
    throttle_cmd: float = 0.0            # 구동 듀티 명령 (-1~1)
    steer_cmd: float = 0.0               # 조향 명령 (-1~1)
    v_est: float = 0.0                   # 속도 추정(텔레메트리, CONS-05: 측정 불가→추정)
    yaw_est: float = 0.0                 # 요 추정(텔레메트리, CONS-05)


@dataclass
class V2VState:                # v2v/leader_state·follower_state  IF-B5  (통신 RX → 판단·주행)
    stamp: float = 0.0                   # 송신측 t_tx
    role: Role = Role.LEADER             # 송신 차량 역할
    seq: int = 0
    mode: Mode = Mode.NORMAL
    throttle_cmd: float = 0.0
    steer_cmd: float = 0.0
    v_est: float = 0.0
    yaw_est: float = 0.0


@dataclass
class LinkStatus:              # v2v/link_status  IF-B6  (통신 → 판단)
    stamp: float = 0.0
    state: LinkState = LinkState.LOST
    age_ms: float = 9999.0               # 마지막 수신 경과(ms)
    rx_seq: int = 0                      # 마지막 수신 시퀀스
