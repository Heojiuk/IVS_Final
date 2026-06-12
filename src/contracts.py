"""토픽 데이터 형식 (ICD IF-B1~B6) — 6명 모두가 공유하는 인터페이스.

각 모듈은 여기 정의된 dataclass만 버스에 publish / read 한다.
- 필드명·타입·단위가 곧 모듈 간 약속이다. 임의로 바꾸지 말 것.
- 변경이 필요하면 보고 → ICD와 함께 수정 (ASPICE 추적성).
- writer 담당이 자기 토픽 dataclass를 소유:
    scene=인지, command/mode=판단, ego_state=주행, v2v/*·link=통신

제어 2원칙: 종방향=throttle_pwm(PWM 듀티)→DC모터 후륜 / 횡방향=steer_pwm(PWM)→서보 1개.
속도센서·IMU 없음(CONS-05) → 속도/요 측정값 없음. V2V는 PWM 명령(+행동)만 교환.
주의: 버스 read 결과는 초기 사이클에 None일 수 있다. 소비 측은 None 가드 필수.
"""
from dataclasses import dataclass, field
from enum import IntEnum
from typing import List, Optional


# ── 열거형 ────────────────────────────────────────────────────────────
class Mode(IntEnum):           # decision/mode (IF-B3) · 시스템 동작 모드
    NORMAL = 0                 # 정상
    DEGRADED = 1               # 축퇴(서행)
    ESTOP = 2                  # 비상정지


class ModeCause(IntEnum):      # decision/mode.cause (IF-B3) · 모드 진입 사유
    NONE = 0                   # 없음
    LINK_LOST = 1              # 링크 두절
    LANE_LOST = 2              # 차선 미검출
    OBSTACLE = 3               # 장애물
    HEALTH = 4                 # health 이상


class DriveBehavior(IntEnum):  # decision/command.behavior (IF-B2) · 주행 행동
    FOLLOW = 0                 # 추종 (앞차/차선 따라)
    LANE_CHANGE = 1            # 변경 (차선 변경)
    STOP = 2                   # 정지 (정지선/장애물/비상)
    SLOW = 3                   # 서행 (감속)
    RELEASE = 4                # 개방 (군집 해제/독립)  TODO: 의미 확정


class LinkState(IntEnum):      # v2v/link_status.state (IF-B6)
    ALIVE = 0                  # 수신 정상
    STALE = 1                  # 지연(> LINK_STALE_MS)
    LOST = 2                   # 두절(> LINK_LOST_MS) → 안전 폴백


class Role(IntEnum):           # STATE 송신 차량 (와이어값과 동일)
    LEADER = 1                 # 선행
    FOLLOWER = 2               # 후행


# ── 토픽 페이로드 ─────────────────────────────────────────────────────
@dataclass
class Detection:               # YOLO 검출 1건 (정규화 좌표 0~1)
    cls: int = 0
    conf: float = 0.0
    cx: float = 0.0
    cy: float = 0.0
    w: float = 0.0
    h: float = 0.0


@dataclass
class Scene:                   # perception/scene  IF-B1  (인지 → 판단·주행)
    stamp: float = 0.0
    lane_valid: bool = False             # 차선 검출 유효
    lane_offset_m: float = 0.0           # 차선 중심 횡오차 (m, 좌측 +)  IF-B1
    lane_heading_rad: float = 0.0        # 차선 대비 헤딩오차 (rad)
    lane_curvature_1pm: float = 0.0      # 차선 곡률 (1/m, 좌회전 +)  IF-B1
    front_clear: bool = True             # 전방 가용(카메라+초음파 융합)
    dist_front_m: Optional[float] = None # 전방 초음파 거리(m), 미검 시 None
    stop_signal: bool = False            # 정지선/STOP 검출
    objects: List[Detection] = field(default_factory=list)


@dataclass
class DriveCommand:            # decision/command  IF-B2  (판단 → 주행·통신)
    stamp: float = 0.0
    behavior: DriveBehavior = DriveBehavior.FOLLOW   # 무슨 행동을 할지
    target_lane: int = 0                             # 변경 시 목표 차로 (1|2)


@dataclass
class ModeCmd:                 # decision/mode  IF-B3  (판단 → 전 모듈)
    stamp: float = 0.0
    mode: Mode = Mode.NORMAL
    cause: ModeCause = ModeCause.NONE    # 모드 진입 사유 (IF-B3)


@dataclass
class EgoState:                # motion/ego_state  IF-B4  (주행 → 통신)
    stamp: float = 0.0
    throttle_pwm: float = 0.0            # 구동 PWM 듀티 -1~1 (gpiozero Motor speed·방향)
    steer_pwm: float = 0.0               # 조향 PWM -1~1 (서보 PWMOutputDevice value)
    behavior: DriveBehavior = DriveBehavior.FOLLOW   # 현재 수행 중 행동 (판단 command 반영)
    # 속도센서·IMU 없음(CONS-05) → 속도·요 측정값 없음. PWM 명령 + 행동만 전송.


@dataclass
class V2VState:                # v2v/leader_state·follower_state  IF-B5  (통신 RX → 판단·주행)
    t_tx: float = 0.0                    # 송신측 송신 시각 (IF-B5 t_tx)
    role: Role = Role.LEADER             # 송신 차량
    seq: int = 0
    throttle_pwm: float = 0.0            # 상대 구동 PWM (후행 추종 피드포워드)
    steer_pwm: float = 0.0               # 상대 조향 PWM (곡선 예고)
    behavior: DriveBehavior = DriveBehavior.FOLLOW   # 상대 현재 행동
    t_rx: float = 0.0                    # 자차 수신 시각(monotonic) — 링크 age 산출 (IF-B5)


@dataclass
class LinkStatus:              # v2v/link_status  IF-B6  (통신 → 판단)
    stamp: float = 0.0
    state: LinkState = LinkState.LOST
    age_rx: float = 9999.0               # 마지막 수신 경과(ms) (IF-B6 age_rx)
    last_seq: int = 0                    # 마지막 정상 수신 seq (IF-B6 last_seq)
