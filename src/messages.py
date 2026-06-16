"""토픽 데이터 형식 — 6명이 버스로 주고받는 데이터의 '약속된 모양' (ICD IF-B1~B6).

여기 정의된 dataclass만 버스에 올리고(publish) 읽는다(read).
- 필드 이름·타입·단위가 곧 모듈 간 약속. 마음대로 바꾸면 읽는 쪽이 전부 깨진다.
- 바꿔야 하면 먼저 공유 → ICD 문서와 같이 수정 (ASPICE 추적성).
- 각 데이터의 '주인'(만들어서 발행하는 담당):
    scene=인지, command·mode=판단, ego_state=주행, v2v·link=통신

제어 2원칙: 종방향=throttle_pwm(전·후진 세기)→DC모터 후륜 / 횡방향=steer_pwm(좌·우)→서보 1개.
속도센서·IMU 없음(CONS-05) → 실제 속도·방향 측정값은 없고, PWM 명령값만 주고받는다.
주의: 버스에서 읽은 값은 처음 몇 사이클간 None일 수 있다 → 쓰기 전에 None인지 꼭 확인.
"""
from dataclasses import dataclass, field
from enum import IntEnum
from typing import List, Optional


# ── enum (정해진 값 중 하나) ──────────────────────────────────────────
class Mode(IntEnum):           # 시스템 전체 동작 모드 (decision/mode, IF-B3)
    NORMAL = 0                 # 정상 주행
    DEGRADED = 1               # 문제 발생 → 서행 등 축소 운행
    ESTOP = 2                  # 비상 정지


class ModeCause(IntEnum):      # 위 모드로 바뀐 '이유' (decision/mode, IF-B3)
    NONE = 0                   # 사유 없음 (정상)
    LINK_LOST = 1              # V2V 통신 끊김
    LANE_LOST = 2              # 차선을 못 찾음
    OBSTACLE = 3               # 앞에 장애물
    HEALTH = 4                 # 시스템 자체 이상


class DriveBehavior(IntEnum):  # 지금 할 주행 동작 (decision/command, IF-B2)
    FOLLOW = 0                 # 추종 — 앞차·차선 따라가기
    LANE_CHANGE = 1            # 차선 바꾸기
    STOP = 2                   # 정지 — 정지선·장애물·비상
    SLOW = 3                   # 서행 — 천천히
    RELEASE = 4                # 군집 풀고 독립 주행 (TODO: 정확한 의미 미정)


class LinkState(IntEnum):      # 상대차 수신(V2V) 상태 (v2v/link_status, IF-B6)
    ALIVE = 0                  # 수신 정상
    STALE = 1                  # 수신 지연 (LINK_STALE_MS 초과)
    LOST = 2                   # 수신 끊김 (LINK_LOST_MS 초과) → 안전 폴백


class Role(IntEnum):           # 어느 차량인지 (V2V 패킷에 실리는 값과 동일)
    LEADER = 1                 # 선행차
    FOLLOWER = 2               # 후행차


# ── 토픽 메시지 (버스로 주고받는 데이터 묶음) ─────────────────────────
@dataclass
class Detection:               # 카메라가 찾은 물체 1개 (YOLO 검출 1건)
    cls: int = 0               # 물체 종류 번호 (예: 0=차, 1=사람 …)
    conf: float = 0.0          # 검출 확신도 0~1 (높을수록 확실)
    cx: float = 0.0            # 박스 중심 x, 0~1 (화면 왼→오 비율)
    cy: float = 0.0            # 박스 중심 y, 0~1 (화면 위→아래 비율)
    w: float = 0.0             # 박스 너비, 0~1 (화면 폭 대비 비율)
    h: float = 0.0             # 박스 높이, 0~1 (화면 높이 대비 비율)


@dataclass
class Scene:                   # 인지 결과: 차선·전방·물체 (perception/scene, IF-B1, 인지→판단·주행)
    stamp: float = 0.0                   # 이 데이터를 만든 시각 (초)
    lane_valid: bool = False             # 차선을 제대로 찾았나 (False면 아래 차선값 신뢰 X)
    current_lane: int = 0                # 현재 주행 차로 (1·2, 0=미확정/unknown)
    lane_offset_m: float = 0.0           # 차로 중앙에서 좌우로 벗어난 거리 (m, 왼쪽이 +)
    lane_heading_rad: float = 0.0        # 차선 방향과 차 방향의 각도 차 (rad)
    lane_curvature_1pm: float = 0.0      # 차선이 휜 정도 (1/m, 좌회전 +). 0이면 직선
    front_clear: bool = True             # 앞이 비었나 (카메라+초음파 종합 판단)
    dist_front_m: Optional[float] = None # 앞 장애물까지 거리 (m). 못 재면 None
    stop_signal: bool = False            # 정지선·STOP 표지를 봤나
    objects: List[Detection] = field(default_factory=list)   # 검출한 물체 목록 (위 Detection들)


@dataclass
class DriveCommand:            # 판단이 내리는 주행 지시 (decision/command, IF-B2, 판단→주행·통신)
    stamp: float = 0.0
    behavior: DriveBehavior = DriveBehavior.FOLLOW   # 무슨 동작을 할지 (위 DriveBehavior)
    target_lane: int = 0                             # 차선 변경 시 목표 차로 번호 (1 또는 2)


@dataclass
class ModeCmd:                 # 시스템 모드 지시 (decision/mode, IF-B3, 판단→전 모듈)
    stamp: float = 0.0
    mode: Mode = Mode.NORMAL             # 정상 / 서행 / 비상정지
    cause: ModeCause = ModeCause.NONE    # 그 모드가 된 이유


@dataclass
class EgoState:                                      # 자차의 현재 제어 상태 (motion/ego_state, IF-B4, 주행→통신)
    stamp: float = 0.0                               # 이 데이터를 만든 시각 (초)
    throttle_pwm: float = 0.0                        # 구동 세기 -1~1 (+전진 / −후진) → DC모터로 출력
    steer_pwm: float = 0.0                           # 조향 -1~1 (−왼쪽 / +오른쪽) → 서보로 출력
    behavior: DriveBehavior = DriveBehavior.FOLLOW   # 지금 수행 중인 동작 (판단 command 반영)
    # 속도센서·IMU 없음 → 실제 속도·방향 측정값은 없음. PWM 명령값과 동작만 보낸다.


@dataclass
class V2VState:                          # WiFi로 받은 상대 차량 상태 (v2v/leader_state·follower_state, IF-B5, 통신→판단·주행)
    t_tx: float = 0.0                    # 상대가 보낸 시각 (상대 monotonic 시계, 상대적)
    tx_abs: int = 0                      # 상대(송신측)가 찍은 절대 송신시각 (자정 기준 ms = HH:mm:ss.fff, 0~86399999; wire=uint32 'I')
    role: Role = Role.LEADER             # 보낸 차량 (선행 / 후행)
    seq: int = 0                         # 패킷 일련번호 (순서·중복·끊김 확인용)
    lane: int = 0                        # 상대 현재 차로 (1·2, 0=미확정)
    throttle_pwm: float = 0.0            # 상대의 구동 세기 (후행이 따라갈 때 참고)
    steer_pwm: float = 0.0               # 상대의 조향 (앞으로 곡선 올 것을 미리 알기)
    behavior: DriveBehavior = DriveBehavior.FOLLOW   # 상대가 지금 하는 동작
    t_rx: float = 0.0                    # 내가 받은 시각 → 통신 끊김(오래됨) 판단에 사용


@dataclass
class LinkStatus:              # V2V 통신 상태 (v2v/link_status, IF-B6, 통신→판단)
    stamp: float = 0.0
    state: LinkState = LinkState.LOST    # 정상 / 지연 / 끊김
    age_rx: float = 9999.0               # 마지막 수신 후 지난 시간 (ms). 클수록 오래 못 받음
    last_seq: int = 0                    # 마지막으로 제대로 받은 패킷 번호
