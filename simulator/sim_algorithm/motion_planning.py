"""시뮬레이터 mock 모션 (추종 제어) — src/algorithm/motion_planning.py 의 대역.

타 팀(모션)의 src 모듈이 아직 STUB(PWM 0)이므로, 시뮬레이터가 자체적으로
"선행차를 따라가는" 모션을 하도록 임시 구현한다. 공유 src 코드는 건드리지 않는다.

추종 모델 (FOLLOWER):
  위치·속도 센서 없음(CONS-05) → 선행차의 PWM 명령(throttle·steer)을 N틱 지연 재생.
  같은 시작점에서 출발해도 선행차가 '지난 경로'를 그대로 따라가며 일정 간격 뒤를 트레일한다.
  여기에 후행차 자신의 실시간 인지(정지선·전방장애물)·링크 상태로 안전 오버라이드를 얹는다.

src MotionModule 과 동일한 `.step(bus)` 인터페이스(role enum, step(bus)).
"""
import collections
import time

from core_module.bus import Topics
from messages import EgoState, DriveBehavior, Mode, Role

# ── 추종 파라미터 ─────────────────────────────────────────────────────
DELAY_STEPS = 12        # 명령 지연 틱 수 (12 × 50ms = 0.6s) → 트레일 간격을 만든다
SLOW_CAP    = 0.30      # SLOW/DEGRADED 시 throttle 상한
SAFE_GAP_CM = 30.0      # 이 거리(cm) 이하부터 감속 (Scene.dist_front_cm)
MIN_GAP_CM  = 15.0      # 이 거리(cm) 이하면 정지
THR_MIN, THR_MAX = -1.0, 1.0   # throttle_pwm 범위 (messages.py)
STR_MIN, STR_MAX = -1.0, 1.0   # steer_pwm 범위


def _clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


class LocalMotionModule:
    """로컬 모션 — 선행차 명령을 지연 재생해 추종 PWM 산출, EGO_STATE 발행.
    인터페이스는 src MotionModule 과 동일 (role enum, step(bus))."""

    def __init__(self, role):
        self.role = role
        # 선행차 (throttle, steer) 명령 버퍼 — DELAY_STEPS 틱 지연 재생용
        self._buf = collections.deque(maxlen=DELAY_STEPS + 1)

    def step(self, bus):
        cmd    = bus.read(Topics.COMMAND)
        mode   = bus.read(Topics.MODE)
        scene  = bus.read(Topics.SCENE)
        leader = bus.read(Topics.LEADER_STATE) if self.role == Role.FOLLOWER else None
        behavior = cmd.behavior if cmd is not None else DriveBehavior.CRUISE

        throttle, steer = 0.0, 0.0

        if self.role == Role.FOLLOWER and leader is not None:
            # 선행차 명령을 버퍼에 적재 → DELAY_STEPS 틱 전 명령을 재생 (버퍼 미충전 구간은 정지 → 간격 형성)
            self._buf.append((leader.throttle_pwm, leader.steer_pwm))
            if len(self._buf) > DELAY_STEPS:
                throttle, steer = self._buf[0]

        # ── 안전 오버라이드 (후행차 '현재' 인지 기준 — 지연 재생 위에 덮어씀) ──
        m = mode.mode if mode is not None else Mode.NORMAL
        if m == Mode.ESTOP or behavior == DriveBehavior.STOP:
            throttle = 0.0
        elif m == Mode.DEGRADED or behavior == DriveBehavior.SLOW:
            throttle = _clamp(throttle, -SLOW_CAP, SLOW_CAP)

        # 전방 거리(초음파, cm) 기반 감속/정지 — 장애물에 반응
        if scene is not None and scene.dist_front_cm is not None and throttle > 0.0:
            d = scene.dist_front_cm
            if d <= MIN_GAP_CM:
                throttle = 0.0
            elif d < SAFE_GAP_CM:
                throttle *= (d - MIN_GAP_CM) / (SAFE_GAP_CM - MIN_GAP_CM)

        throttle = _clamp(throttle, THR_MIN, THR_MAX)
        steer    = _clamp(steer,    STR_MIN, STR_MAX)
        bus.publish(Topics.EGO_STATE, EgoState(stamp=time.monotonic(),
                                               throttle_pwm=throttle, steer_pwm=steer,
                                               behavior=behavior))
