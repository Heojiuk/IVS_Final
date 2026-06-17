"""시뮬레이터 mock 판단 (추종 제어) — src/algorithm/decision.py 의 대역.

타 팀(판단)의 src 모듈이 아직 STUB(PWM 0)이므로, 시뮬레이터가 자체적으로
"선행차를 따라가는" 판단을 하도록 임시 구현한다. 공유 src 코드는 건드리지 않는다.

src DecisionModule 과 동일한 `.step(bus)` 인터페이스(role enum, step(bus)) →
VILSEngine 이 use_local_control 플래그로 실제 src 모듈과 교체 가능.
"""
import time

from core_module.bus import Topics
from messages import (DriveCommand, ModeCmd, DriveBehavior,
                      Mode, ModeCause, Role, LinkState)


class LocalDecisionModule:
    """로컬 판단 — SCENE·LEADER_STATE·LINK 으로 behavior·mode 결정, COMMAND·MODE 발행.
    인터페이스는 src DecisionModule 과 동일 (role enum, step(bus))."""

    def __init__(self, role):
        self.role = role

    def step(self, bus):
        scene = bus.read(Topics.SCENE)
        link  = bus.read(Topics.LINK_STATUS)
        leader = bus.read(Topics.LEADER_STATE) if self.role == Role.FOLLOWER else None

        behavior = DriveBehavior.CRUISE
        mode, cause = Mode.NORMAL, ModeCause.NONE

        # 1) 자기 인지 기반 안전 (최우선): 정지선·전방 장애물 → 정지
        if scene is not None and (scene.stop_signal or not scene.front_clear):
            behavior = DriveBehavior.STOP
            cause = ModeCause.OBSTACLE if not scene.front_clear else ModeCause.NONE
        # 2) 통신 끊김 → 안전 폴백 (선행차를 못 보면 정지)
        elif self.role == Role.FOLLOWER and link is not None and link.state == LinkState.LOST:
            behavior = DriveBehavior.STOP
            mode, cause = Mode.DEGRADED, ModeCause.LINK_LOST
        # 3) 정상: 선행차 동작을 미러 (FOLLOW/LANE_CHANGE/SLOW…)
        elif leader is not None:
            behavior = leader.behavior

        target_lane = leader.lane if (leader is not None and behavior == DriveBehavior.LANE_CHANGE) else 0
        bus.publish(Topics.COMMAND, DriveCommand(stamp=time.monotonic(),
                                                 behavior=behavior, target_lane=target_lane))
        bus.publish(Topics.MODE, ModeCmd(stamp=time.monotonic(), mode=mode, cause=cause))
