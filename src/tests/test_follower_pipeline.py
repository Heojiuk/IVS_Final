"""후행차 풀 파이프라인 E2E — 실제 UDP로 선행차 STATE 수신 → 판단 COMMAND 까지.

흐름:  leader V2VModule(송신) --UDP--> follower V2VModule(수신·LEADER_STATE/LINK_STATUS 발행)
       --> DecisionModule(FOLLOWER).step --> COMMAND
선행차 behavior(CRUISE/STOP/LANE_CHANGE)가 V2V로 전달돼 후행 판단에 반영되는지 통합 검증.

실행:  cd src && python tests/test_follower_pipeline.py
"""
import os
import sys
import time

os.environ["IVS_MODE"] = "loopback"   # 127.0.0.1 송수신 (V2VModule 생성 전 설정)
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from core_module.bus import MessageBus, Topics                  # noqa: E402
from core_module.v2v import V2VModule                           # noqa: E402
from algorithm.decision import DecisionModule, FOLLOW_SLOW_CM   # noqa: E402
from messages import (EgoState, Scene, DriveBehavior, LinkState,  # noqa: E402
                      Role)


def _run_case(leader_behavior, foll_lane=2):
    """선행차가 leader_behavior 로 송신할 때, 후행 판단 COMMAND 와 수신 LEADER_STATE 반환.
    매 케이스 독립: 모듈·버스·판단 새로 생성."""
    lead_bus, foll_bus = MessageBus(), MessageBus()
    lead, foll = V2VModule("leader"), V2VModule("follower")
    decision = DecisionModule(Role.FOLLOWER)

    # 선행차가 송신할 자차 상태 (behavior 가 패킷에 실림)
    lead_bus.publish(Topics.EGO_STATE,
                     EgoState(throttle_pwm=0.5, steer_pwm=0.0, behavior=leader_behavior))
    lead_bus.publish(Topics.SCENE, Scene(current_lane=foll_lane))
    # 후행차 자체 인지(차선 정상, 선행차까지 거리 멀어 거리트리거 없음)
    foll_bus.publish(Topics.SCENE,
                     Scene(lane_valid=True, current_lane=foll_lane,
                           front_clear=True, dist_front_cm=FOLLOW_SLOW_CM + 50))
    foll_bus.publish(Topics.EGO_STATE,
                     EgoState(throttle_pwm=0.3, steer_pwm=0.0, behavior=DriveBehavior.CRUISE))

    lead.start(lead_bus)
    foll.start(foll_bus)
    try:
        for _ in range(6):
            lead.step(lead_bus)
            foll.step(foll_bus)
            time.sleep(0.05)
        time.sleep(0.03)                       # 마지막 패킷 RX 처리 여유 (age<200ms → ALIVE)

        leader_state = foll_bus.read(Topics.LEADER_STATE)
        link = foll_bus.read(Topics.LINK_STATUS)
        decision.step(foll_bus)                # 인지·링크·선행상태 → COMMAND 발행
        command = foll_bus.read(Topics.COMMAND)
        return command, leader_state, link
    finally:
        lead.stop()
        foll.stop()


def test_leader_cruise_follower_cruise():
    cmd, ls, link = _run_case(DriveBehavior.CRUISE)
    assert ls is not None and ls.behavior == DriveBehavior.CRUISE, "선행 CRUISE 미수신"
    assert link.state == LinkState.ALIVE, f"링크 ALIVE 아님: {link.state.name}"
    assert cmd.behavior == DriveBehavior.CRUISE, f"후행 CRUISE 아님: {cmd.behavior.name}"


def test_leader_stop_follower_stop():
    cmd, ls, link = _run_case(DriveBehavior.STOP)
    assert ls is not None and ls.behavior == DriveBehavior.STOP, "선행 STOP 미수신"
    assert cmd.behavior == DriveBehavior.STOP, f"후행 STOP 아님: {cmd.behavior.name}"


def test_leader_lane_change_follower_follows():
    cmd, ls, link = _run_case(DriveBehavior.LANE_CHANGE, foll_lane=2)
    assert ls is not None and ls.behavior == DriveBehavior.LANE_CHANGE, "선행 LANE_CHANGE 미수신"
    assert cmd.behavior == DriveBehavior.LANE_CHANGE, f"후행 LANE_CHANGE 아님: {cmd.behavior.name}"
    assert cmd.target_lane == 1, f"목표차로 반대(1) 아님: {cmd.target_lane}"


_TESTS = [
    ("1.선행CRUISE→후행CRUISE(+ALIVE)", test_leader_cruise_follower_cruise),
    ("2.선행STOP→후행STOP", test_leader_stop_follower_stop),
    ("3.선행LANE_CHANGE→후행추종(target=1)", test_leader_lane_change_follower_follows),
]


if __name__ == "__main__":
    print("[test_follower_pipeline] E2E (UDP loopback): 선행 V2V → 후행 판단")
    n_pass = 0
    for name, fn in _TESTS:
        try:
            fn()
            print(f"  PASS  {name}")
            n_pass += 1
        except AssertionError as e:
            print(f"  FAIL  {name}  -> {e!r}")
        except Exception as e:
            print(f"  ERROR {name}  -> {type(e).__name__}: {e}")
    print(f"\n{n_pass}/{len(_TESTS)} PASS")
    sys.exit(0 if n_pass == len(_TESTS) else 1)
