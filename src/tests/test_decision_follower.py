"""후행차 판단 _decide_follower 검증 — 버스 없이 함수 직접 호출.

스펙(우선순위 STOP > LANE_CHANGE > SLOW > CRUISE):
  STOP   : scene=None / link LOST / 선행차 STOP / dist < FOLLOW_STOP_CM   (+hold)
  LANE_CHANGE: 선행차 behavior==LANE_CHANGE → 반대차로 토글, 도달 시 종료
  SLOW   : link STALE / FOLLOW_STOP_CM ≤ dist < FOLLOW_SLOW_CM / 차선미인식
  CRUISE : 그 외
  mode   : LOST→ESTOP/LINK_LOST, STALE→DEGRADED/LINK_LOST, 차선미인식→DEGRADED/LANE_LOST, else NORMAL

실행:  cd src && python tests/test_decision_follower.py
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from algorithm.decision import DecisionModule, FOLLOW_STOP_CM, FOLLOW_SLOW_CM  # noqa: E402
from messages import (Scene, V2VState, LinkStatus, DriveBehavior, Mode,         # noqa: E402
                      ModeCause, LinkState, Role)

ALIVE, STALE, LOST = LinkState.ALIVE, LinkState.STALE, LinkState.LOST


def _scene(lane_valid=True, current_lane=2, dist_cm=100.0):
    return Scene(lane_valid=lane_valid, current_lane=current_lane,
                 front_clear=True, dist_front_cm=dist_cm)


def _link(state):
    return LinkStatus(state=state, age_rx=10.0, last_seq=1)


def _peer(behavior=DriveBehavior.CRUISE, lane=2):
    return V2VState(role=Role.LEADER, seq=1, lane=lane, behavior=behavior)


def _fresh():
    return DecisionModule(Role.FOLLOWER)


# ── 1. 평소 추종 → CRUISE / NORMAL ──
def test_cruise_normal():
    cmd, mode = _fresh()._decide_follower(_scene(dist_cm=FOLLOW_SLOW_CM + 50),
                                          _link(ALIVE), _peer())
    assert cmd.behavior == DriveBehavior.CRUISE, cmd.behavior
    assert mode.mode == Mode.NORMAL, mode.mode


# ── 2. 선행차 STOP(정지선) → follower STOP ──
def test_leader_stop():
    cmd, _ = _fresh()._decide_follower(_scene(), _link(ALIVE),
                                       _peer(behavior=DriveBehavior.STOP))
    assert cmd.behavior == DriveBehavior.STOP, cmd.behavior


# ── 3. 초음파 추돌근접(<STOP) → STOP ──
def test_too_close_stop():
    cmd, _ = _fresh()._decide_follower(_scene(dist_cm=FOLLOW_STOP_CM - 1),
                                       _link(ALIVE), _peer())
    assert cmd.behavior == DriveBehavior.STOP, cmd.behavior


# ── 4. 거리 밴드(STOP~SLOW) → SLOW ──
def test_gap_slow():
    mid = (FOLLOW_STOP_CM + FOLLOW_SLOW_CM) / 2
    cmd, _ = _fresh()._decide_follower(_scene(dist_cm=mid), _link(ALIVE), _peer())
    assert cmd.behavior == DriveBehavior.SLOW, cmd.behavior


# ── 5. 통신 STALE → SLOW / DEGRADED(LINK_LOST) ──
def test_stale_slow_degraded():
    cmd, mode = _fresh()._decide_follower(_scene(dist_cm=100.0), _link(STALE), _peer())
    assert cmd.behavior == DriveBehavior.SLOW, cmd.behavior
    assert mode.mode == Mode.DEGRADED and mode.cause == ModeCause.LINK_LOST


# ── 6. 통신 LOST → STOP / ESTOP(LINK_LOST) ──
def test_lost_stop_estop():
    cmd, mode = _fresh()._decide_follower(_scene(dist_cm=100.0), _link(LOST), _peer())
    assert cmd.behavior == DriveBehavior.STOP, cmd.behavior
    assert mode.mode == Mode.ESTOP and mode.cause == ModeCause.LINK_LOST


# ── 7. 시동(전부 None) → STOP / ESTOP (안전 기본) ──
def test_startup_all_none():
    cmd, mode = _fresh()._decide_follower(None, None, None)
    assert cmd.behavior == DriveBehavior.STOP, cmd.behavior
    assert mode.mode == Mode.ESTOP, mode.mode


# ── 8. 선행차 LANE_CHANGE 추종 — 트리거→진행→도달 종료 ──
def test_lane_change_follow():
    d = _fresh()
    # (a) 트리거: 선행차 LC, 내 차로=2 → target=1
    cmd, _ = d._decide_follower(_scene(current_lane=2), _link(ALIVE),
                                _peer(behavior=DriveBehavior.LANE_CHANGE))
    assert cmd.behavior == DriveBehavior.LANE_CHANGE and cmd.target_lane == 1, \
        (cmd.behavior, cmd.target_lane)
    assert d._lane_target == 1
    # (b) 아직 차로 안 바뀜(=2) → 선행차가 CRUISE로 돌아가도 진행 유지
    cmd, _ = d._decide_follower(_scene(current_lane=2), _link(ALIVE), _peer())
    assert cmd.behavior == DriveBehavior.LANE_CHANGE, cmd.behavior
    # (c) 차로 도달(=1) → 종료, CRUISE 복귀
    cmd, _ = d._decide_follower(_scene(current_lane=1), _link(ALIVE), _peer())
    assert cmd.behavior == DriveBehavior.CRUISE and d._lane_target == 0, \
        (cmd.behavior, d._lane_target)


# ── 9. 우선순위 STOP > LANE_CHANGE (LC 중 선행차 STOP) ──
def test_priority_stop_over_lc():
    d = _fresh()
    d._decide_follower(_scene(current_lane=2), _link(ALIVE),
                       _peer(behavior=DriveBehavior.LANE_CHANGE))   # LC 진입
    cmd, _ = d._decide_follower(_scene(current_lane=2), _link(ALIVE),
                                _peer(behavior=DriveBehavior.STOP))
    assert cmd.behavior == DriveBehavior.STOP, cmd.behavior


# ── 10. 차선 미인식 → SLOW / DEGRADED(LANE_LOST) ──
def test_lane_lost_slow_degraded():
    cmd, mode = _fresh()._decide_follower(_scene(lane_valid=False, dist_cm=100.0),
                                          _link(ALIVE), _peer())
    assert cmd.behavior == DriveBehavior.SLOW, cmd.behavior
    assert mode.mode == Mode.DEGRADED and mode.cause == ModeCause.LANE_LOST


# ── 경계값 ─────────────────────────────────────────────────────────────────
def test_boundary_dist_eq_stop_is_slow():
    """dist == FOLLOW_STOP_CM → too_close는 strict(<)라 STOP 아님 → SLOW."""
    cmd, _ = _fresh()._decide_follower(_scene(dist_cm=FOLLOW_STOP_CM),
                                       _link(ALIVE), _peer())
    assert cmd.behavior == DriveBehavior.SLOW, cmd.behavior


def test_boundary_dist_eq_slow_is_cruise():
    """dist == FOLLOW_SLOW_CM → slow_gap은 strict(<)라 SLOW 아님 → CRUISE."""
    cmd, _ = _fresh()._decide_follower(_scene(dist_cm=FOLLOW_SLOW_CM),
                                       _link(ALIVE), _peer())
    assert cmd.behavior == DriveBehavior.CRUISE, cmd.behavior


def test_dist_none_alive_is_cruise():
    """초음파 None(범위밖=멀다) + link ALIVE → CRUISE."""
    cmd, _ = _fresh()._decide_follower(_scene(dist_cm=None), _link(ALIVE), _peer())
    assert cmd.behavior == DriveBehavior.CRUISE, cmd.behavior


def test_peer_none_but_link_alive_is_cruise():
    """peer=None(아직 패킷 디코드 전)인데 link ALIVE·scene 정상 → CRUISE (None 가드)."""
    cmd, _ = _fresh()._decide_follower(_scene(dist_cm=100.0), _link(ALIVE), None)
    assert cmd.behavior == DriveBehavior.CRUISE, cmd.behavior


def test_stop_hold_persists_on_link_recovery():
    """STOP hold(2s) 중 link가 ALIVE 회복+선행차 CRUISE여도 만료 전엔 STOP 유지(채터 방지).
    behavior=STOP 이지만 mode 는 현재 link 기준(ALIVE→NORMAL)."""
    d = _fresh()
    cmd, _ = d._decide_follower(_scene(dist_cm=100.0), _link(LOST), _peer())   # LOST→STOP, hold 세팅
    assert cmd.behavior == DriveBehavior.STOP, cmd.behavior
    cmd, mode = d._decide_follower(_scene(dist_cm=100.0), _link(ALIVE), _peer())  # 즉시 회복
    assert cmd.behavior == DriveBehavior.STOP, f"hold 중 STOP 미유지: {cmd.behavior.name}"
    assert mode.mode == Mode.NORMAL, f"회복 후 mode NORMAL 아님: {mode.mode.name}"


_TESTS = [
    ("1.평소 CRUISE/NORMAL", test_cruise_normal),
    ("2.선행차STOP→STOP", test_leader_stop),
    ("3.추돌근접→STOP", test_too_close_stop),
    ("4.거리밴드→SLOW", test_gap_slow),
    ("5.STALE→SLOW/DEGRADED", test_stale_slow_degraded),
    ("6.LOST→STOP/ESTOP", test_lost_stop_estop),
    ("7.시동(None)→STOP/ESTOP", test_startup_all_none),
    ("8.LANE_CHANGE 추종", test_lane_change_follow),
    ("9.우선순위 STOP>LC", test_priority_stop_over_lc),
    ("10.차선미인식→SLOW/DEGRADED", test_lane_lost_slow_degraded),
    ("11.경계 dist==STOP→SLOW", test_boundary_dist_eq_stop_is_slow),
    ("12.경계 dist==SLOW→CRUISE", test_boundary_dist_eq_slow_is_cruise),
    ("13.dist=None+ALIVE→CRUISE", test_dist_none_alive_is_cruise),
    ("14.peer=None+ALIVE→CRUISE", test_peer_none_but_link_alive_is_cruise),
    ("15.STOP hold 회복후 유지", test_stop_hold_persists_on_link_recovery),
]


if __name__ == "__main__":
    print(f"[test_decision_follower] FOLLOW_STOP_CM={FOLLOW_STOP_CM} FOLLOW_SLOW_CM={FOLLOW_SLOW_CM}")
    n_pass = 0
    for name, fn in _TESTS:
        try:
            fn()
            print(f"  PASS  {name}")
            n_pass += 1
        except AssertionError as e:
            print(f"  FAIL  {name}  -> {e!r}")
    print(f"\n{n_pass}/{len(_TESTS)} PASS")
    sys.exit(0 if n_pass == len(_TESTS) else 1)
