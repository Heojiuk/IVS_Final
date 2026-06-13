"""진입점 — 조립(build) 후 스케줄러를 돌린다 (SDD 2장)

호출 구조:  main() → build() → Scheduler.run() ──50ms──> module.step(bus)
실행:       cd src && python main.py --role leader      (또는 --role follower)
로컬 2프로세스 통신 테스트:  IVS_PEER_IP=127.0.0.1 두 셸에서 leader/follower 각각 실행.
"""
import argparse
import signal

from core_module import config
from core_module.bus import MessageBus
from core_module.scheduler import Scheduler
from messages import Role
from algorithm.perception import PerceptionModule
from algorithm.decision import DecisionModule
from algorithm.motion_planning import MotionModule
from core_module.v2v import V2VModule


def build(role):
    """버스 1개 + 모듈 4개(인지·판단·주행·통신)를 조립해 (bus, modules, v2v)을 반환한다.  role='leader'|'follower'"""
    role = role.lower()
    role_id = Role.LEADER if role == "leader" else Role.FOLLOWER
    bus = MessageBus()
    v2v = V2VModule(role)
    modules = [
        PerceptionModule(),
        DecisionModule(role_id),
        MotionModule(role_id),
        v2v,
    ]
    return bus, modules, v2v


def main():
    """진입점 — --role 파싱·조립·RX 기동 후 스케줄러를 돌린다(Ctrl+C까지).  파라미터 없음 (CLI: --role leader|follower)"""
    ap = argparse.ArgumentParser(description="IVS V2V 군집주행 노드")
    ap.add_argument("--role", choices=["leader", "follower"], default="leader")
    args = ap.parse_args()

    bus, modules, v2v = build(args.role)
    sched = Scheduler(config.LOOP_PERIOD_S, modules, bus)
    v2v.start(bus)                            # V2V 수신 스레드 기동

    signal.signal(signal.SIGINT, lambda *_: sched.stop())
    signal.signal(signal.SIGTERM, lambda *_: sched.stop())

    print(f"[IVS] role={args.role} 20Hz 루프 시작 (Ctrl+C 종료)")
    try:
        sched.run()
    finally:
        v2v.stop()
        print(f"[IVS] 종료 — cycles={sched.cycles} overruns={sched.overruns}")


if __name__ == "__main__":
    main()
