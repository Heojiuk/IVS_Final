"""진입점 — 조립(build) 후 스케줄러를 돌린다 (SDD 2장)

호출 구조:  main() → build() → Scheduler.run() ──50ms──> module.step(bus)
실행:       cd src && python main.py --role leader      (또는 --role follower)
로컬 2프로세스 통신 테스트:  IVS_PEER_IP=127.0.0.1 두 셸에서 leader/follower 각각 실행.
"""
import argparse
import signal

import config
from bus import MessageBus
from contracts import Role
from scheduler import Scheduler
from perception import PerceptionModule
from decision import DecisionModule
from motion import MotionModule
from comm import CommModule


def build(role):
    """버스 1개 + 모듈 4개 조립. 스케줄러는 modules 순서대로 step() 호출."""
    role = role.lower()
    role_id = Role.LEADER if role == "leader" else Role.FOLLOWER
    bus = MessageBus()
    comm = CommModule(role)
    modules = [
        PerceptionModule(),
        DecisionModule(role_id),
        MotionModule(role_id),
        comm,
    ]
    return bus, modules, comm


def main():
    ap = argparse.ArgumentParser(description="IVS V2V 군집주행 노드")
    ap.add_argument("--role", choices=["leader", "follower"], default="leader")
    args = ap.parse_args()

    bus, modules, comm = build(args.role)
    sched = Scheduler(config.LOOP_PERIOD_S, modules, bus)
    comm.start(bus)                           # V2V 수신 스레드 기동

    signal.signal(signal.SIGINT, lambda *_: sched.stop())
    signal.signal(signal.SIGTERM, lambda *_: sched.stop())

    print(f"[IVS] role={args.role} 20Hz 루프 시작 (Ctrl+C 종료)")
    try:
        sched.run()
    finally:
        comm.stop()
        print(f"[IVS] 종료 — cycles={sched.cycles} overruns={sched.overruns}")


if __name__ == "__main__":
    main()
