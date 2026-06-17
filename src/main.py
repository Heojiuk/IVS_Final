"""진입점 — 조립(build) 후 스케줄러를 돌린다 (SDD 2장)

호출 구조:  main() → build() → Scheduler.run() ──50ms──> module.step(bus)
실행:       cd src && python main.py --role leader      (또는 --role follower)
IP 모드:    환경변수 IVS_MODE = release(기본,192.168.0.x) | dev(강의실 WiFi) | loopback(127.0.0.1, 단일PC 2프로세스)
"""
import argparse
import hashlib
import signal
import sys

from core_module import config
from core_module.bus import MessageBus
from core_module.scheduler import Scheduler
from messages import Role
from algorithm.perception import PerceptionModule
from algorithm.decision import DecisionModule
from algorithm.motion_planning import MotionModule
from core_module.v2v import V2VModule


def build(role, peer_ip=None):
    """버스 1개 + 모듈 4개(인지·판단·주행·통신)를 조립해 (bus, modules, v2v)을 반환한다.  role='leader'|'follower', peer_ip=상대 IP(--peer, 주면 _IPS 무시)"""
    role = role.lower()
    role_id = Role.LEADER if role == "leader" else Role.FOLLOWER
    bus = MessageBus()
    v2v = V2VModule(role, peer_ip)  # 소켓 bind·키 로드가 여기서 일어남 (실패 시 OSError/ValueError)
    modules = [
        PerceptionModule(),
        DecisionModule(role_id),
        MotionModule(role_id),
        v2v,
    ]
    return bus, modules, v2v


def _role(s):
    """역할 인자 정규화 — l/leader, f/follower (대소문자 무관) → 'leader'|'follower'."""
    m = {"l": "leader", "leader": "leader", "f": "follower", "follower": "follower"}
    key = s.strip().lower()
    if key not in m:
        raise argparse.ArgumentTypeError("role must be leader|l or follower|f")
    return m[key]


def main():
    """진입점 — -r/--role 파싱 → 기동 전 조립·소켓·키 검증(실패 시 깔끔히 중단) → 스케줄러 실행(Ctrl+C까지)."""
    ap = argparse.ArgumentParser(description="IVS V2V platooning node")
    ap.add_argument("-r", "-R", "--role", required=True, type=_role,
                    metavar="{leader|l, follower|f}",
                    help="차량 역할 — leader|l 또는 follower|f (대소문자 무관)")  # 기본값 없음 — 역할 누락 방지
    ap.add_argument("-p", "--peer", default=None, metavar="IP",
                    help="상대 차량 IP (주면 IVS_MODE/_IPS 무시 — DHCP 대응)")
    args = ap.parse_args()

    # ── 주행 시작 전 방어: 조립·소켓 bind·키 로드를 시도하고, 실패하면 raw 트레이스백 대신 명확히 중단 ──
    try:
        bus, modules, v2v = build(args.role, args.peer)
        cfg = config.for_role(args.role, args.peer)
        key_fp = hashlib.sha256(config.load_key()).hexdigest()[:8]
    except OSError as e:
        print(
            f"[IVS] STARTUP FAILED (socket/port: {e}). "
            "Another instance running, or duplicate --role on this Pi? Aborting.",
            file=sys.stderr,
        )
        sys.exit(1)
    except Exception as e:
        print(f"[IVS] STARTUP FAILED: {e}. Aborting.", file=sys.stderr)
        sys.exit(1)

    sched = Scheduler(config.LOOP_PERIOD_S, modules, bus)
    v2v.start(bus)  # V2V RX 스레드 기동

    signal.signal(signal.SIGINT, lambda *_: sched.stop())
    signal.signal(signal.SIGTERM, lambda *_: sched.stop())

    # 기동 배너 — 운영자가 주행 전 역할·모드·상대·키지문을 육안 확인 (양 Pi 키지문 동일해야 함)
    print(
        f"[IVS] role={args.role} mode={config.mode()} rx_port={cfg['rx_port']} "
        f"peer={cfg['peer_ip']}:{cfg['peer_port']} key={key_fp} "
        f"-> 20Hz loop start (Ctrl+C to stop)"
    )
    try:
        sched.run()
    finally:
        v2v.stop()
        print(
            f"[IVS] stopped — cycles={sched.cycles} overruns={sched.overruns} errors={sched.errors}"
        )


if __name__ == "__main__":
    main()
