"""core_module 검증 (follower 관점) — v2v 링크상태 전이 + scheduler dev 주입 가드 + 예외 격리.

기존 test_v2v_loopback 은 ALIVE 교환만 검증 → 여기선 follower 생명선인
link STALE/LOST 전이(미수신 경과시간 임계)와, scheduler의 dev 버스로거 자동주입 가드를 검증한다.

실행:  cd src && python tests/test_core_module_follower.py
"""
import os
import sys
import time
import tempfile
import shutil

os.environ["IVS_MODE"] = "dev"   # scheduler dev 주입 가드 검증을 위해 dev (V2VModule 생성 전 설정)
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from core_module import config                                  # noqa: E402
from core_module.bus import MessageBus, Topics                  # noqa: E402
from core_module.scheduler import Scheduler                     # noqa: E402
from core_module.v2v import V2VModule                           # noqa: E402
import core_module.bus_logger as bus_logger                     # noqa: E402
from messages import LinkState                                  # noqa: E402


# ── v2v 링크상태 전이 (follower 생명선) ─────────────────────────────────────
def test_link_status_transitions():
    """_link_status(): 미수신 경과(age)에 따라 ALIVE→STALE→LOST. 임계는 config 값."""
    node = V2VModule("follower")        # rx 5006 bind, RX 스레드는 start() 안 하면 안 돎
    try:
        # (a) 아직 한 번도 수신 안 함 → LOST
        node._last_rx = None
        assert node._link_status().state == LinkState.LOST, "초기(미수신) LOST 아님"

        # (b) 방금 수신 (age~0 < STALE) → ALIVE
        node._last_rx = time.monotonic()
        assert node._link_status().state == LinkState.ALIVE, "최근수신 ALIVE 아님"

        # (c) STALE 구간 (STALE_MS ≤ age < LOST_MS)
        mid = (config.LINK_STALE_MS + config.LINK_LOST_MS) / 2 / 1000.0
        node._last_rx = time.monotonic() - mid
        assert node._link_status().state == LinkState.STALE, "지연수신 STALE 아님"

        # (d) LOST 구간 (age ≥ LOST_MS)
        node._last_rx = time.monotonic() - (config.LINK_LOST_MS / 1000.0 + 0.1)
        assert node._link_status().state == LinkState.LOST, "오래미수신 LOST 아님"
    finally:
        node._rx.close()
        node._tx.close()


def test_link_status_boundaries():
    """임계 근방(±30ms) 전이 — STALE_MS·LOST_MS 경계가 의도대로 갈리는지."""
    node = V2VModule("follower")
    try:
        off = lambda ms: time.monotonic() - ms / 1000.0
        node._last_rx = off(config.LINK_STALE_MS - 30)   # 임계 직전 → ALIVE
        assert node._link_status().state == LinkState.ALIVE, "STALE 직전인데 ALIVE 아님"
        node._last_rx = off(config.LINK_STALE_MS + 30)   # 임계 직후 → STALE
        assert node._link_status().state == LinkState.STALE, "STALE 진입 실패"
        node._last_rx = off(config.LINK_LOST_MS - 30)    # LOST 직전 → STALE
        assert node._link_status().state == LinkState.STALE, "LOST 직전인데 STALE 아님"
        node._last_rx = off(config.LINK_LOST_MS + 30)    # LOST 직후 → LOST
        assert node._link_status().state == LinkState.LOST, "LOST 진입 실패"
    finally:
        node._rx.close()
        node._tx.close()


# ── scheduler dev 버스로거 자동주입 가드 ────────────────────────────────────
class _Dummy:
    """V2VModule 이 아닌 더미 모듈 (가드가 제외해야 함)."""
    def step(self, bus):
        pass


def _has_buslogger(sched):
    return any(type(m).__name__ == "BusLoggerModule" for m in sched.modules)


def test_scheduler_injects_buslogger_in_dev_with_v2v():
    """dev 모드 + production V2VModule → BusLoggerModule 자동 주입."""
    tmp = tempfile.mkdtemp()
    bus_logger.LOG_DIR = tmp                    # 실제 DevBus 오염 방지
    v2v = V2VModule("follower")
    try:
        sched = Scheduler(0.05, [v2v], MessageBus())
        assert _has_buslogger(sched), "dev+V2VModule인데 BusLoggerModule 미주입"
        for m in sched.modules:                 # 주입된 로거 파일 닫기
            if type(m).__name__ == "BusLoggerModule":
                m.close()
    finally:
        v2v._rx.close()
        v2v._tx.close()
        shutil.rmtree(tmp, ignore_errors=True)


def test_scheduler_no_inject_without_v2v():
    """dev 모드라도 V2VModule 이 없으면(더미만) 주입 안 함 (가드 = 정확한 타입)."""
    tmp = tempfile.mkdtemp()
    bus_logger.LOG_DIR = tmp
    try:
        sched = Scheduler(0.05, [_Dummy()], MessageBus())
        assert not _has_buslogger(sched), "V2VModule 없는데 BusLoggerModule 주입됨"
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_scheduler_no_inject_in_release():
    """release 모드면 V2VModule 있어도 주입 안 함."""
    os.environ["IVS_MODE"] = "release"
    tmp = tempfile.mkdtemp()
    bus_logger.LOG_DIR = tmp
    v2v = V2VModule("follower")
    try:
        sched = Scheduler(0.05, [v2v], MessageBus())
        assert not _has_buslogger(sched), "release인데 BusLoggerModule 주입됨"
    finally:
        v2v._rx.close()
        v2v._tx.close()
        shutil.rmtree(tmp, ignore_errors=True)
        os.environ["IVS_MODE"] = "dev"          # 원복


# ── scheduler 예외 격리 (한 모듈이 죽어도 루프 계속) ─────────────────────────
class _Raiser:
    def step(self, bus):
        raise RuntimeError("boom")


def test_scheduler_isolates_module_exception():
    """모듈 step() 예외가 격리되어 errors 증가 + 루프(cycles) 계속."""
    os.environ["IVS_MODE"] = "release"   # 이 테스트에선 로거 주입 불필요
    import threading
    sched = Scheduler(0.05, [_Raiser()], MessageBus())
    t = threading.Thread(target=sched.run, daemon=True)
    t.start()
    time.sleep(0.16)        # ~3 사이클
    sched.stop()
    t.join(timeout=1.0)
    os.environ["IVS_MODE"] = "dev"
    assert sched.errors >= 1, f"예외가 errors로 안 잡힘 (errors={sched.errors})"
    assert sched.cycles >= 1, f"예외로 루프가 멈춤 (cycles={sched.cycles})"


_TESTS = [
    ("1.link 전이 ALIVE/STALE/LOST", test_link_status_transitions),
    ("1b.link 임계 근방 전이", test_link_status_boundaries),
    ("2.dev+V2V→로거주입", test_scheduler_injects_buslogger_in_dev_with_v2v),
    ("3.dev+더미→주입안함", test_scheduler_no_inject_without_v2v),
    ("4.release→주입안함", test_scheduler_no_inject_in_release),
    ("5.예외격리(루프계속)", test_scheduler_isolates_module_exception),
]


if __name__ == "__main__":
    print(f"[test_core_module_follower] STALE_MS={config.LINK_STALE_MS} LOST_MS={config.LINK_LOST_MS}")
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
