"""스케줄러 — 50ms(20Hz)마다 모듈 step()을 순차 호출 (DD-INF-03)

호출 구조의 엔진. 모듈끼리는 직접 호출 안 하고 버스로만 주고받음(스케줄러는 순서만 보장).
드리프트 보정(next_t 누적)으로 평균 주기를 20Hz에 고정.
"""
import time

from core_module import config


class Scheduler:
    def __init__(self, period_s, modules, bus):
        """스케줄러 생성.  period_s=주기(초, 0.05=20Hz), modules=step() 호출 모듈 리스트(인지·판단·주행·통신 순), bus=공유 메시지버스"""
        self.period_s = period_s
        self.modules = modules
        self.bus = bus
        # dev 모드 + production 노드(main.py)일 때만 버스 로거를 마지막 모듈로 자동 삽입(디버깅).
        # 정확한 V2VModule 타입만 — run_scenario 의 RecordableV2VModule(서브클래스)은 제외하여
        # 통합테스트엔 영향 없음. 파일 열기 실패해도 주행은 계속(격리).
        if config.mode() == "dev":
            try:
                from core_module.v2v import V2VModule
                from core_module.bus_logger import BusLoggerModule
                if any(type(m) is V2VModule for m in modules):
                    role = next((m._role for m in modules if hasattr(m, "_role")), None)
                    self.modules.append(BusLoggerModule(role))
            except Exception as e:
                print(f"[scheduler] bus logger disabled: {e!r}")
        self._running = False
        self.cycles = 0          # 누적 사이클
        self.overruns = 0        # 주기 초과(데드라인 미스) 횟수
        self.errors = 0          # 모듈 step() 예외 누적 (격리되어 루프는 계속 돎)

    def run(self):
        """무한 루프 — 매 주기마다 모든 모듈 step(bus)을 순서대로 호출(드리프트 보정). stop() 전까지 반환 안 함.
        모듈 step() 예외는 격리(로깅·카운트 후 다음 모듈로) — 한 모듈 오류가 전체 루프·통신을 멈추지 않음.  파라미터 없음"""
        self._running = True
        next_t = time.monotonic()
        while self._running:
            for m in self.modules:            # 인지 → 판단 → 주행 → 통신
                try:
                    m.step(self.bus)
                except Exception as e:        # 한 모듈 예외가 전체 루프·통신을 죽이지 않게 격리
                    self.errors += 1
                    print(f"[scheduler] {type(m).__name__}.step() raised (skipped): {e!r}")
            self.cycles += 1
            next_t += self.period_s
            sleep = next_t - time.monotonic()
            if sleep > 0.0:
                time.sleep(sleep)
            else:
                self.overruns += 1            # 50ms 초과 — TODO: 로깅/연속초과 시 ESTOP
                next_t = time.monotonic()     # 드리프트 리셋

    def stop(self):
        """루프 종료 플래그를 세워 run()을 빠져나오게 한다 (Ctrl+C/종료 시).  파라미터 없음"""
        self._running = False
