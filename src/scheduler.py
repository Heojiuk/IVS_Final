"""스케줄러 — 50ms(20Hz)마다 모듈 step()을 순차 호출 (DD-INF-03)

호출 구조의 엔진. 모듈끼리는 직접 호출 안 하고 버스로만 주고받음(스케줄러는 순서만 보장).
드리프트 보정(next_t 누적)으로 평균 주기를 20Hz에 고정.
"""
import time


class Scheduler:
    def __init__(self, period_s, modules, bus):
        self.period_s = period_s
        self.modules = modules
        self.bus = bus
        self._running = False
        self.cycles = 0          # 누적 사이클
        self.overruns = 0        # 주기 초과(데드라인 미스) 횟수

    def run(self):
        self._running = True
        next_t = time.monotonic()
        while self._running:
            for m in self.modules:            # 인지 → 판단 → 주행 → 통신
                m.step(self.bus)
            self.cycles += 1
            next_t += self.period_s
            sleep = next_t - time.monotonic()
            if sleep > 0.0:
                time.sleep(sleep)
            else:
                self.overruns += 1            # 50ms 초과 — TODO: 로깅/연속초과 시 ESTOP
                next_t = time.monotonic()     # 드리프트 리셋

    def stop(self):
        self._running = False
