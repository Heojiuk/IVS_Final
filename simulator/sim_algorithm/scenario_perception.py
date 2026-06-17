"""시나리오 구동 mock 인지 — sim_algorithm/perception.py(SimPerception)의 타임라인 재생 버전.

UI 대신 ScenarioStep 타임라인을 50ms마다 재생해, 시나리오대로의 clean Scene 을 발행한다.
통합테스트에서 실제 판단·모션·통신에 노이즈 없는 인지 입력을 시각순으로 먹이는 용도.

  - .step(bus) 인터페이스는 SimPerception 과 동일 → Scheduler/VILSEngine 에 그대로 끼운다.
  - step 은 ScenarioStep.{t_ms,param,value} 만 duck-typing 으로 읽으므로 scenario.py 의존 없음.
  - 재생 기준점(t0)은 첫 step() 호출 시각 → scn 경과(ms)는 elapsed_ms() 로 노출(기록·검증용).
"""
import time

from sim_algorithm.perception import SimPerception


class ScenarioPerception:
    """ScenarioStep 타임라인을 재생하는 인지 모사.

    steps:   [ScenarioStep(t_ms, param, value), ...]  — t_ms = 재생 시작 후 경과 ms
    initial: 시작 시 SimPerception.params 에 덮어쓸 초기값 dict (없으면 SimPerception 기본 = 안전값)
    """

    def __init__(self, steps, initial=None):
        self._sp = SimPerception()
        if initial:
            self._sp.params.update(initial)
        self._steps = sorted(steps, key=lambda s: s.t_ms)
        self._i = 0          # 다음 적용할 step 인덱스
        self._t0 = None      # 첫 step() 호출 monotonic = 재생 기준점

    def elapsed_ms(self):
        """재생 시작(첫 step) 후 경과 ms. 아직 시작 전이면 0.  파라미터 없음"""
        return 0.0 if self._t0 is None else (time.monotonic() - self._t0) * 1000.0

    @property
    def done(self):
        """모든 ScenarioStep 을 적용했으면 True (타임라인 종료).  파라미터 없음"""
        return self._i >= len(self._steps)

    def step(self, bus):
        """50ms 주기 — 경과시각에 도달한 step 들을 params 에 반영한 뒤 Scene 을 발행.  bus=메시지버스"""
        if self._t0 is None:
            self._t0 = time.monotonic()
        el = self.elapsed_ms()
        # 경과시각에 도달한 step 을 모두 적용 (같은 t_ms 의 여러 step 동시 반영)
        while self._i < len(self._steps) and self._steps[self._i].t_ms <= el:
            s = self._steps[self._i]
            self._sp.params[s.param] = s.value
            self._i += 1
        self._sp.step(bus)   # params(m) → Scene(cm) 발행
