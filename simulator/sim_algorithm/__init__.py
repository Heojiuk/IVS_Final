"""시뮬레이터용 mock 알고리즘 — 실제 src/algorithm/ 의 대역(代役).

실제 환경(src/algorithm/)과 동일한 구조로 둔다:
    perception.py        ← SimPerception        (UI/시나리오 → Scene 발행)
    decision.py          ← LocalDecisionModule  (SCENE·LEADER_STATE·LINK → COMMAND·MODE)
    motion_planning.py   ← LocalMotionModule    (지연 재생 추종 PWM → EGO_STATE)

세 모듈 모두 src/algorithm 의 PerceptionModule·DecisionModule·MotionModule 과
동일한 `.step(bus)` 인터페이스 → VILSEngine 이 use_local_control 플래그 하나로
실제 src 모듈 ↔ 이 mock 을 그대로 교체할 수 있다. (공유 src 코드는 건드리지 않는다.)
"""
import _src_path; _src_path.add()  # core_module·messages 를 src 에서 import 가능하게 (패키지 진입 시 1회)
