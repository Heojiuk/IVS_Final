"""통합테스트 시나리오 — 타원 트랙 1바퀴 (선행차 LEADER 기준).

실제 src/algorithm/decision.py 의 _decide_leader 우선순위(STOP>LANE_CHANGE>SLOW>CRUISE)를
모두 자극하도록 인지(Scene) 타임라인을 구성한다. 인지는 노이즈 없는 clean 값을 50ms로 재생.

  STEPS:       ScenarioStep(t_ms, param, value) 리스트 (재생 시작 기준 경과 ms)
  INITIAL:     시작 시 인지 파라미터 (current_lane=2 에서 출발)
  DURATION_MS: 총 재생 길이
  CHECKPOINTS: 시각 구간별 기대값 (verify_scenario 가 측정값과 대조)

단위 주의: 인지 내부 param 은 m (lane_offset_m·dist_front_m) — SimPerception 이 ×100 해서 Scene(cm) 로 변환.
"""
from scenario import ScenarioStep   # simulator/scenario.py (경로는 run/verify 가 잡아줌)

# ── 출발 상태 — 2차로 직진, 차선 인식 양호, 전방 비어있음 ──────────────
INITIAL = {
    'lane_valid':         True,
    'current_lane':       2,
    'lane_offset_m':      0.0,
    'lane_heading_rad':   0.0,
    'lane_curvature_1pm': 0.0,
    'front_clear':        True,
    'dist_front_m':       None,
    'stop_signal':        False,
}

# ── 타임라인 (재생 시작 기준 ms) ───────────────────────────────────────
STEPS = [
    # 3.0~5.0s : 타원 커브 진입(곡률만 변화) — 판단은 평소(CRUISE) 유지해야 함
    ScenarioStep(3000, 'lane_curvature_1pm', 0.8),
    ScenarioStep(5000, 'lane_curvature_1pm', 0.0),
    # 6.0~7.0s : 차선 일시 미인식 → SLOW
    ScenarioStep(6000, 'lane_valid', False),
    ScenarioStep(7000, 'lane_valid', True),
    # 8.0s : 전방 장애물 출현(0.2s 펄스) → LANE_CHANGE 트리거 (반대차선=1, hold 1.5s 는 판단이 유지)
    ScenarioStep(8000, 'front_clear', False),
    ScenarioStep(8200, 'front_clear', True),
    # 10.5s : 차선변경 완료 — 인지가 변경 시작(8.0s) +2.5s 후 current_lane 2→1 반영
    ScenarioStep(10500, 'current_lane', 1),
    # 12.0s : 정지선(0.2s 펄스) → STOP 트리거 (hold 2.0s 는 판단이 유지)
    ScenarioStep(12000, 'stop_signal', True),
    ScenarioStep(12200, 'stop_signal', False),
]
DURATION_MS = 16000

# ── 검증 체크포인트 — 시각 구간(ms) 안에서 CSV 필드가 기대값이어야 함 ──
#    win 은 전이 가장자리를 피해 안쪽으로 잡음. 판단 hold: STOP=2.0s, LANE_CHANGE=1.5s.
CHECKPOINTS = [
    {'name': '시작 평소주행(CRUISE)',        'win': (200, 2800),    'field': 'behavior',     'expect': 'CRUISE'},
    {'name': '시작 차로=2',                  'win': (200, 2800),    'field': 'current_lane', 'expect': '2'},
    {'name': '커브 중 평소 유지(CRUISE)',    'win': (3200, 4800),   'field': 'behavior',     'expect': 'CRUISE'},
    {'name': '차선 미인식 → SLOW',           'win': (6100, 6900),   'field': 'behavior',     'expect': 'SLOW'},
    {'name': '차선 복귀 → CRUISE',           'win': (7300, 7900),   'field': 'behavior',     'expect': 'CRUISE'},
    {'name': '장애물 → LANE_CHANGE',         'win': (8150, 9400),   'field': 'behavior',     'expect': 'LANE_CHANGE'},
    {'name': 'LANE_CHANGE 목표차로=1',       'win': (8150, 9400),   'field': 'target_lane',  'expect': '1'},
    {'name': 'LANE_CHANGE 종료 후 CRUISE',   'win': (9700, 10300),  'field': 'behavior',     'expect': 'CRUISE'},
    {'name': '차선변경 완료(인지 +2.5s)=1',  'win': (10700, 11800), 'field': 'current_lane', 'expect': '1'},
    {'name': '정지선 → STOP(2s hold)',       'win': (12150, 13800), 'field': 'behavior',     'expect': 'STOP'},
    {'name': 'STOP 종료 후 CRUISE',          'win': (14300, 15800), 'field': 'behavior',     'expect': 'CRUISE'},
    # ── 모션 PWM (실제 MotionModule 출력: ego.throttle_pwm 0~1, steer_pwm ±1) ──
    {'name': 'CRUISE throttle=0.6',          'win': (200, 2800),    'field': 'throttle_pwm', 'expect': '0.6'},
    {'name': 'SLOW throttle=0.3',            'win': (6100, 6900),   'field': 'throttle_pwm', 'expect': '0.3'},
    {'name': 'LANE_CHANGE 좌조향 steer=-0.5', 'win': (8150, 9400),   'field': 'steer_pwm',    'expect': '-0.5'},
    {'name': 'STOP throttle=0.0',            'win': (12150, 13800), 'field': 'throttle_pwm', 'expect': '0.0'},
]
