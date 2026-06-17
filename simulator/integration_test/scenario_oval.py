"""통합테스트 시나리오 — 반시계 방향 원형 트랙 1바퀴 (선행차 LEADER 기준).

GUI 가상 트랙(2.5×2.05m, 원형, 반시계 방향) 실제 장애물 배치 기반.

  진행 순서(시계 방향 위치 → 반시계 이동):
    출발(6시, lane2) → 1차 장애물(9시) → 2차(12시) → 3차(2시) → 4차(4시) → 정지선(6시)

  차선:
    lane 1 = 내차선 (황선 안쪽, 트랙 내부)
    lane 2 = 외차선 (황선 바깥쪽, 출발 차선)

  LANE_CHANGE 동작:
    장애물 감지(front_clear=False) → 반대 차선으로 전환 (결정: decision._decide_leader)
    lane2→1: 좌조향 steer=-0.5 / lane1→2: 우조향 steer=+0.5 (motion._set_servo)
    물리 차선 변경 소요: ~2.5s → 인지가 current_lane 업데이트

  STEPS:       ScenarioStep(t_ms, param, value) 리스트 (재생 시작 기준 경과 ms)
  INITIAL:     시작 상태 (외차선=2, lane_valid, 전방 비어있음)
  DURATION_MS: 총 재생 길이
  CHECKPOINTS: 시각 구간(ms) 안에서 CSV 필드가 기대값이어야 함

단위 주의: 내부 param = m (lane_offset_m·dist_front_m) — SimPerception 이 ×100 해서 Scene(cm) 로 변환.
"""
from scenario import ScenarioStep   # simulator/scenario.py

# ── 출발 상태 — 외차선(2) 출발, 차선 인식 양호, 전방 비어있음 ──────────
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

# ── 타임라인 (반시계: 6시→9시→12시→2시→4시→6시 정지선) ──────────────────
#    장애물 pulse=0.2s / current_lane은 LANE_CHANGE 트리거 +2.5s 후 인지 반영
#    LANE_CHANGE_HOLD_S=1.5s → in_action 해제 후 최소 0.5s 여유 두고 next 트리거
STEPS = [
    # 5.0s : 1차 장애물(~9시, 좌측) — lane2 진입 중, front_clear=False 감지
    #         decision: lane2→target=1, LANE_CHANGE / motion: steer=-0.5(좌조향)
    ScenarioStep(5000, 'front_clear', False),
    ScenarioStep(5200, 'front_clear', True),
    ScenarioStep(7500, 'current_lane', 1),    # 5.0+2.5s : 물리 차선 변경 완료 인지

    # 10.0s : 2차 장애물(~12시, 상단) — lane1 주행 중, 장애물 재감지
    #          decision: lane1→target=2, LANE_CHANGE / motion: steer=+0.5(우조향)
    ScenarioStep(10000, 'front_clear', False),
    ScenarioStep(10200, 'front_clear', True),
    ScenarioStep(12500, 'current_lane', 2),   # 10.0+2.5s

    # 16.0s : 3차 장애물(~2시, 우측상단) — lane2 복귀 후 다시 감지
    #          decision: lane2→target=1, LANE_CHANGE / motion: steer=-0.5(좌조향)
    ScenarioStep(16000, 'front_clear', False),
    ScenarioStep(16200, 'front_clear', True),
    ScenarioStep(18500, 'current_lane', 1),   # 16.0+2.5s

    # 20.0s : 4차 장애물(~4시, 우측하단) — 3차와 근접 배치, lane1 주행 중
    #          decision: lane1→target=2, LANE_CHANGE / motion: steer=+0.5(우조향)
    ScenarioStep(20000, 'front_clear', False),
    ScenarioStep(20200, 'front_clear', True),
    ScenarioStep(22500, 'current_lane', 2),   # 20.0+2.5s

    # 25.0s : 정지선(~6시, 출발점 도착) — STOP 2.0s hold
    ScenarioStep(25000, 'stop_signal', True),
    ScenarioStep(25200, 'stop_signal', False),
]
DURATION_MS = 29000

# ── 검증 체크포인트 ── win 단위=ms, 전이 가장자리(±150ms) 제외 ──────────
#    판단 hold: STOP=2.0s, LANE_CHANGE=1.5s
#    current_lane: LANE_CHANGE 트리거 +2.5s 후 안정 (여유 100ms 추가)
CHECKPOINTS = [
    # ── 출발 평소주행 ────────────────────────────────────────────────
    {'name': '출발 CRUISE',              'win': (200,   4800),  'field': 'behavior',     'expect': 'CRUISE'},
    {'name': '출발 차로=2(외차선)',       'win': (200,   4800),  'field': 'current_lane', 'expect': '2'},

    # ── 1차 장애물(~9시) lane2→lane1, 좌조향 ──────────────────────
    {'name': '1차 장애물 LANE_CHANGE',   'win': (5150,  6400),  'field': 'behavior',     'expect': 'LANE_CHANGE'},
    {'name': '1차LC target=1(내차선)',   'win': (5150,  6400),  'field': 'target_lane',  'expect': '1'},
    {'name': '1차LC steer=-0.5(좌조향)','win': (5150,  6400),  'field': 'steer_pwm',    'expect': '-0.5'},
    {'name': '1차LC 후 CRUISE',          'win': (6700,  9800),  'field': 'behavior',     'expect': 'CRUISE'},
    {'name': '1차LC 완료 차로=1',        'win': (7700,  9800),  'field': 'current_lane', 'expect': '1'},

    # ── 2차 장애물(~12시) lane1→lane2, 우조향 ─────────────────────
    {'name': '2차 장애물 LANE_CHANGE',   'win': (10150, 11400), 'field': 'behavior',     'expect': 'LANE_CHANGE'},
    {'name': '2차LC target=2(외차선)',   'win': (10150, 11400), 'field': 'target_lane',  'expect': '2'},
    {'name': '2차LC steer=+0.5(우조향)','win': (10150, 11400), 'field': 'steer_pwm',    'expect': '0.5'},
    {'name': '2차LC 후 CRUISE',          'win': (11700, 15800), 'field': 'behavior',     'expect': 'CRUISE'},
    {'name': '2차LC 완료 차로=2',        'win': (12700, 15800), 'field': 'current_lane', 'expect': '2'},

    # ── 3차 장애물(~2시) lane2→lane1, 좌조향 ──────────────────────
    {'name': '3차 장애물 LANE_CHANGE',   'win': (16150, 17400), 'field': 'behavior',     'expect': 'LANE_CHANGE'},
    {'name': '3차LC target=1(내차선)',   'win': (16150, 17400), 'field': 'target_lane',  'expect': '1'},
    {'name': '3차LC steer=-0.5(좌조향)','win': (16150, 17400), 'field': 'steer_pwm',    'expect': '-0.5'},
    {'name': '3차LC 후 CRUISE',          'win': (17700, 19800), 'field': 'behavior',     'expect': 'CRUISE'},
    {'name': '3차LC 완료 차로=1',        'win': (18700, 19800), 'field': 'current_lane', 'expect': '1'},

    # ── 4차 장애물(~4시) lane1→lane2, 우조향 ─────────────────────
    {'name': '4차 장애물 LANE_CHANGE',   'win': (20150, 21400), 'field': 'behavior',     'expect': 'LANE_CHANGE'},
    {'name': '4차LC target=2(외차선)',   'win': (20150, 21400), 'field': 'target_lane',  'expect': '2'},
    {'name': '4차LC steer=+0.5(우조향)','win': (20150, 21400), 'field': 'steer_pwm',    'expect': '0.5'},
    {'name': '4차LC 후 CRUISE',          'win': (21700, 24800), 'field': 'behavior',     'expect': 'CRUISE'},
    {'name': '4차LC 완료 차로=2',        'win': (22700, 24800), 'field': 'current_lane', 'expect': '2'},

    # ── 정지선(~6시) ─────────────────────────────────────────────
    {'name': '정지선 → STOP(2s hold)',   'win': (25150, 26800), 'field': 'behavior',     'expect': 'STOP'},
    {'name': 'STOP 후 CRUISE',           'win': (27300, 28800), 'field': 'behavior',     'expect': 'CRUISE'},

    # ── 모션 PWM ─────────────────────────────────────────────────
    {'name': 'CRUISE throttle=0.6',     'win': (200,   4800),  'field': 'throttle_pwm', 'expect': '0.6'},
    {'name': 'STOP throttle=0.0',       'win': (25150, 26800), 'field': 'throttle_pwm', 'expect': '0.0'},
]
