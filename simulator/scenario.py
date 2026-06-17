"""시나리오 플레이어 — 재생 중 특정 시각에 파라미터를 자동으로 변경한다.

ScenarioStep: t_ms(재생 시작 기준 경과 ms), param(파라미터 키), value(변경값)
JSON 저장/불러오기 지원.
"""
import json
from dataclasses import dataclass, asdict
from typing import Any

# ── 조작 가능한 파라미터 목록 (키: 표시명)
PARAM_LABELS = {
    'lane_valid':         '차선유효(lane_valid)',
    'current_lane':       '현재차선(current_lane)',
    'lane_offset_m':      '측방오프셋(offset_m)',
    'lane_heading_rad':   '헤딩각(heading_rad)',
    'lane_curvature_1pm': '곡률(curvature_1pm)',
    'front_clear':        '전방비어있음(front_clear)',
    'dist_front_m':       '전방거리(dist_front_m)',
    'stop_signal':        '정지신호(stop_signal)',
    '__kv':               '속도배율(k_v)',
    '__kw':               '조향배율(k_w)',
}

PARAM_BOOL  = {'lane_valid', 'front_clear', 'stop_signal'}
PARAM_INT   = {'current_lane'}
PARAM_FLOAT = set(PARAM_LABELS) - PARAM_BOOL - PARAM_INT


@dataclass
class ScenarioStep:
    t_ms:  int    # 재생 시작 후 경과 ms
    param: str    # 파라미터 키
    value: Any    # 변경 값

    def label(self) -> str:
        pname = PARAM_LABELS.get(self.param, self.param)
        return f'{self.t_ms:>6}ms  {pname}  →  {self.value}'


def save_scenario(steps: list[ScenarioStep], path: str):
    data = [asdict(s) for s in steps]
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(data, f, ensure_ascii=False, indent=2)


def load_scenario(path: str) -> list[ScenarioStep]:
    with open(path, 'r', encoding='utf-8') as f:
        data = json.load(f)
    return [ScenarioStep(**d) for d in data]


def coerce_value(param: str, raw: str) -> Any:
    """문자열 입력값을 파라미터 타입에 맞게 변환."""
    raw = raw.strip()
    if param in PARAM_BOOL:
        return raw.lower() in ('1', 'true', 'yes', '참', '예')
    if param in PARAM_INT:
        return int(float(raw))
    return float(raw)
