"""시뮬레이터 mock 인지 — src/algorithm/perception.py 의 대역.

UI/시나리오가 채운 파라미터(dict)를 매 50ms Scene 토픽으로 발행한다.
실제 인지(카메라·초음파)를 대신해 노이즈 없는 clean 한 Scene 을 버스에 올린다.
내부 단위는 m, Scene 계약은 cm(IF-B1) → 경계에서 ×100 변환.
"""
import threading
import time

from core_module.bus import Topics
from messages import Scene


class SimPerception:
    """UI-controlled fake perception. Publish to bus every 50ms step()."""

    def __init__(self):
        self._lock = threading.Lock()
        self.params = {
            'lane_valid': False,
            'current_lane': 0,
            'lane_offset_m': 0.0,
            'lane_heading_rad': 0.0,
            'lane_curvature_1pm': 0.0,
            'front_clear': True,
            'dist_front_m': None,
            'stop_signal': False,
        }

    def step(self, bus):
        with self._lock:
            p = dict(self.params)
        # 내부 param은 m, Scene 계약은 cm (IF-B1) → 경계에서 ×100 변환
        scene = Scene(
            stamp=time.monotonic(),
            lane_valid=bool(p['lane_valid']),
            current_lane=int(p['current_lane']),
            lane_offset_cm=float(p['lane_offset_m']) * 100.0,
            lane_heading_rad=float(p['lane_heading_rad']),
            lane_curvature_1pm=float(p['lane_curvature_1pm']),
            front_clear=bool(p['front_clear']),
            dist_front_cm=float(p['dist_front_m']) * 100.0 if p['dist_front_m'] is not None else None,
            stop_signal=bool(p['stop_signal']),
        )
        bus.publish(Topics.SCENE, scene)
