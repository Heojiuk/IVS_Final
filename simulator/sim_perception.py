import time
import _src_path; _src_path.add()

from core_module.bus import Topics
from messages import Scene


class SimPerception:
    """UI-controlled fake perception. Publish to bus every 50ms step()."""

    def __init__(self):
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
        p = self.params
        scene = Scene(
            stamp=time.monotonic(),
            lane_valid=bool(p['lane_valid']),
            current_lane=int(p['current_lane']),
            lane_offset_m=float(p['lane_offset_m']),
            lane_heading_rad=float(p['lane_heading_rad']),
            lane_curvature_1pm=float(p['lane_curvature_1pm']),
            front_clear=bool(p['front_clear']),
            dist_front_m=p['dist_front_m'],
            stop_signal=bool(p['stop_signal']),
        )
        bus.publish(Topics.SCENE, scene)
