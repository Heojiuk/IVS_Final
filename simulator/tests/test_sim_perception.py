import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import _src_path; _src_path.add()

from core_module.bus import MessageBus, Topics
from sim_algorithm.perception import SimPerception

def test_step_publishes_scene():
    bus = MessageBus()
    sp = SimPerception()
    sp.params['lane_valid'] = True
    sp.params['current_lane'] = 2
    sp.params['lane_offset_m'] = 0.1   # 내부 m → Scene cm (×100)
    sp.step(bus)
    scene = bus.read(Topics.SCENE)
    assert scene is not None
    assert scene.lane_valid is True
    assert scene.current_lane == 2
    assert abs(scene.lane_offset_cm - 10.0) < 1e-9   # 0.1m = 10cm

def test_defaults_are_safe():
    bus = MessageBus()
    SimPerception().step(bus)
    scene = bus.read(Topics.SCENE)
    assert scene.lane_valid is False
    assert scene.front_clear is True
    assert scene.dist_front_cm is None

def test_dist_front_cm_coercion():
    bus = MessageBus()
    sp = SimPerception()
    sp.params['dist_front_m'] = 3.5   # 내부 m → Scene cm
    sp.step(bus)
    scene = bus.read(Topics.SCENE)
    assert isinstance(scene.dist_front_cm, float)
    assert abs(scene.dist_front_cm - 350.0) < 1e-9   # 3.5m = 350cm

if __name__ == '__main__':
    test_step_publishes_scene()
    test_defaults_are_safe()
    test_dist_front_cm_coercion()
    print('OK')
