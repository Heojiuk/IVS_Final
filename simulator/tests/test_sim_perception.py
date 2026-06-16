import os, sys, time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import _src_path; _src_path.add()

from core_module.bus import MessageBus, Topics
from sim_perception import SimPerception

def test_step_publishes_scene():
    bus = MessageBus()
    sp = SimPerception()
    sp.params['lane_valid'] = True
    sp.params['current_lane'] = 2
    sp.params['lane_offset_m'] = 0.1
    sp.step(bus)
    scene = bus.read(Topics.SCENE)
    assert scene is not None
    assert scene.lane_valid is True
    assert scene.current_lane == 2
    assert abs(scene.lane_offset_m - 0.1) < 1e-9

def test_defaults_are_safe():
    bus = MessageBus()
    SimPerception().step(bus)
    scene = bus.read(Topics.SCENE)
    assert scene.lane_valid is False
    assert scene.front_clear is True
    assert scene.dist_front_m is None

if __name__ == '__main__':
    test_step_publishes_scene()
    test_defaults_are_safe()
    print('OK')
