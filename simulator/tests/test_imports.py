import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import _src_path; _src_path.add()

from core_module.bus import MessageBus, Topics
from core_module.v2v import packet_generator, packet_parser, PACKET_LEN, fmt_ms_of_day
from algorithm.decision import DecisionModule
from algorithm.motion_planning import MotionModule
from messages import EgoState, Scene, V2VState, Role, DriveBehavior

def test_imports_work():
    bus = MessageBus()
    assert bus.read(Topics.SCENE) is None

if __name__ == '__main__':
    test_imports_work()
    print('OK')
