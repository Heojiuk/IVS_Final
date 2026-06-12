"""주행 (STUB — 모션팀 담당). Maneuver 읽어 제어, EgoState 발행 + 액추에이터.

실제: 경로계획·Pure Pursuit(횡)·CACC(후행 종)·구동 듀티 → GPIO/PWM (gpiozero).
입력: maneuver(IF-B2)·scene(IF-B1)·leader_state(IF-B5, 후행 CACC)
출력: ego_state(IF-B4) + 서보/DC모터 GPIO
주의: GPIO는 라즈베리에서만. 더미는 하드웨어 미접근(노트북 실행 OK).
"""
import time

from bus import Topics
from contracts import EgoState


class MotionModule:
    def __init__(self, role):
        self.role = role          # 후행(FOLLOWER)만 CACC 거리 추종

    def step(self, bus):
        maneuver = bus.read(Topics.MANEUVER)           # IF-B2
        scene = bus.read(Topics.SCENE)                 # IF-B1
        leader = bus.read(Topics.LEADER_STATE)         # IF-B5
        # TODO(모션팀): 제어 산출 → throttle/steer 듀티, GPIO 출력
        bus.publish(Topics.EGO_STATE, EgoState(stamp=time.monotonic()))   # 출력 IF-B4
