"""인지 (STUB — 인지팀 담당). 센서 읽어 Scene 발행.

실제: 카메라(차선·객체 YOLO) + 초음파 → Scene. 지금은 흐름 확인용 더미값.
입력: 카메라/초음파(HW)   출력: bus[perception/scene]  (IF-B1)
"""
import time

from bus import Topics
from contracts import Scene

# 센서 핀 (ICD IF-H2 / HWD)
ULTRASONIC_TRIG, ULTRASONIC_ECHO = 23, 24   # 전방 초음파 (ECHO 5V→3.3V 분압)


class PerceptionModule:
    def step(self, bus):
        # TODO(인지팀): picamera2 / Hailo YOLO / 초음파 읽어 Scene 채우기
        scene = Scene(stamp=time.monotonic(), lane_valid=True, front_clear=True)
        bus.publish(Topics.SCENE, scene)               # 출력 IF-B1
