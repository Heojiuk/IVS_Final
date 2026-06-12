"""인지 (STUB — 인지팀 담당). 센서 읽어 Scene 발행.

실제: 카메라(차선·객체 YOLO) + 초음파 → Scene. 지금은 흐름 확인용 더미값.
입력: 카메라/초음파(HW)   출력: bus[perception/scene]  (IF-B1)
"""
import time

from core_module.bus import Topics
from contracts import Scene

# 센서 핀 (ICD IF-H2 / HWD)
ULTRASONIC_TRIG, ULTRASONIC_ECHO = 23, 24   # 전방 초음파 (ECHO 5V→3.3V 분압)


class PerceptionModule:
    def step(self, bus):
        """매 50ms 호출 — 센서를 읽어 Scene을 perception/scene에 발행한다.  bus=메시지버스"""

        # ===== ★ 인지팀 여기 작업 =====================================
        # TODO: picamera2(차선) · Hailo YOLO(객체) · 초음파(거리) 를 읽어
        #       아래 Scene 의 필드를 실제 인식값으로 채우세요.
        scene = Scene(stamp=time.monotonic(), lane_valid=True, front_clear=True)   # ← 지금은 더미값
        # ==============================================================

        bus.publish(Topics.SCENE, scene)               # 출력 IF-B1 (토픽·형식 고정 — 건드리지 말 것)
