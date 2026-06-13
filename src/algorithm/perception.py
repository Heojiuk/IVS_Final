"""인지 (STUB — 인지팀 담당). 센서 읽어 Scene 발행.

실제: 카메라(차선·객체 YOLO) + 초음파 → Scene. 지금은 흐름 확인용 더미값.
입력: 카메라/초음파(HW)   출력: bus[perception/scene]  (IF-B1)
"""
import time

from core_module.bus import Topics # 버스에 쓰이는 토픽
from contracts import Scene # 전방 객체 및 차선 정보 (IF-B1)

# 센서 핀 (ICD IF-H2 / HWD) 개발자가 수정가능.
ULTRASONIC_TRIG, ULTRASONIC_ECHO = 23, 24   # 전방 초음파 (ECHO 5V→3.3V 분압)


class PerceptionModule:
    def step(self, bus):
        """50ms 주기 — 카메라·초음파로 전방 인지해 scene에 저장하여 bus에 전송.  bus=메시지버스"""

        # ===== ★ 인지팀 여기 작업 =====================================
        # TODO: picamera2(차선) · Hailo YOLO(객체) · 초음파(거리) 를 읽어
        #       아래 Scene 의 필드를 실제 인식값으로 채우세요.
        scene = Scene(stamp=time.monotonic(), lane_valid=True, front_clear=True)   # ← 지금은 더미값
        # ==============================================================

        bus.publish(Topics.SCENE, scene)               # 출력 IF-B1 (토픽·형식 고정 — 건드리지 말 것)
