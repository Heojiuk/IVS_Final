"""판단 (STUB — 판단팀 담당). Scene·V2V·링크 읽어 DriveCommand·Mode 발행.

실제: 모드 전이·행동 결정·추돌 가드·통신 폴백. 지금은 흐름 확인용 더미값.
입력: scene(IF-B1)·link_status(IF-B6)·leader_state(IF-B5)
출력: command(IF-B2, behavior+target_lane)·mode(IF-B3)
"""
import time

from core_module.bus import Topics
from contracts import DriveCommand, ModeCmd, DriveBehavior, Mode


class DecisionModule:
    def __init__(self, role):
        """판단 모듈 초기화.  role=자차 역할(Role) — 후행(FOLLOWER)만 leader_state로 추종 판단"""
        self.role = role          # 후행(FOLLOWER)만 leader_state로 추종 판단

    def step(self, bus):
        """매 50ms 호출 — scene·링크·상대상태로 행동/모드를 정해 command·mode를 발행한다.  bus=메시지버스"""
        scene = bus.read(Topics.SCENE)                 # 입력 IF-B1
        link = bus.read(Topics.LINK_STATUS)            # 입력 IF-B6
        leader = bus.read(Topics.LEADER_STATE)         # 입력 IF-B5 (후행만 사용)

        # ===== ★ 판단팀 여기 작업 =====================================
        # TODO: 위 scene·link·leader 로 행동(behavior)·모드(mode)를 결정하세요.
        #       (None 가드 필수 — 초기 사이클엔 입력이 None일 수 있음)
        command = DriveCommand(stamp=time.monotonic(), behavior=DriveBehavior.FOLLOW)   # ← 지금은 더미값
        mode = ModeCmd(stamp=time.monotonic(), mode=Mode.NORMAL)                        # ← 지금은 더미값
        # ==============================================================

        bus.publish(Topics.COMMAND, command)           # 출력 IF-B2 (토픽·형식 고정 — 건드리지 말 것)
        bus.publish(Topics.MODE, mode)                 # 출력 IF-B3
