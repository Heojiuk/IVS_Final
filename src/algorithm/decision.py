"""판단 (STUB — 판단팀 담당). Scene·V2V·링크 읽어 DriveCommand·Mode 발행.

실제: 모드 전이·행동 결정·추돌 가드·통신 폴백. 지금은 흐름 확인용 더미값.
입력: scene(IF-B1)·link_status(IF-B6)·상대차 상태(IF-B5, 역할별 leader/follower_state)
출력: command(IF-B2, behavior+target_lane)·mode(IF-B3)
"""
import time

from core_module.bus import Topics
from messages import DriveCommand, ModeCmd, DriveBehavior, Mode, Role


class DecisionModule:
    def __init__(self, role):
        """판단 모듈 초기화.  role=자차 역할(Role) — 역할별 상대차 상태 선택(후행=선행추종 / 선행=후행모니터링)"""
        self.role = role          # 역할별 상대 상태 선택 (step에서 분기)

    def step(self, bus):
        """50ms 주기 — scene·링크·상대상태로 행동·모드 정해 command·mode를 bus에 전송.  bus=메시지버스"""
        scene = bus.read(Topics.SCENE)                 # 입력 IF-B1
        link = bus.read(Topics.LINK_STATUS)            # 입력 IF-B6
        # 역할별 상대차 상태 (같은 py, self.role 분기): 후행→선행추종 / 선행→후행모니터링
        peer = bus.read(Topics.LEADER_STATE if self.role == Role.FOLLOWER else Topics.FOLLOWER_STATE)  # 입력 IF-B5

        # ===== ★ 판단팀 여기 작업 =====================================
        # TODO: 위 scene·link·peer 로 행동(behavior)·모드(mode)를 결정하세요.
        #       (None 가드 필수 — 초기 사이클엔 입력이 None일 수 있음)
        command = DriveCommand(stamp=time.monotonic(), behavior=DriveBehavior.FOLLOW)   # ← 지금은 더미값
        mode = ModeCmd(stamp=time.monotonic(), mode=Mode.NORMAL)                        # ← 지금은 더미값
        # ==============================================================

        bus.publish(Topics.COMMAND, command)           # 출력 IF-B2 (토픽·형식 고정 — 건드리지 말 것)
        bus.publish(Topics.MODE, mode)                 # 출력 IF-B3
