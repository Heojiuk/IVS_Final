"""판단 (STUB — 판단팀 담당). Scene·V2V·링크 읽어 DriveCommand·Mode 발행.

실제: 모드 전이·행동 결정·추돌 가드·통신 폴백. 지금은 흐름 확인용 더미값.
입력: scene(IF-B1)·link_status(IF-B6)·leader_state(IF-B5)
출력: command(IF-B2, behavior+target_lane)·mode(IF-B3)
"""
import time

from bus import Topics
from contracts import DriveCommand, ModeCmd, DriveBehavior, Mode


class DecisionModule:
    def __init__(self, role):
        self.role = role          # 후행(FOLLOWER)만 leader_state로 추종 판단

    def step(self, bus):
        scene = bus.read(Topics.SCENE)                 # IF-B1
        link = bus.read(Topics.LINK_STATUS)            # IF-B6
        leader = bus.read(Topics.LEADER_STATE)         # IF-B5 (후행만 사용)
        # TODO(판단팀): 위 입력으로 행동/모드 결정 (None 가드 필수)
        bus.publish(Topics.COMMAND, DriveCommand(stamp=time.monotonic(), behavior=DriveBehavior.FOLLOW))
        bus.publish(Topics.MODE, ModeCmd(stamp=time.monotonic(), mode=Mode.NORMAL))
