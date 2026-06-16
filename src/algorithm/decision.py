"""판단 (STUB — 판단팀 담당). Scene·V2V·링크 읽어 DriveCommand·Mode 발행.

실제: 모드 전이·행동 결정·추돌 가드·통신 폴백. 지금은 흐름 확인용 더미값.
입력: scene(IF-B1)·link_status(IF-B6)·상대차 상태(IF-B5, 역할별 leader/follower_state)
출력: command(IF-B2, behavior+target_lane)·mode(IF-B3)
"""

import time

from core_module.bus import Topics
from messages import DriveCommand, ModeCmd, DriveBehavior, Mode, Role

# ── 선행차 판단 임계값 ───────────────────────────────────────────────
SAFE_FRONT_DIST_M = 0.5  # 앞 장애물 감지 거리(m) — 이 이내면 LANE_CHANGE 트리거 (RC카 실측 후 튜닝 TODO)
STOP_HOLD_S = 2.0  # STOP 진입 후 최소 유지 시간(초) — 정지선 hold + 채터링 방지
LANE_CHANGE_HOLD_S = 1.5  # LANE_CHANGE 동작 유지 시간(초) — 끝나면 차선 변경 완료로 간주


class DecisionModule:
    def __init__(self, role):
        """판단 모듈 초기화.  role=자차 역할(Role) — 역할별 상대차 상태 선택(후행=선행추종 / 선행=후행모니터링)"""
        self.role = role  # 역할별 상대 상태 선택 (step에서 분기)
        self._stop_until = 0.0  # 이 시각까지 STOP 유지 (정지 hold — _decide_leader)
        self._lane_target = 0  # LANE_CHANGE 목표 차로 (트리거 시 설정, 1·2·0=미설정)
        self._lane_change_until = 0.0  # 이 시각까지 LANE_CHANGE 동작 유지

    def step(self, bus):
        """50ms 주기 — scene·링크·상대상태로 행동·모드 정해 command·mode를 bus에 전송.  bus=메시지버스"""
        scene = bus.read(Topics.SCENE)  # 입력 IF-B1
        link = bus.read(Topics.LINK_STATUS)  # 입력 IF-B6
        # 역할별 상대차 상태 (같은 py, self.role 분기): 후행→선행추종 / 선행→후행모니터링
        peer = bus.read(Topics.LEADER_STATE if self.role == Role.FOLLOWER else Topics.FOLLOWER_STATE)  # 입력 IF-B5

        # ===== ★ 판단팀 여기 작업 =====================================
        # TODO: 위 scene·link·peer 로 행동(behavior)·모드(mode)를 결정하세요.
        #       (None 가드 필수 — 초기 사이클엔 입력이 None일 수 있음)
        # 역할별 분기 — 각 라즈베리파이는 둘 중 한 함수만 실제로 돌게 된다.
        if self.role == Role.LEADER:
            command, mode = self._decide_leader(scene, link, peer)
        else:  # Role.FOLLOWER
            command, mode = self._decide_follower(scene, link, peer)
        # ==============================================================

        bus.publish(Topics.COMMAND, command)  # 출력 IF-B2 (토픽·형식 고정 — 건드리지 말 것)
        bus.publish(Topics.MODE, mode)  # 출력 IF-B3

    # ── 역할별 판단 로직 (각 라즈베리파이는 자기 역할 함수만 사용) ──────
    def _decide_leader(self, scene, link, peer):
        """선행차 판단 — 정지선·첫사이클은 STOP, 앞에 차/장애물 보이면 반대 차선으로 LANE_CHANGE.
        우선순위(위가 이김): STOP > LANE_CHANGE > SLOW(차선 미인식) > FOLLOW(평소)
        반환: (DriveCommand, ModeCmd)
        """
        now = time.monotonic()
        scene_valid = scene is not None

        # ① STOP 트리거 (정지선·첫사이클만 — 장애물은 STOP 아님!)
        if not scene_valid or scene.stop_signal:
            self._stop_until = now + STOP_HOLD_S

        # ② 장애물 감지 → LANE_CHANGE 트리거 (반대 차선으로 토글)
        obstacle_in_lane = scene_valid and (
            not scene.front_clear or (scene.dist_front_m is not None and scene.dist_front_m < SAFE_FRONT_DIST_M)
        )
        # 이미 STOP 중·LANE_CHANGE 중이면 중복 트리거 방지
        in_action = now < self._stop_until or now < self._lane_change_until
        # current_lane 미확정(0)이면 안전상 변경 안 함 — 내가 어느 차로인지 모르면 못 바꿈
        if obstacle_in_lane and not in_action and scene.current_lane in (1, 2):
            self._lane_target = 2 if scene.current_lane == 1 else 1  # 반대 차선
            self._lane_change_until = now + LANE_CHANGE_HOLD_S

        # ③ behavior 결정 (우선순위 사다리)
        if now < self._stop_until:
            behavior = DriveBehavior.STOP
            target_lane = 0  # STOP 중엔 target 의미 X
        elif now < self._lane_change_until:
            behavior = DriveBehavior.LANE_CHANGE
            target_lane = self._lane_target  # 가야 할 차선
        elif scene_valid and not scene.lane_valid:
            behavior = DriveBehavior.SLOW  # 차선 못 찾음
            target_lane = 0
        else:
            behavior = DriveBehavior.FOLLOW  # 평소
            target_lane = 0

        command = DriveCommand(stamp=now, behavior=behavior, target_lane=target_lane)
        # ModeCmd 는 토픽 호환 유지용 placeholder (NORMAL/NONE 기본값).
        # 아키텍트 합의: 현재 behavior 만으로 커버 가능 → 추후 팀 전체 합의 후 토픽째 제거 예정.
        mode = ModeCmd(stamp=now)
        return command, mode

    def _decide_follower(self, scene, link, peer):
        """후행차 판단 — 선행 추종(peer.throttle_pwm·steer_pwm 참고) + 차선·거리 보정.
        반환: (DriveCommand, ModeCmd)"""
        # ── 후행차 ────────────────────────────────────────────────
        # peer = 선행 상태(LEADER_STATE) — link.state 확인 후 추종에 반영
        # TODO(follower-mode):     scene·link 로 mode 결정 (link LOST → DEGRADED 등)
        # TODO(follower-behavior): peer.throttle_pwm·steer_pwm·behavior 참고 + 차선 보정
        command = DriveCommand(stamp=time.monotonic(), behavior=DriveBehavior.FOLLOW)  # ← 지금은 더미값
        mode = ModeCmd(stamp=time.monotonic(), mode=Mode.NORMAL)  # ← 지금은 더미값
        return command, mode
