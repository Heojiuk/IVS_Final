"""판단 (STUB — 판단팀 담당). Scene·V2V·링크 읽어 DriveCommand·Mode 발행.

실제: 모드 전이·행동 결정·추돌 가드·통신 폴백. 지금은 흐름 확인용 더미값.
입력: scene(IF-B1)·link_status(IF-B6)·상대차 상태(IF-B5, 역할별 leader/follower_state)
출력: command(IF-B2, behavior+target_lane)·mode(IF-B3)
"""

import time

from core_module.bus import Topics
from messages import DriveCommand, ModeCmd, DriveBehavior, Mode, Role

# ── 선행차 판단 임계값 ───────────────────────────────────────────────
STOP_HOLD_S = 2.0  # STOP 진입 후 최소 유지 시간(초) — 정지선 hold + 채터링 방지
STOP_SIGNAL_DEBOUNCE_N = 10  # stop_signal 연속 감지 사이클 수 (10×50ms=500ms) — 단발 노이즈로 인한 오정지 방지
# LANE_CHANGE는 위치 기반으로 종료 — scene.current_lane이 _lane_target에 도달하면 즉시 CRUISE 복귀


class DecisionModule:
    def __init__(self, role):
        """판단 모듈 초기화.  role=자차 역할(Role) — 역할별 상대차 상태 선택(후행=선행추종 / 선행=후행모니터링)"""
        self.role = role  # 역할별 상대 상태 선택 (step에서 분기)
        self._stop_until = 0.0  # 이 시각까지 STOP 유지 (정지 hold — _decide_leader)
        self._stop_signal_count = 0  # stop_signal 연속 감지 카운트 (디바운스)
        self._lane_target = 0  # LANE_CHANGE 목표 차로 (0=비활성, 1·2=활성 — current_lane이 이 값 도달하면 종료)

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
        우선순위(위가 이김): STOP > LANE_CHANGE > SLOW(차선 미인식) > CRUISE(평소)
        반환: (DriveCommand, ModeCmd)
        """
        now = time.monotonic()
        scene_valid = scene is not None

        # ① STOP 트리거 (정지선·첫사이클만 — 장애물은 STOP 아님!)
        # scene=None(시동·인지 결손)은 즉시 STOP, stop_signal은 연속 N사이클 감지 시에만 STOP (노이즈 방지)
        if not scene_valid:
            self._stop_until = now + STOP_HOLD_S
        elif scene.stop_signal:
            self._stop_signal_count += 1
            if self._stop_signal_count >= STOP_SIGNAL_DEBOUNCE_N:
                self._stop_until = now + STOP_HOLD_S
        else:
            self._stop_signal_count = 0  # stop_signal=False 들어오면 카운트 리셋

        # ② 장애물 감지 → LANE_CHANGE 트리거 (반대 차선으로 토글)
        # front_clear는 인지가 카메라+초음파 융합 판단한 결과이므로 이것만 보면 됨
        obstacle_in_lane = scene_valid and not scene.front_clear
        # 이미 STOP 중·LANE_CHANGE 중이면 중복 트리거 방지
        in_action = now < self._stop_until or self._lane_target != 0
        # current_lane은 인지가 항상 1 또는 2로 보장 → 별도 가드 불필요
        if obstacle_in_lane and not in_action:
            self._lane_target = 2 if scene.current_lane == 1 else 1  # 반대 차선

        # ②' LANE_CHANGE 완료 — 인지가 목표 차로 도달 보고하면 즉시 종료 (perception 신뢰)
        if self._lane_target != 0 and scene_valid and scene.current_lane == self._lane_target:
            self._lane_target = 0

        # ③ behavior 결정 (우선순위 사다리)
        if now < self._stop_until:
            behavior = DriveBehavior.STOP
            target_lane = 0  # STOP 중엔 target 의미 X
        elif self._lane_target != 0:
            behavior = DriveBehavior.LANE_CHANGE
            target_lane = self._lane_target  # 가야 할 차선
        elif scene_valid and not scene.lane_valid:
            behavior = DriveBehavior.SLOW  # 차선 못 찾음
            target_lane = 0
        else:
            behavior = DriveBehavior.CRUISE  # 평소
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
        command = DriveCommand(stamp=time.monotonic(), behavior=DriveBehavior.CRUISE)  # ← 지금은 더미값
        mode = ModeCmd(stamp=time.monotonic(), mode=Mode.NORMAL)  # ← 지금은 더미값
        return command, mode
