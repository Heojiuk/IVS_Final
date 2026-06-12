"""주행 (STUB — 모션팀 담당). DriveCommand 읽어 제어, EgoState 발행 + 액추에이터.

제어 2원칙:
  종방향 = throttle_pwm(PWM 듀티) → DC 모터드라이버 → DC 모터 → 후륜 (개루프; 후행만 초음파 거리 보정)
  횡방향 = steer_pwm(PWM) → 서보 1개 → 좌우 조향 (카메라 차선 루프; 조향각 센서 없음)
입력: command(IF-B2)·scene(IF-B1)·leader_state(IF-B5)
출력: ego_state(IF-B4, throttle_pwm·steer_pwm) + 서보/DC모터 GPIO
주의: GPIO는 라즈베리에서만. 더미는 하드웨어 미접근(노트북 실행 OK).
"""
import time

from bus import Topics
from contracts import EgoState, DriveBehavior

# 액추에이터 (gpiozero PWM, 피드백 없음. 핀: ICD IF-H3/H4·HWD 기준)
#   서보(조향): PWMOutputDevice(SERVO_PIN, frequency=50), 펄스폭 1.0~2.0ms·중립 1.5ms (ICD IF-H3)
#   DC모터(구동): Motor(forward=IN1, backward=IN2, enable=PWM, pwm=True), forward/backward(speed 0~1)
SERVO_PIN, SERVO_FREQ = 12, 50                          # IF-H3 서보 PWM (HW PWM0)
SERVO_LEFT, SERVO_CENTER, SERVO_RIGHT = 0.05, 0.075, 0.10   # 1.0ms / 1.5ms 중립 / 2.0ms @50Hz (실측 보정 TBD)
MOTOR_FORWARD, MOTOR_BACKWARD, MOTOR_ENABLE = 5, 6, 13  # IF-H4 IN1=5·IN2=6·EN/PWM=13 (HW PWM1)


class MotionModule:
    def __init__(self, role):
        self.role = role          # 후행(FOLLOWER)만 초음파 거리 보정

    def step(self, bus):
        cmd = bus.read(Topics.COMMAND)                 # IF-B2 (behavior)
        scene = bus.read(Topics.SCENE)                 # IF-B1
        leader = bus.read(Topics.LEADER_STATE)         # IF-B5
        behavior = cmd.behavior if cmd is not None else DriveBehavior.FOLLOW
        # TODO(모션팀): 행동대로 throttle_pwm·steer_pwm 산출 → GPIO 출력
        bus.publish(Topics.EGO_STATE,
                    EgoState(stamp=time.monotonic(), behavior=behavior))   # 출력 IF-B4
