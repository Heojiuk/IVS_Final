import time

from core_module.bus import Topics
from messages import EgoState, DriveBehavior, Mode, ModeCause, Role

_GPIO_AVAILABLE = False
try:
    import RPi.GPIO as GPIO
    from gpiozero import AngularServo
    _GPIO_AVAILABLE = True
except ImportError:
    pass

SERVO_PIN                        = 12
SERVO_RIGHT_DEG, SERVO_LEFT_DEG = 50, 50
MOTOR_FORWARD, MOTOR_BACKWARD, MOTOR_ENABLE = 5, 6, 13

OFFSET_GAIN  = 0.7
HEADING_GAIN = 0.7

THROTTLE_NORMAL = 60   # 코너 (조향 클 때)
THROTTLE_STEER  = 40   # 직진 (조향 작을 때)
THROTTLE_STOP   = 0

# 코너 판정 히스테리시스 (steer 떨림에 의한 속도 채터링 방지) — 진입/이탈 두 임계
STEER_CORNER_ENTER = 0.45   # |steer| 이 이상 → 코너 진입
STEER_CORNER_EXIT  = 0.30   # |steer| 이 이하로 내려와야 → 직진 복귀

# 종방향 슬루레이트 — 한 사이클(50ms)당 throttle '증가' 최대량 (급가속 차단)
THROTTLE_SLEW = 5

LANE_CHANGE_STEER = 0.7


class MotionModule:
    def __init__(self, role):
        self.role = role
        self._servo = None
        self._dc_pwm = None
        self._cornering = False      # 코너 상태 래치 (히스테리시스)
        self._last_throttle = 0.0    # 직전 출력 throttle (슬루레이트)

        if _GPIO_AVAILABLE:
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(MOTOR_FORWARD,  GPIO.OUT)
            GPIO.setup(MOTOR_BACKWARD, GPIO.OUT)
            GPIO.setup(MOTOR_ENABLE,   GPIO.OUT)

            self._servo = AngularServo(
                SERVO_PIN,
                min_angle=0,
                max_angle=180,
                initial_angle=90,
            )
            self._dc_pwm = GPIO.PWM(MOTOR_ENABLE, 1000)
            self._dc_pwm.start(0)

    def step(self, bus):
        cmd    = bus.read(Topics.COMMAND)
        mode   = bus.read(Topics.MODE)
        scene  = bus.read(Topics.SCENE)
        leader = bus.read(Topics.LEADER_STATE)
        behavior    = cmd.behavior    if cmd is not None else DriveBehavior.CRUISE
        target_lane = cmd.target_lane if cmd is not None else 0

        throttle_pwm = THROTTLE_STOP
        steer_pwm    = 0.0

        if mode is not None and mode.mode == Mode.ESTOP:
            throttle_pwm = THROTTLE_STOP
            steer_pwm    = 0.0

        elif mode is not None and mode.mode == Mode.DEGRADED:
            throttle_pwm = THROTTLE_STEER
            if mode.cause == ModeCause.LANE_LOST:
                steer_pwm = 0.0
            elif mode.cause == ModeCause.OBSTACLE:
                throttle_pwm = THROTTLE_STOP
            else:
                steer_pwm = self._calc_steer(scene)

        else:
            if behavior == DriveBehavior.STOP:
                throttle_pwm = THROTTLE_STOP
                steer_pwm    = 0.0

            else:
                if behavior == DriveBehavior.LANE_CHANGE:
                    if scene is not None and scene.current_lane != target_lane:
                        if target_lane == 1:
                            steer_pwm = -LANE_CHANGE_STEER
                        elif target_lane == 2:
                            steer_pwm = LANE_CHANGE_STEER
                    else:
                        steer_pwm = self._calc_steer(scene)
                else:
                    steer_pwm = self._calc_steer(scene)

                # 조향각 기반 속도 — 히스테리시스로 채터링 방지
                throttle_pwm = self._corner_throttle(steer_pwm)

        # 슬루레이트 제한 후 GPIO 출력 (가속만 완만, 감속·정지는 즉시)
        throttle_pwm = self._slew(throttle_pwm)
        self._set_servo(steer_pwm)
        self._set_dc(throttle_pwm)

        ego = EgoState(
            stamp=time.monotonic(),
            throttle_pwm=throttle_pwm / 100.0,
            steer_pwm=steer_pwm,
            behavior=behavior,
        )

        bus.publish(Topics.EGO_STATE, ego)

    def _calc_steer(self, scene):
        if scene is None or not scene.lane_valid:
            return 0.0
        steer = OFFSET_GAIN * scene.lane_offset_cm + HEADING_GAIN * scene.lane_heading_rad
        return max(-1.0, min(1.0, steer))

    def _corner_throttle(self, steer_pwm):
        # 진입/이탈 두 임계로 코너 상태를 래치 → steer가 임계 근처서 떨려도 속도 안 깜빡임
        s = abs(steer_pwm)
        if self._cornering and s < STEER_CORNER_EXIT:
            self._cornering = False
        elif not self._cornering and s > STEER_CORNER_ENTER:
            self._cornering = True
        return THROTTLE_NORMAL if self._cornering else THROTTLE_STEER

    def _slew(self, target):
        # throttle '증가(가속)'만 한 사이클 THROTTLE_SLEW로 제한. 감속·정지·ESTOP은 즉시.
        if target > self._last_throttle:
            target = min(target, self._last_throttle + THROTTLE_SLEW)
        self._last_throttle = target
        return target

    def _set_servo(self, steer_pwm):
        if self._servo is None:
            return
        if steer_pwm > 0:
            angle = 90 + steer_pwm * SERVO_RIGHT_DEG
        elif steer_pwm < 0:
            angle = 90 + steer_pwm * SERVO_LEFT_DEG
        else:
            angle = 90
        self._servo.angle = max(0, min(180, angle))

    def _set_dc(self, throttle_pwm):
        if self._dc_pwm is None:
            return
        if throttle_pwm > 0:
            GPIO.output(MOTOR_FORWARD,  GPIO.HIGH)
            GPIO.output(MOTOR_BACKWARD, GPIO.LOW)
            self._dc_pwm.ChangeDutyCycle(throttle_pwm)
        elif throttle_pwm < 0:
            GPIO.output(MOTOR_FORWARD,  GPIO.LOW)
            GPIO.output(MOTOR_BACKWARD, GPIO.HIGH)
            self._dc_pwm.ChangeDutyCycle(abs(throttle_pwm))
        else:
            self._dc_pwm.ChangeDutyCycle(0)
            GPIO.output(MOTOR_FORWARD,  GPIO.LOW)
            GPIO.output(MOTOR_BACKWARD, GPIO.LOW)