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
SERVO_RIGHT_DEG, SERVO_LEFT_DEG = 30, 30
MOTOR_FORWARD, MOTOR_BACKWARD, MOTOR_ENABLE = 5, 6, 13


OFFSET_GAIN  = 1.0
HEADING_GAIN = 1.0


THROTTLE_NORMAL = 50
THROTTLE_STEER  = 50
THROTTLE_STOP   = 0


STEER_THRESHOLD   = 30 / 40
LANE_CHANGE_STEER = 0.9




class MotionModule:
    def __init__(self, role):
        self.role = role
        self._servo = None
        self._dc_pwm = None


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


                # speed control based on steer angle
                if abs(steer_pwm) >= STEER_THRESHOLD:
                    throttle_pwm =THROTTLE_NORMAL    
                else:
                    throttle_pwm = THROTTLE_STEER  


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








