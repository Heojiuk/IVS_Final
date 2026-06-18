"""test_motion.py — 더미 신호 주입 후 MotionModule 단독 테스트
실행: cd src && python test_motion.py --role leader
"""
import time
import argparse
import sys
import tty
import termios
import select

from core_module.bus import MessageBus, Topics
from messages import (
    Scene, DriveCommand, ModeCmd,
    DriveBehavior, Mode, ModeCause, Role
)
from algorithm.motion_planning import MotionModule


def set_raw_mode(fd):
    old = termios.tcgetattr(fd)
    tty.setraw(fd)
    return old


def restore_mode(fd, old):
    termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--role", choices=["leader", "follower"], default="leader")
    args = ap.parse_args()
    role = Role.LEADER if args.role == "leader" else Role.FOLLOWER

    bus    = MessageBus()
    motion = MotionModule(role)

    # 초기 더미값
    behavior    = DriveBehavior.STOP
    mode_val    = Mode.NORMAL
    cause_val   = ModeCause.NONE
    offset      = 0.0
    heading     = 0.0
    dist        = None
    lane_valid  = True
    target_lane = 0

    print("=== Motion Module Test (50ms loop) ===")
    print("-- behavior --")
    print("w=FOLLOW  s=STOP  x=SLOW  l=LANE_CHANGE")
    print("-- mode --")
    print("n=NORMAL  e=ESTOP  g=DEGRADED")
    print("-- cause --")
    print("1=NONE  2=LINK_LOST  3=LANE_LOST  4=OBSTACLE")
    print("-- scene --")
    print("a=offset-  d=offset+  f=offset 0")
    print("j=heading- k=heading+ h=heading 0")
    print("o=dist 0.5m  p=dist None")
    print("v=lane_valid toggle")
    print("-- lane change --")
    print("b=target_lane 1  m=target_lane 2")
    print("-- q=quit --")
    print("======================================")

    fd  = sys.stdin.fileno()
    old = set_raw_mode(fd)

    try:
        while True:
            # 논블로킹 키 입력 확인
            if select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1)

                if key == 'q':
                    print("\nQuit.")
                    break

                # behavior
                elif key == 'w':
                    behavior = DriveBehavior.FOLLOW
                elif key == 's':
                    behavior = DriveBehavior.STOP
                elif key == 'x':
                    behavior = DriveBehavior.SLOW
                elif key == 'l':
                    behavior = DriveBehavior.LANE_CHANGE

                # mode
                elif key == 'n':
                    mode_val  = Mode.NORMAL
                    cause_val = ModeCause.NONE
                elif key == 'e':
                    mode_val  = Mode.ESTOP
                    cause_val = ModeCause.NONE
                elif key == 'g':
                    mode_val = Mode.DEGRADED

                # cause
                elif key == '1':
                    cause_val = ModeCause.NONE
                elif key == '2':
                    cause_val = ModeCause.LINK_LOST
                elif key == '3':
                    cause_val = ModeCause.LANE_LOST
                elif key == '4':
                    cause_val = ModeCause.OBSTACLE

                # scene offset
                elif key == 'a':
                    offset = max(-1.0, offset - 0.1)
                elif key == 'd':
                    offset = min(1.0, offset + 0.1)
                elif key == 'f':
                    offset = 0.0

                # scene heading
                elif key == 'j':
                    heading = max(-1.0, heading - 0.1)
                elif key == 'k':
                    heading = min(1.0, heading + 0.1)
                elif key == 'h':
                    heading = 0.0

                # dist_front_m
                elif key == 'o':
                    dist = 0.5
                elif key == 'p':
                    dist = None

                # lane_valid 토글
                elif key == 'v':
                    lane_valid = not lane_valid

                # target_lane
                elif key == 'b':
                    target_lane = 1
                elif key == 'm':
                    target_lane = 2

            # 50ms마다 버스에 더미값 주입 + step 실행
            bus.publish(Topics.SCENE, Scene(
                stamp=time.monotonic(),
                lane_valid=lane_valid,
                lane_offset_m=offset,
                lane_heading_rad=heading,
                dist_front_m=dist,
            ))
            bus.publish(Topics.COMMAND, DriveCommand(
                stamp=time.monotonic(),
                behavior=behavior,
                target_lane=target_lane,
            ))
            bus.publish(Topics.MODE, ModeCmd(
                stamp=time.monotonic(),
                mode=mode_val,
                cause=cause_val,
            ))

            motion.step(bus)

            ego = bus.read(Topics.EGO_STATE)
            if ego is not None:
                print(
                    f"behavior={behavior.name:<12} mode={mode_val.name:<9} "
                    f"cause={cause_val.name:<10} offset={offset:+.1f} "
                    f"heading={heading:+.1f} dist={str(dist):<6} "
                    f"lane_valid={str(lane_valid):<5} | "
                    f"throttle={ego.throttle_pwm:+.2f} steer={ego.steer_pwm:+.2f}",
                    end="\r"
                )

            time.sleep(0.05)  # 20Hz

    finally:
        restore_mode(fd, old)


if __name__ == "__main__":
    main()