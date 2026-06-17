"""인지 디버그 뷰어 — 통합 인지(차선+객체)를 그대로 돌리며 화면 2창을 띄운다.

실주행 노드(main.py)는 headless. 이건 통합 인지가 제대로 도는지 눈으로 확인하는 용도.
같은 camera_loop 코드를 쓰되, 스케줄러 없이 메인 스레드에서 돌려 imshow를 안정화한다.

실행:  cd src && python perception_view.py            (기본 hef: yolov11n.hef)
       cd src && python perception_view.py --hef path/to/yolov11n.hef
종료:  ESC (창에서) 또는 Ctrl+C

화면:
  Camera : main 프레임 + 객체 박스 + ego ROI(하단 밴드)
  BEV    : 마젠타 중앙선 + heading(rad+deg) + curvature
"""
import argparse
import threading
import time

from algorithm.perception import PerceptionModule
from algorithm import sensing


def _dist_printer(perception, stop_event):
    """초음파 거리를 0.5s마다 프린트. 센서 미연결이면 None 으로 표시."""
    while not stop_event.is_set():
        d = perception._latest["dist_front_m"]      # 미터 (없으면 None)
        print(f"[ultrasonic] dist = {'None' if d is None else f'{d * 100:.0f} cm'}",
              flush=True)                            # GUI 동작 중 버퍼링 방지
        time.sleep(0.5)


def main():
    ap = argparse.ArgumentParser(description="Perception debug viewer (integrated lane + object + ultrasonic)")
    ap.add_argument("--hef", default="yolov11n.hef", help="HEF path (default: yolov11n.hef)")
    args = ap.parse_args()

    perception = PerceptionModule()      # update_*() 가 호출되며 _latest 가 채워짐 (버스 발행은 안 함)
    stop_event = threading.Event()

    # 초음파 + 거리 프린트 (데몬). 카메라는 메인 스레드에서 (imshow 안정).
    threading.Thread(target=sensing.ultrasonic_loop,
                     args=(perception, stop_event), daemon=True).start()
    threading.Thread(target=_dist_printer,
                     args=(perception, stop_event), daemon=True).start()

    print("[VIEW] perception debug viewer — ESC or Ctrl+C to quit")
    try:
        sensing.camera_loop(perception, stop_event, hef_path=args.hef, debug_view=True)
    except KeyboardInterrupt:
        stop_event.set()
    print("[VIEW] stopped.")


if __name__ == "__main__":
    main()
