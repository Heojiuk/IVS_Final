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

from algorithm.perception import PerceptionModule
from algorithm import sensing


def main():
    ap = argparse.ArgumentParser(description="Perception debug viewer (integrated lane + object)")
    ap.add_argument("--hef", default="yolov11n.hef", help="HEF path (default: yolov11n.hef)")
    args = ap.parse_args()

    perception = PerceptionModule()      # update_*() 가 호출되며 _latest 가 채워짐 (버스 발행은 안 함)
    stop_event = threading.Event()

    print("[VIEW] perception debug viewer — ESC or Ctrl+C to quit")
    try:
        sensing.camera_loop(perception, stop_event, hef_path=args.hef, debug_view=True)
    except KeyboardInterrupt:
        stop_event.set()
    print("[VIEW] stopped.")


if __name__ == "__main__":
    main()
