"""후행차 인지 프로토타입 — 차선인식 전용 (객체인식/YOLO 없음, AI HAT 불필요).

선행차 코드(perception.py·sensing.py·main.py)는 한 줄도 건드리지 않는 독립 실행 프로토타입.
같은 lane_pipeline·초음파·디버그뷰를 '재사용만' 하되, ObjectDetector(Hailo)는 기동하지 않는다.
→ AI HAT 없는 후행 Pi에서 차선 offset/heading/curvature + 초음파를 눈으로 확인하는 용도.

문서 근거: SRS — Follower는 AI HAT 없이 '차선인식 + 예측추종 + 거리보정(초음파)'.
          차선중앙 유지는 전 차량 공통(선행과 동일 lane_pipeline 사용).

실행:  cd src && python follower_view.py
종료:  ESC (창에서) 또는 Ctrl+C

화면:
  Camera : lores 프레임 + ego ROI(하단 밴드) + 초음파 거리   (객체 박스 없음)
  BEV    : 마젠타 중앙선 + heading(rad+deg) + curvature
"""
import threading
import time

from algorithm.perception import PerceptionModule   # 데이터 보관용 (start() 호출 안 함 → YOLO 미기동)
from algorithm import sensing                        # 초음파 루프·디버그뷰·카메라 상수 재사용


def _dist_printer(perception, stop_event):
    """초음파 거리를 0.5s마다 프린트. 센서 미연결이면 None 으로 표시."""
    while not stop_event.is_set():
        d = perception._latest["dist_front_m"]
        print(f"[ultrasonic] dist = {'None' if d is None else f'{d * 100:.0f} cm'}",
              flush=True)
        time.sleep(0.5)


def main():
    perception = PerceptionModule()      # update_lane/update_distance 로 _latest 채움 (버스 발행 없음)
    stop_event = threading.Event()

    # 초음파 + 거리 프린트 (데몬). 카메라는 메인 스레드에서 (imshow 안정).
    threading.Thread(target=sensing.ultrasonic_loop,
                     args=(perception, stop_event), daemon=True).start()
    threading.Thread(target=_dist_printer,
                     args=(perception, stop_event), daemon=True).start()

    print("[FOLLOWER] lane-only perception prototype (no YOLO) — ESC or Ctrl+C to quit")
    try:
        sensing.lane_camera_loop(perception, stop_event, debug_view=True)   # main.py 후행과 동일 루프
    except KeyboardInterrupt:
        stop_event.set()
    print("[FOLLOWER] stopped.")


if __name__ == "__main__":
    main()
