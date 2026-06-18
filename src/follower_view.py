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


def camera_loop_lane_only(perception, stop_event, debug_view=True):
    """차선 전용 카메라 루프 — sensing.camera_loop 에서 객체(main/YOLO) 부분만 뺀 형태.

    선행과 동일한 멀티스트림 설정(raw=풀 FOV)으로 BEV 캘리를 그대로 공유하되,
    main 스트림 추론(ObjectDetector) 없이 lores→차선만 처리한다.
    """
    import cv2
    from picamera2 import Picamera2
    from algorithm import lane_pipeline

    cam = Picamera2()
    cfg = cam.create_video_configuration(
        main={"size": sensing.MAIN_SIZE,  "format": "RGB888"},   # lores 의존성상 유지(캡처는 안 함)
        lores={"size": sensing.LORES_SIZE, "format": "YUV420"},
        raw={"size": sensing.RAW_SIZE},     # 선행과 동일 FOV — BEV 캘리(SRC_POINTS 등) 그대로 적용
        controls={"FrameRate": sensing.FRAME_RATE},
    )                                        # 카메라 정방향 장착 → 회전 없음 (선행과 동일)
    cam.configure(cfg)
    cam.start()

    try:
        frame_i = 0
        while not stop_event.is_set():
            # 차선: lores 스트림 (YUV → BGR) — 객체인식(main/YOLO)은 수행하지 않음
            yuv = cam.capture_array("lores")
            bgr = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_I420)

            # 인지는 매 프레임. 디버그 창은 N프레임마다 1회만 그림(저전력 — undervoltage 완화).
            if debug_view and frame_i % sensing.VIEW_RENDER_EVERY == 0:
                lane, bev_vis = lane_pipeline.process_view(bgr)
                perception.update_lane(*lane)
                # objects=[] → 박스 없음. Camera 창은 lores 프레임에 ego ROI/초음파만 표시
                sensing._show_debug(cv2, bgr, [], bev_vis,
                                    lane_pipeline._L.NEAR_ROI_Y0_FRAC,
                                    perception._latest["dist_front_m"])
                if (cv2.waitKey(1) & 0xFF) == 27:    # ESC
                    stop_event.set()
            else:
                perception.update_lane(*lane_pipeline.process(bgr))   # 화면만 스킵, 인지는 유지
            frame_i += 1
    finally:
        cam.stop()
        cam.close()
        if debug_view:
            cv2.destroyAllWindows()


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
        camera_loop_lane_only(perception, stop_event, debug_view=True)
    except KeyboardInterrupt:
        stop_event.set()
    print("[FOLLOWER] stopped.")


if __name__ == "__main__":
    main()
