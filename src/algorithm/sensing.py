"""인지 하드웨어 계층 (라즈베리파이 전용).

한 카메라(멀티스트림) + 초음파를 백그라운드 스레드로 읽어 PerceptionModule의
update_*() 로 최신값을 갱신한다. PerceptionModule.step() 은 그 최신값을 50ms마다
Scene 으로 발행(이미 구현). 하드웨어 라이브러리는 지연 import (dev PC에서도 로드 가능).

  camera_loop     : Picamera2 main(RGB, YOLO) + lores(YUV, 차선) → update_objects/update_lane
  ultrasonic_loop : gpiozero DistanceSensor → update_distance
  ObjectDetector  : detect.py(단일 소스) 재사용 → List[Detection]
"""
import threading
import time

from messages import Detection

# ── 카메라 스트림 설정 ────────────────────────────────────────────────
MAIN_SIZE  = (1920, 1080)   # YOLO 용 (detect.py CAMERA_W/H 와 일치)
LORES_SIZE = (640, 360)     # 차선 용 (v2.3 PREVIEW_SIZE 와 일치)
RAW_SIZE   = (2304, 1296)   # ★ imx708 풀 FOV 비닝 모드 — 차선 BEV 캘리가 이 화각 기준.
                            #   raw 미지정 시 picamera2가 다른 센서모드(다른 FOV) 선택 → BEV 깨짐
FRAME_RATE = 30

# ── YOLO 모델 (고정 경로 — 명령어 인자 없이 사용) ─────────────────────
HEF_PATH = "/home/jhoh/yolov11n.hef"   # Pi마다 위치 다르면 이 한 줄만 수정

# ── 초음파 ────────────────────────────────────────────────────────────
ULTRA_ECHO, ULTRA_TRIG = 8, 11      # BCM (벤치 테스트와 동일)
ULTRA_MAX_DIST_M       = 4.0
ULTRA_PERIOD_S         = 0.05       # 20Hz


class ObjectDetector:
    """Hailo YOLO 추론 래퍼. detect.py 의 전·후처리를 재사용해 Detection 리스트 반환.

    수명주기: with ObjectDetector(hef) as det:  → det.infer_detections(frame_rgb)
    (InferVStreams/activate 컨텍스트를 진입/종료에서 관리)
    """

    def __init__(self, hef_path=HEF_PATH):
        from algorithm import object_detection as detect  # 객체 인식 단일 소스
        from hailo_platform import (
            HEF, VDevice, HailoStreamInterface, InferVStreams,
            ConfigureParams, InputVStreamParams, OutputVStreamParams, FormatType,
        )
        self._detect = detect
        self._InferVStreams = InferVStreams

        self.hef    = HEF(hef_path)
        self.target = VDevice()
        ngs = self.target.configure(
            self.hef, ConfigureParams.create_from_hef(
                self.hef, interface=HailoStreamInterface.PCIe))
        self.ng        = ngs[0]
        self.ng_params = self.ng.create_params()
        self.in_params  = InputVStreamParams.make(self.ng, format_type=FormatType.UINT8)
        self.out_params = OutputVStreamParams.make(self.ng, format_type=FormatType.FLOAT32)
        self.input_name = self.hef.get_input_vstream_infos()[0].name

        self._pipe_cm = None
        self._act_cm  = None
        self._pipe    = None

    def __enter__(self):
        self._pipe_cm = self._InferVStreams(self.ng, self.in_params, self.out_params)
        self._pipe    = self._pipe_cm.__enter__()
        self._act_cm  = self.ng.activate(self.ng_params)
        self._act_cm.__enter__()
        return self

    def __exit__(self, *exc):
        if self._act_cm:
            self._act_cm.__exit__(*exc)
        if self._pipe_cm:
            self._pipe_cm.__exit__(*exc)

    def infer_detections(self, frame_rgb):
        """프레임(RGB) → 정규화 Detection 리스트 (Scene 계약 형식)."""
        d = self._detect
        h, w = frame_rgb.shape[:2]
        out = self._pipe.infer({self.input_name: d.preprocess(frame_rgb)})
        raw = d.postprocess(out, w, h)                 # 픽셀 박스 + 모델 NMS
        dets = d.correct_all_signs(frame_rgb, raw)     # 색 보정 + 교차 NMS
        return [
            Detection(
                cls=int(cls), conf=float(conf),
                cx=(x1 + x2) / 2.0 / w, cy=(y1 + y2) / 2.0 / h,
                w=(x2 - x1) / w,        h=(y2 - y1) / h,
            )
            for (x1, y1, x2, y2, conf, cls) in dets
        ]


def ultrasonic_loop(perception, stop_event):
    """HC-SR04 거리 → perception.update_distance() 20Hz. 범위 밖이면 None.

    센서/gpiozero 가 없거나 초기화 실패하면 죽지 않고 계속 None 을 보고한다.
    (배선 안 된 상태에서도 통합·뷰어가 그대로 돌아가게)
    """
    try:
        from gpiozero import DistanceSensor
        sensor = DistanceSensor(echo=ULTRA_ECHO, trigger=ULTRA_TRIG,
                                max_distance=ULTRA_MAX_DIST_M)
    except Exception as e:
        print(f"[ultrasonic] sensor unavailable ({e}) -> reporting None")
        while not stop_event.is_set():
            perception.update_distance(None)
            time.sleep(ULTRA_PERIOD_S)
        return

    out_of_range = ULTRA_MAX_DIST_M * 0.99
    while not stop_event.is_set():
        try:
            d = sensor.distance                   # 0~max (m)
            perception.update_distance(None if d >= out_of_range else d)
        except Exception:
            perception.update_distance(None)      # 읽기 실패 → None
        time.sleep(ULTRA_PERIOD_S)


def camera_loop(perception, stop_event, hef_path=HEF_PATH, debug_view=False):
    """멀티스트림 카메라: main(RGB)→YOLO, lores(YUV)→차선. 한 루프에서 둘 다 갱신.

    debug_view=True 면 두 창을 띄운다 (반드시 메인 스레드에서 호출 — imshow 안정성):
      'Camera' : main 프레임 + 객체 박스 + ego ROI(하단 밴드)
      'BEV'    : 마젠타 중앙선 + heading(rad+deg) + curvature HUD
    """
    import cv2
    from picamera2 import Picamera2
    from algorithm import lane_pipeline

    cam = Picamera2()
    cfg = cam.create_video_configuration(
        main={"size": MAIN_SIZE,  "format": "RGB888"},
        lores={"size": LORES_SIZE, "format": "YUV420"},
        raw={"size": RAW_SIZE},                 # 풀 FOV 고정 (차선 BEV 캘리와 동일 화각)
        controls={"FrameRate": FRAME_RATE},
    )                                            # 카메라 정방향 장착 → 회전(hflip/vflip) 없음
    cam.configure(cfg)
    cam.start()

    try:
        with ObjectDetector(hef_path) as detector:
            while not stop_event.is_set():
                # --- 객체: main 스트림 (RGB) ---
                frame_rgb = cam.capture_array("main")
                objects = detector.infer_detections(frame_rgb)
                perception.update_objects(objects)

                # --- 차선: lores 스트림 (YUV → BGR) ---
                yuv = cam.capture_array("lores")
                bgr = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_I420)

                if debug_view:
                    lane, bev_vis = lane_pipeline.process_view(bgr)
                    perception.update_lane(*lane)
                    _show_debug(cv2, frame_rgb, objects, bev_vis,
                                lane_pipeline._L.NEAR_ROI_Y0_FRAC,
                                perception._latest["dist_front_m"])   # 초음파 거리(m or None)
                    if (cv2.waitKey(1) & 0xFF) == 27:    # ESC
                        stop_event.set()
                else:
                    perception.update_lane(*lane_pipeline.process(bgr))
    finally:
        cam.stop()
        cam.close()
        if debug_view:
            cv2.destroyAllWindows()


def _show_debug(cv2, frame_rgb, objects, bev_vis, roi_y0_frac, dist_m=None):
    """디버그 두 창 렌더 (camera_loop debug_view 전용)."""
    from algorithm import object_detection as _d
    # picamera2 "RGB888" 배열은 실제로 BGR 순서 → cv2/imshow 에 그대로 사용 (변환 금지)
    cam_bgr = frame_rgb.copy()
    h, w = cam_bgr.shape[:2]

    # 초음파 거리 (화면 상단) — 미연결/범위밖이면 None
    dist_txt = "ULTRASONIC: None" if dist_m is None else f"ULTRASONIC: {dist_m * 100:.0f} cm"
    cv2.putText(cam_bgr, dist_txt, (10, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)

    # 객체 박스 (정규화 Detection → 픽셀 박스 복원 후 detect.draw 재사용)
    px = [(int((o.cx - o.w / 2) * w), int((o.cy - o.h / 2) * h),
           int((o.cx + o.w / 2) * w), int((o.cy + o.h / 2) * h),
           o.conf, o.cls) for o in objects]
    _d.draw(cam_bgr, px)

    # ego 차선 검출 ROI (하단 밴드, 차선이 보는 근거리 영역)
    y0 = int(h * roi_y0_frac)
    cv2.rectangle(cam_bgr, (0, y0), (w - 1, h - 1), (0, 255, 255), 2)
    cv2.putText(cam_bgr, "ego ROI", (10, max(y0 - 8, 20)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

    cv2.imshow("Camera (objects + ego ROI)", cv2.resize(cam_bgr, (960, 540)))
    cv2.imshow("BEV (lane)", bev_vis)
