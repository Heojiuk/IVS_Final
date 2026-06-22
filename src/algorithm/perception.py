"""인지 (STUB — 인지팀 담당). 센서 읽어 Scene 발행.

실제: 카메라(차선·객체 YOLO) + 초음파 → Scene. 지금은 흐름 확인용 더미값.
입력: 카메라/초음파(HW)   출력: bus[perception/scene]  (IF-B1)

구조: step()은 '최신값 스냅샷 → Scene 조립 → 발행'만 (논블로킹).
      실제 센서 읽기(카메라/초음파)는 update_*()로 최신값만 갱신한다.
      → YOLO 추론이 50ms를 넘겨도 스케줄러가 안 밀린다.
"""
import threading
import time

from core_module.bus import Topics # 버스에 쓰이는 토픽
from messages import Scene, Detection, Role # 전방 객체 및 차선 정보 (IF-B1), 차량 역할

# 센서 핀 (ICD IF-H2 / HWD) 개발자가 수정가능.
ULTRASONIC_TRIG, ULTRASONIC_ECHO = 23, 24   # 전방 초음파 (ECHO 5V→3.3V 분압)

# 객체 클래스 번호 — detect.py 결과 그대로 (CLS_STOP=1)
CLS_STOP = 1                # stop_signal 판정에 사용

# 융합 임계값 (개발자가 조정 — NFR-03 config 후보)
SAFE_DIST_M = 0.5           # 이보다 가까우면 front_clear=False
FRONT_CY_MIN = 0.5          # 박스 중심이 화면 아래 절반에 있어야 '전방'으로 간주


class PerceptionModule:
    def __init__(self, role=Role.LEADER):
        # role=자차 역할 — 후행차(AI HAT 없음)는 YOLO 없이 차선 전용 루프를 돈다
        self.role = role
        # 세 인지원이 각자 주기로 갱신하는 최신값 (lock 보호)
        self._lock = threading.Lock()
        self._latest = {
            # 차선 (BEV+HSV)
            'lane_valid': False,
            'current_lane': 0,
            'lane_offset_m': 0.0,
            'lane_heading_rad': 0.0,
            'lane_curvature_1pm': 0.0,
            # 객체 (YOLO)
            'objects': [],          # List[Detection]
            # 초음파
            'dist_front_m': None,   # 못 재면 None
        }
        self._stop = None           # 센서 스레드 정지 신호 (start에서 생성)
        self._threads = []

    # ===== 센서 스레드 기동/정지 (라즈베리파이 전용) =====================
    def start(self, debug_view=False):
        """카메라·초음파 스레드를 띄운다. 하드웨어 import는 여기서 지연 로딩.
        선행=차선+YOLO(camera_loop), 후행=차선 전용(lane_camera_loop, AI HAT 불필요).
        HEF 경로는 sensing.HEF_PATH 고정값 사용 (명령어 인자 없음).
        debug_view=True 면 MJPEG 스트리밍 활성화 (http://<IP>:8080/)."""
        from algorithm import sensing
        self._stop = threading.Event()
        cam_loop = (sensing.camera_loop if self.role == Role.LEADER
                    else sensing.lane_camera_loop)   # 후행은 YOLO 없는 차선 전용 루프
        self._threads = [
            threading.Thread(target=cam_loop,
                             args=(self, self._stop),
                             kwargs={"debug_view": debug_view}, daemon=True),
            threading.Thread(target=sensing.ultrasonic_loop,
                             args=(self, self._stop), daemon=True),
        ]
        for t in self._threads:
            t.start()

    def stop(self):
        """센서 스레드 정지 신호."""
        if self._stop is not None:
            self._stop.set()

    # ===== ★ 인지팀 여기 작업 =====================================
    # TODO: picamera2(차선) · Hailo YOLO(객체) · 초음파(거리) 를 읽어
    #       아래 update_*() 로 최신값을 갱신하세요. (백그라운드 스레드 권장)
    def update_lane(self, valid, current_lane, offset_m, heading_rad, curvature_1pm):
        """차선 인지 결과 갱신 (BEV+HSV 파이프라인이 호출)."""
        with self._lock:
            self._latest.update(
                lane_valid=valid, current_lane=current_lane,
                lane_offset_m=offset_m, lane_heading_rad=heading_rad,
                lane_curvature_1pm=curvature_1pm,
            )

    def update_objects(self, detections):
        """객체 검출 결과 갱신 (detect.py 결과를 Detection 리스트로). detections=List[Detection]"""
        with self._lock:
            self._latest['objects'] = detections

    def update_distance(self, dist_m):
        """초음파 거리 갱신 (m). 못 재면 None."""
        with self._lock:
            self._latest['dist_front_m'] = dist_m
    # ==============================================================

    def step(self, bus):
        """50ms 주기 — 카메라·초음파로 전방 인지해 scene에 저장하여 bus에 전송.  bus=메시지버스"""
        with self._lock:
            p = dict(self._latest)

        objects = p['objects']
        dist = p['dist_front_m']

        # 가공 1) stop_signal — STOP 표지 검출 여부 (사실만, 정지 '판단'은 decision 몫)
        stop_signal = any(d.cls == CLS_STOP for d in objects)

        # 가공 2) front_clear — 초음파 멀고 AND 전방에 막는 객체 없음 (카메라+초음파 종합)
        far = (dist is None) or (dist > SAFE_DIST_M)
        no_blocker = not any(d.cy > FRONT_CY_MIN for d in objects)
        front_clear = far and no_blocker

        scene = Scene(
            stamp=time.monotonic(),
            lane_valid=p['lane_valid'],
            current_lane=p['current_lane'],
            lane_offset_cm=p['lane_offset_m'] * 100.0,             # 내부 m → 계약 cm
            lane_heading_rad=p['lane_heading_rad'],
            lane_curvature_1pm=p['lane_curvature_1pm'],
            front_clear=front_clear,
            dist_front_cm=(dist * 100.0 if dist is not None else None),  # m → cm
            stop_signal=stop_signal,
            objects=objects,
        )
        bus.publish(Topics.SCENE, scene)               # 출력 IF-B1 (토픽·형식 고정 — 건드리지 말 것)
