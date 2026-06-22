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
FRAME_RATE = 20            # 20Hz 제어 루프와 정합 (30→20, 캡처/추론/렌더 부하·전력↓)

# 디버그 창을 매 프레임 그리면 imshow/X 합성 부하로 저전압(undervoltage)→멈춤이 날 수 있다.
# → 인지는 매 프레임 돌리되, 화면만 N프레임마다 1회 그려 전력 스파이크를 줄인다 (debug_view 전용).
VIEW_RENDER_EVERY   = 4
VIEW_CAM_SIZE       = (480, 270)  # 디버그 프레임 크기 (작을수록 JPEG 인코딩·전송 전력↓)
VIEW_BEV_SCALE      = 0.6         # BEV 축소 배율
VIEW_STREAM_PORT    = 8080        # MJPEG 스트리밍 포트 (debug_view 전용)
VIEW_STREAM_QUALITY = 60          # JPEG 화질 — 낮을수록 인코딩 전력↓

# ── YOLO 모델 (고정 경로 — 명령어 인자 없이 사용) ─────────────────────
HEF_PATH = "/home/jhoh/yolov11n.hef"   # Pi마다 위치 다르면 이 한 줄만 수정

# ── 초음파 ────────────────────────────────────────────────────────────
ULTRA_ECHO, ULTRA_TRIG = 8, 11      # BCM (벤치 테스트와 동일)
ULTRA_MAX_DIST_M       = 4.0
ULTRA_PERIOD_S         = 0.05       # 20Hz


class MJPEGStreamer:
    """cv2.imshow() 대체 — MJPEG + 데이터 대시보드 HTTP 서버 (저전력 원격 디버그).

    X11 합성 없이 JPEG 인코딩만 수행. 브라우저에서 http://<RPi-IP>:VIEW_STREAM_PORT/ 접속.
    bus 전달 시 /data 엔드포인트로 모든 토픽(Scene/Command/Mode/Link/Peer/EgoState) 제공.
    """

    _DASH = """\
<!doctype html><html><head><meta charset="utf-8"><title>IVS Debug</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{background:#0d0d0d;color:#e0e0e0;font-family:'Courier New',monospace;font-size:13px;display:flex;flex-direction:column;height:100vh}
.hdr{background:#111;padding:6px 14px;display:flex;align-items:center;gap:12px;border-bottom:1px solid #1e1e1e;flex-shrink:0}
.hdr h1{font-size:13px;color:#4fc3f7;letter-spacing:3px}.role{background:#1a2630;padding:1px 8px;border-radius:2px;color:#80cbc4;font-size:11px}
.dot{width:7px;height:7px;border-radius:50%;background:#333;display:inline-block}.dot.ok{background:#69f0ae}.dot.err{background:#f44336}
.clk{margin-left:auto;color:#333;font-size:11px}
.body{display:flex;flex:1;overflow:hidden}
.vp{flex-shrink:0;padding:8px;display:flex;flex-direction:column;gap:4px;border-right:1px solid #1a1a1a}
.vp img{display:block;max-width:700px;width:auto;height:auto;border:1px solid #1e1e1e;background:#080808}
.vlbl{font-size:10px;color:#333;text-align:center}
.dp{flex:1;overflow-y:auto;padding:8px;display:grid;grid-template-columns:1fr 1fr;gap:7px;align-content:start}
.card{background:#141414;border:1px solid #1e1e1e;border-radius:3px;padding:9px}
.ct{font-size:10px;color:#444;text-transform:uppercase;letter-spacing:1px;margin-bottom:7px}
.bdg{display:inline-block;padding:3px 10px;border-radius:2px;font-size:14px;font-weight:bold;letter-spacing:1px}
.bCRUISE{background:#1b5e20;color:#69f0ae}.bLANE_CHANGE{background:#7f4c00;color:#ffcc02}
.bSTOP{background:#7f0000;color:#ff5252}.bSLOW{background:#7f3000;color:#ffab40}.bX{background:#1c1c1c;color:#444}
.mNORMAL{background:#1b5e20;color:#69f0ae}.mDEGRADED{background:#7f3000;color:#ffab40}.mESTOP{background:#7f0000;color:#ff5252}.mX{background:#1c1c1c;color:#444}
.row{display:flex;justify-content:space-between;padding:2px 0;border-bottom:1px solid #1a1a1a}.row:last-child{border:none}
.k{color:#444}.v{color:#b0bec5;font-weight:bold}.vok{color:#69f0ae}.vwarn{color:#ffab40}.verr{color:#ff5252}
.big{font-size:26px;font-weight:bold;color:#4fc3f7}
.gbg{background:#0a0a0a;height:5px;border-radius:2px;margin-top:4px}
.gf{height:100%;border-radius:2px;transition:width .1s,background .1s}
.s2{grid-column:1/-1}
</style></head><body>
<div class="hdr"><h1>IVS DEBUG</h1><span class="role" id="ri">–</span><span class="dot" id="dot"></span><span class="clk" id="ck"></span></div>
<div class="body">
<div class="vp"><img id="si" src="/stream"><div class="vlbl">Camera &nbsp;|&nbsp; BEV</div></div>
<div class="dp">
<div class="card"><div class="ct">Behavior</div><span class="bdg bX" id="beh">–</span><div style="margin-top:5px;font-size:11px;color:#555" id="tgt"></div></div>
<div class="card"><div class="ct">Mode</div><span class="bdg mX" id="mod">–</span><div style="margin-top:5px;font-size:11px;color:#555" id="cau"></div></div>
<div class="card s2"><div class="ct">Lane</div>
<div style="display:flex;gap:20px;align-items:flex-start">
<div><div style="font-size:10px;color:#444">lane</div><div class="big" id="ln">–</div></div>
<div style="flex:1">
<div class="row"><span class="k">valid</span><span class="v" id="lv">–</span></div>
<div class="row"><span class="k">offset</span><span class="v" id="lo">–</span></div>
<div class="row"><span class="k">heading</span><span class="v" id="lh">–</span></div>
<div class="row"><span class="k">curvature</span><span class="v" id="lc">–</span></div>
</div></div></div>
<div class="card"><div class="ct">Distance</div><div class="big" id="dv">–</div><div class="gbg"><div class="gf" id="dg" style="width:0%"></div></div><div style="margin-top:4px;font-size:11px" id="fc">–</div></div>
<div class="card"><div class="ct">V2V Link</div>
<div class="row"><span class="k">state</span><span class="v" id="ls">–</span></div>
<div class="row"><span class="k">age</span><span class="v" id="la">–</span></div>
<div class="row"><span class="k">seq</span><span class="v" id="lq">–</span></div>
</div>
<div class="card s2"><div class="ct">V2V Peer</div>
<div style="display:flex;gap:8px;flex-wrap:wrap">
<div class="row" style="flex:1;min-width:120px"><span class="k">behavior</span><span class="v" id="pb">–</span></div>
<div class="row" style="flex:1;min-width:80px"><span class="k">lane</span><span class="v" id="pl">–</span></div>
<div class="row" style="flex:1;min-width:100px"><span class="k">throttle</span><span class="v" id="pt">–</span></div>
<div class="row" style="flex:1;min-width:100px"><span class="k">steer</span><span class="v" id="ps">–</span></div>
</div></div>
<div class="card s2"><div class="ct">Ego Output (Motion)</div>
<div style="display:flex;gap:8px;flex-wrap:wrap">
<div class="row" style="flex:1;min-width:100px"><span class="k">throttle</span><span class="v" id="et">–</span></div>
<div class="row" style="flex:1;min-width:100px"><span class="k">steer</span><span class="v" id="es">–</span></div>
<div class="row" style="flex:1;min-width:120px"><span class="k">behavior</span><span class="v" id="eb">–</span></div>
</div></div>
</div></div>
<script>
const BC={CRUISE:'bCRUISE',LANE_CHANGE:'bLANE_CHANGE',STOP:'bSTOP',SLOW:'bSLOW'};
const MC={NORMAL:'mNORMAL',DEGRADED:'mDEGRADED',ESTOP:'mESTOP'};
const LC={ALIVE:'vok',STALE:'vwarn',LOST:'verr'};
function t(id,v){const e=document.getElementById(id);if(e)e.textContent=v!=null?v:'–';}
function bc(id,v,map){const e=document.getElementById(id);if(!e)return;e.textContent=v||'–';const c=e.className.replace(/\b[bm]\w+/g,'').trim();e.className=c+' '+(map[v]||Object.values(map)[0].replace(/\w+$/,'X'));}
async function poll(){
try{
const d=await fetch('/data').then(r=>r.json());
document.getElementById('dot').className='dot ok';
if(d.role)t('ri',d.role.toUpperCase());
if(d.command){bc('beh',d.command.behavior,BC);t('tgt',d.command.target_lane?'→ lane '+d.command.target_lane:'');}
if(d.mode){bc('mod',d.mode.mode,MC);t('cau',d.mode.cause!=='NONE'?d.mode.cause:'');}
if(d.scene){
const s=d.scene;
t('ln',s.current_lane||'–');
const lv=document.getElementById('lv');if(lv){lv.textContent=s.lane_valid?'YES':'NO';lv.className='v '+(s.lane_valid?'vok':'verr');}
t('lo',s.lane_offset_cm!=null?s.lane_offset_cm.toFixed(1)+' cm':'–');
t('lh',s.lane_heading_rad!=null?(s.lane_heading_rad*57.296).toFixed(1)+'°':'–');
t('lc',s.lane_curvature_1pm!=null?s.lane_curvature_1pm.toFixed(3)+' /m':'–');
const dc=s.dist_front_cm,dg=document.getElementById('dg');
if(dc!=null){t('dv',dc.toFixed(0)+' cm');const p=Math.min(100,dc/400*100);dg.style.width=p+'%';dg.style.background=dc<10?'#f44336':dc<20?'#ff9800':'#4fc3f7';}
else{t('dv','None');dg.style.width='0%';}
const fc=document.getElementById('fc');
if(s.stop_signal){fc.textContent='⚠ STOP SIGNAL';fc.style.color='#ffab40';}
else if(!s.front_clear){fc.textContent='✘ blocked';fc.style.color='#ff5252';}
else{fc.textContent='✔ clear';fc.style.color='#69f0ae';}
}
if(d.link){const ls=document.getElementById('ls');ls.textContent=d.link.state;ls.className='v '+(LC[d.link.state]||'');t('la',d.link.age_rx_ms!=null?d.link.age_rx_ms.toFixed(0)+' ms':'–');t('lq',d.link.last_seq);}
if(d.peer){t('pb',d.peer.behavior);t('pl',d.peer.lane);t('pt',d.peer.throttle_pwm!=null?d.peer.throttle_pwm.toFixed(2):'–');t('ps',d.peer.steer_pwm!=null?d.peer.steer_pwm.toFixed(2):'–');}
if(d.ego_state){t('et',d.ego_state.throttle_pwm.toFixed(2));t('es',d.ego_state.steer_pwm.toFixed(2));t('eb',d.ego_state.behavior);}
}catch(e){document.getElementById('dot').className='dot err';}
}
setInterval(()=>{document.getElementById('ck').textContent=new Date().toTimeString().slice(0,8);},1000);
setInterval(poll,200);poll();
</script></body></html>""".encode("utf-8")

    def __init__(self, port=VIEW_STREAM_PORT, bus=None, role="follower"):
        self._port   = port
        self._lock   = threading.Lock()
        self._jpeg   = None
        self._server = None
        self._bus    = bus
        self._role   = role

    def push(self, cv2_mod, frame_bgr):
        """BGR 프레임 → JPEG 인코딩 → 버퍼 갱신. 렌더 스레드에서 호출."""
        ok, buf = cv2_mod.imencode(
            ".jpg", frame_bgr, [cv2_mod.IMWRITE_JPEG_QUALITY, VIEW_STREAM_QUALITY]
        )
        if ok:
            with self._lock:
                self._jpeg = bytes(buf)

    def _get(self):
        with self._lock:
            return self._jpeg

    def _data_json(self):
        """버스 전 토픽을 읽어 JSON bytes 반환. bus 미연결 시 role만 포함."""
        import json
        out = {"role": self._role}
        bus = self._bus
        if bus is None:
            return json.dumps(out).encode()
        try:
            from core_module.bus import Topics
            from messages import DriveBehavior, Mode, ModeCause, LinkState, Role as _Role

            scene = bus.read(Topics.SCENE)
            if scene:
                out["scene"] = {
                    "lane_valid":        scene.lane_valid,
                    "current_lane":      scene.current_lane,
                    "lane_offset_cm":    round(scene.lane_offset_cm, 2),
                    "lane_heading_rad":  round(scene.lane_heading_rad, 4),
                    "lane_curvature_1pm": round(scene.lane_curvature_1pm, 4),
                    "front_clear":       scene.front_clear,
                    "dist_front_cm":     round(scene.dist_front_cm, 1) if scene.dist_front_cm is not None else None,
                    "stop_signal":       scene.stop_signal,
                }

            cmd = bus.read(Topics.COMMAND)
            if cmd:
                out["command"] = {"behavior": DriveBehavior(cmd.behavior).name, "target_lane": cmd.target_lane}

            mode = bus.read(Topics.MODE)
            if mode:
                out["mode"] = {"mode": Mode(mode.mode).name, "cause": ModeCause(mode.cause).name}

            ego = bus.read(Topics.EGO_STATE)
            if ego:
                out["ego_state"] = {
                    "throttle_pwm": round(ego.throttle_pwm, 3),
                    "steer_pwm":    round(ego.steer_pwm, 3),
                    "behavior":     DriveBehavior(ego.behavior).name,
                }

            link = bus.read(Topics.LINK_STATUS)
            if link:
                out["link"] = {"state": LinkState(link.state).name, "age_rx_ms": round(link.age_rx, 1), "last_seq": link.last_seq}

            peer_topic = Topics.LEADER_STATE if self._role == "follower" else Topics.FOLLOWER_STATE
            peer = bus.read(peer_topic)
            if peer:
                out["peer"] = {
                    "behavior":     DriveBehavior(peer.behavior).name,
                    "lane":         peer.lane,
                    "throttle_pwm": round(peer.throttle_pwm, 3),
                    "steer_pwm":    round(peer.steer_pwm, 3),
                }
        except Exception:
            pass
        return json.dumps(out).encode()

    def start(self):
        import socket
        from http.server import BaseHTTPRequestHandler
        from socketserver import ThreadingTCPServer

        streamer = self

        class _H(BaseHTTPRequestHandler):
            def log_message(self, *_): pass

            def do_GET(self):
                if self.path == "/":
                    body = streamer._DASH
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                    self.send_header("Content-Length", len(body))
                    self.end_headers()
                    self.wfile.write(body)
                elif self.path == "/stream":
                    self.send_response(200)
                    self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                    self.end_headers()
                    try:
                        while True:
                            jpeg = streamer._get()
                            if jpeg:
                                self.wfile.write(b"--frame\r\n")
                                self.wfile.write(b"Content-Type: image/jpeg\r\n\r\n")
                                self.wfile.write(jpeg)
                                self.wfile.write(b"\r\n")
                                self.wfile.flush()
                            time.sleep(0.15)
                    except (BrokenPipeError, ConnectionResetError):
                        pass
                elif self.path == "/data":
                    body = streamer._data_json()
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Content-Length", len(body))
                    self.send_header("Cache-Control", "no-cache")
                    self.end_headers()
                    self.wfile.write(body)

        ThreadingTCPServer.allow_reuse_address = True
        self._server = ThreadingTCPServer(("0.0.0.0", self._port), _H)
        threading.Thread(target=self._server.serve_forever, daemon=True).start()

        try:
            ip = socket.gethostbyname(socket.gethostname())
        except Exception:
            ip = "0.0.0.0"
        print(f"[debug-stream] http://{ip}:{self._port}/  (Ctrl+C 종료)")

    def stop(self):
        if self._server is not None:
            self._server.shutdown()


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


def camera_loop(perception, stop_event, hef_path=HEF_PATH, debug_view=False, bus=None, role="leader"):
    """멀티스트림 카메라: main(RGB)→YOLO, lores(YUV)→차선. 한 루프에서 둘 다 갱신.

    debug_view=True 면 MJPEG 스트리밍 대시보드를 활성화한다 (http://<IP>:8080/).
    bus 를 전달하면 /data 로 전 토픽(Scene·Command·Mode·Link 등)이 노출된다.
    """
    import cv2
    from picamera2 import Picamera2
    from algorithm import lane_pipeline

    streamer = MJPEGStreamer(bus=bus, role=role) if debug_view else None
    if streamer:
        streamer.start()

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
        frame_i = 0
        with ObjectDetector(hef_path) as detector:
            while not stop_event.is_set():
                # --- 객체: main 스트림 (RGB) ---
                frame_rgb = cam.capture_array("main")
                objects = detector.infer_detections(frame_rgb)
                perception.update_objects(objects)

                # --- 차선: lores 스트림 (YUV → BGR) ---
                yuv = cam.capture_array("lores")
                bgr = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_I420)

                # 인지는 매 프레임. 디버그 창은 VIEW_RENDER_EVERY 프레임마다 1회만 그림(저전력).
                if debug_view and frame_i % VIEW_RENDER_EVERY == 0:
                    lane, bev_vis = lane_pipeline.process_view(bgr)
                    perception.update_lane(*lane)
                    _show_debug(cv2, frame_rgb, objects, bev_vis,
                                lane_pipeline._L.NEAR_ROI_Y0_FRAC,
                                perception._latest["dist_front_m"],
                                streamer=streamer)
                    if streamer is None and (cv2.waitKey(1) & 0xFF) == 27:   # ESC (로컬창 전용)
                        stop_event.set()
                else:
                    perception.update_lane(*lane_pipeline.process(bgr))   # 화면만 스킵, 인지는 유지
                frame_i += 1
    finally:
        cam.stop()
        cam.close()
        if streamer:
            streamer.stop()
        elif debug_view:
            cv2.destroyAllWindows()


def lane_camera_loop(perception, stop_event, debug_view=False, bus=None, role="follower"):
    """후행차 전용 카메라 루프 — camera_loop 에서 객체(main/YOLO) 부분만 뺀 형태.

    선행과 동일한 멀티스트림 설정(raw=풀 FOV)으로 BEV 캘리를 그대로 공유하되,
    ObjectDetector(Hailo)를 기동하지 않는다 → AI HAT 없는 후행 Pi에서 동작.
    debug_view=True 면 MJPEG 스트리밍 대시보드를 활성화한다 (http://<IP>:8080/).
    bus 를 전달하면 /data 로 전 토픽(Scene·Command·Mode·Link 등)이 노출된다.
    """
    import cv2
    from picamera2 import Picamera2
    from algorithm import lane_pipeline

    streamer = MJPEGStreamer(bus=bus, role=role) if debug_view else None
    if streamer:
        streamer.start()

    cam = Picamera2()
    cfg = cam.create_video_configuration(
        main={"size": MAIN_SIZE,  "format": "RGB888"},   # lores 의존성상 유지(캡처는 안 함)
        lores={"size": LORES_SIZE, "format": "YUV420"},
        raw={"size": RAW_SIZE},                 # 선행과 동일 FOV — BEV 캘리(SRC_POINTS 등) 그대로 적용
        controls={"FrameRate": FRAME_RATE},
    )                                            # 카메라 정방향 장착 → 회전(hflip/vflip) 없음
    cam.configure(cfg)
    cam.start()

    try:
        frame_i = 0
        while not stop_event.is_set():
            # --- 차선: lores 스트림 (YUV → BGR), 객체인식(main/YOLO)은 수행하지 않음 ---
            yuv = cam.capture_array("lores")
            bgr = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_I420)

            # 인지는 매 프레임. 디버그 창은 VIEW_RENDER_EVERY 프레임마다 1회만 그림(저전력).
            if debug_view and frame_i % VIEW_RENDER_EVERY == 0:
                lane, bev_vis = lane_pipeline.process_view(bgr)
                perception.update_lane(*lane)
                _show_debug(cv2, bgr, [], bev_vis,         # objects=[] → 박스 없음
                            lane_pipeline._L.NEAR_ROI_Y0_FRAC,
                            perception._latest["dist_front_m"],
                            streamer=streamer)
                if streamer is None and (cv2.waitKey(1) & 0xFF) == 27:   # ESC (로컬창 전용)
                    stop_event.set()
            else:
                perception.update_lane(*lane_pipeline.process(bgr))   # 화면만 스킵, 인지는 유지
            frame_i += 1
    finally:
        cam.stop()
        cam.close()
        if streamer:
            streamer.stop()
        elif debug_view:
            cv2.destroyAllWindows()


def _show_debug(cv2, frame_rgb, objects, bev_vis, roi_y0_frac, dist_m=None, streamer=None):
    """디버그 렌더 — streamer 있으면 MJPEG 스트림, 없으면 로컬 창 표시."""
    from algorithm import object_detection as _d
    gray    = cv2.cvtColor(frame_rgb, cv2.COLOR_BGR2GRAY)
    cam_bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    h, w = cam_bgr.shape[:2]

    dist_txt = "ULTRASONIC: None" if dist_m is None else f"ULTRASONIC: {dist_m * 100:.0f} cm"
    cv2.putText(cam_bgr, dist_txt, (10, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)

    px = [(int((o.cx - o.w / 2) * w), int((o.cy - o.h / 2) * h),
           int((o.cx + o.w / 2) * w), int((o.cy + o.h / 2) * h),
           o.conf, o.cls) for o in objects]
    _d.draw(cam_bgr, px)

    y0 = int(h * roi_y0_frac)
    cv2.rectangle(cam_bgr, (0, y0), (w - 1, h - 1), (0, 255, 255), 2)
    cv2.putText(cam_bgr, "ego ROI", (10, max(y0 - 8, 20)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

    cam_small = cv2.resize(cam_bgr, VIEW_CAM_SIZE)
    bev_small = cv2.resize(bev_vis, None, fx=VIEW_BEV_SCALE, fy=VIEW_BEV_SCALE)

    if streamer is not None:
        # cam | bev 좌우 합성 후 JPEG 인코딩 (X11 합성 없음 → 전력↓)
        bev_h = cam_small.shape[0]
        bev_w = int(bev_small.shape[1] * bev_h / bev_small.shape[0])
        composite = cv2.hconcat([cam_small, cv2.resize(bev_small, (bev_w, bev_h))])
        streamer.push(cv2, composite)
    else:
        cv2.imshow("Camera (objects + ego ROI)", cam_small)
        cv2.imshow("BEV (lane)", bev_small)
