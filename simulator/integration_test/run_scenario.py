"""통합테스트 러너 — 시나리오 인지로 실제 [판단→모션→통신] 파이프라인을 50ms로 돌리고 CSV 기록.

실행 (라즈베리파이 leader 또는 PC):
    cd simulator
    python integration_test/run_scenario.py                  # 실제 src/algorithm (기본)
    python integration_test/run_scenario.py --control mock    # sim_algorithm mock (하니스 점검용)
그 후:
    python integration_test/verify_scenario.py "<기록된 CSV>"

생산 루프와 동일하게 실제 Scheduler(50ms·예외격리)를 쓴다. 인지만 ScenarioPerception 으로,
통신만 RecordableV2VModule(TX 패킷 캡처)로 교체/래핑한다. recorder 가 마지막 모듈로 매 사이클 기록.
"""
import os, sys
sys.path.insert(0, os.path.normpath(os.path.join(os.path.dirname(__file__), '..')))  # simulator/
import _src_path; _src_path.add()                                                     # src/

import argparse, csv, datetime, threading, time

from core_module.bus import MessageBus, Topics
from core_module import config
from core_module.scheduler import Scheduler
from core_module.v2v import packet_parser, fmt_ms_of_day
from messages import Role
from logger import RecordableV2VModule
from sim_algorithm.scenario_perception import ScenarioPerception
import scenario_oval

PERIOD_S = config.LOOP_PERIOD_S
LOG_DIR = os.path.normpath(os.path.join(os.path.dirname(__file__), '..', '..', 'data', 'log', 'IntegrationTest'))

CSV_COLUMNS = [
    'tick', 't_wall', 't_mono_s', 'dt_ms', 'cycle_jitter_ms', 'scn_t_ms',
    'lane_valid', 'current_lane', 'lane_offset_cm', 'lane_heading_rad',
    'lane_curvature_1pm', 'front_clear', 'dist_front_cm', 'stop_signal',
    'behavior', 'target_lane', 'mode', 'cause',
    'throttle_pwm', 'steer_pwm', 'ego_behavior',
    'tx_seq', 'tx_abs', 'tx_lane', 'tx_behavior', 'tx_throttle_pwm', 'tx_steer_pwm',
    'link_state', 'link_age_rx_ms', 'link_last_seq',
]


class _TxHolder:
    """on_tx_cb 로 받은 직전 송신 패킷(raw 60B)을 한 칸 보관 — 같은 사이클 recorder 가 읽어 비움."""
    def __init__(self):
        self.last = None
    def capture(self, raw):
        self.last = raw


def _enum_name(v):
    return v.name if hasattr(v, 'name') else ('' if v is None else str(v))


class IntegrationRecorder:
    """스케줄러 마지막 모듈 — 매 사이클 버스 스냅샷 + 실제 송신 패킷(TX)을 CSV 한 행으로 기록."""

    def __init__(self, writer, perception, tx_holder, key):
        self._w = writer
        self._perc = perception
        self._tx = tx_holder
        self._key = key
        self._tick = 0
        self._prev_mono = None
        self.dts = []        # 사이클 주기(ms) 누적 — 종료 시 요약

    def step(self, bus):
        now = time.monotonic()
        if self._prev_mono is None:
            dt_ms = jitter_ms = ''
        else:
            dt = (now - self._prev_mono) * 1000.0
            self.dts.append(dt)
            dt_ms, jitter_ms = round(dt, 2), round(dt - 50.0, 2)   # cycle_jitter_ms = 50ms 대비 ±편차 (+늦음/−빠름)
        self._prev_mono = now

        scene = bus.read(Topics.SCENE)
        cmd   = bus.read(Topics.COMMAND)
        mode  = bus.read(Topics.MODE)
        ego   = bus.read(Topics.EGO_STATE)
        link  = bus.read(Topics.LINK_STATUS)

        row = {c: '' for c in CSV_COLUMNS}
        row['tick'] = self._tick
        row['t_wall'] = datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3]
        row['t_mono_s'] = round(now, 4)
        row['dt_ms'] = dt_ms
        row['cycle_jitter_ms'] = jitter_ms
        row['scn_t_ms'] = round(self._perc.elapsed_ms(), 1)

        if scene is not None:
            row['lane_valid'] = scene.lane_valid
            row['current_lane'] = scene.current_lane
            row['lane_offset_cm'] = round(scene.lane_offset_cm, 2)
            row['lane_heading_rad'] = round(scene.lane_heading_rad, 4)
            row['lane_curvature_1pm'] = round(scene.lane_curvature_1pm, 4)
            row['front_clear'] = scene.front_clear
            row['dist_front_cm'] = '' if scene.dist_front_cm is None else round(scene.dist_front_cm, 1)
            row['stop_signal'] = scene.stop_signal
        if cmd is not None:
            row['behavior'] = _enum_name(cmd.behavior)
            row['target_lane'] = cmd.target_lane
        if mode is not None:
            row['mode'] = _enum_name(mode.mode)
            row['cause'] = _enum_name(mode.cause)
        if ego is not None:
            row['throttle_pwm'] = round(ego.throttle_pwm, 4)
            row['steer_pwm'] = round(ego.steer_pwm, 4)
            row['ego_behavior'] = _enum_name(ego.behavior)  # 패킷이 실제 직렬화하는 값(모션 출력)
        if self._tx.last is not None:   # 실제 송신 패킷(ground truth) 파싱
            try:
                st = packet_parser(self._tx.last, self._key)
                row['tx_seq'] = st.seq
                row['tx_abs'] = fmt_ms_of_day(st.tx_abs)
                row['tx_lane'] = st.lane
                row['tx_behavior'] = _enum_name(st.behavior)
                row['tx_throttle_pwm'] = round(st.throttle_pwm, 4)
                row['tx_steer_pwm'] = round(st.steer_pwm, 4)
            except ValueError:
                pass
            self._tx.last = None        # 사이클 경계 — 다음 사이클 미송신이면 빈칸
        if link is not None:
            row['link_state'] = _enum_name(link.state)
            row['link_age_rx_ms'] = round(link.age_rx, 1)
            row['link_last_seq'] = link.last_seq

        self._w.writerow(row)
        self._tick += 1


def build_modules(control, steps, initial):
    """(perception, decision, motion, v2v, tx_holder) — 인지=시나리오, 판단·모션=real|mock, 통신=실제(TX캡처)."""
    role = Role.LEADER
    perception = ScenarioPerception(steps, initial)
    if control == 'real':
        from algorithm.decision import DecisionModule
        from algorithm.motion_planning import MotionModule
        decision, motion = DecisionModule(role), MotionModule(role)
    else:  # mock — sim_algorithm (하니스 점검용)
        from sim_algorithm.decision import LocalDecisionModule
        from sim_algorithm.motion_planning import LocalMotionModule
        decision, motion = LocalDecisionModule(role), LocalMotionModule(role)
    tx_holder = _TxHolder()
    v2v = RecordableV2VModule('leader', on_tx_cb=tx_holder.capture)  # rx 5006 bind
    return perception, decision, motion, v2v, tx_holder


def main():
    ap = argparse.ArgumentParser(description='V2V 통합테스트 — 시나리오로 실제 판단/모션/통신 검증 (leader)')
    ap.add_argument('--control', choices=['real', 'mock'], default='real',
                    help="판단·모션: real=src/algorithm(기본) | mock=sim_algorithm(하니스 점검용)")
    ap.add_argument('--out', default=None, help='CSV 출력 경로 (기본: data/log/IntegrationTest/<시각>_<control>.csv)')
    ap.add_argument('--duration', type=float, default=None, help='재생 길이(초) 강제 (기본: 시나리오 길이)')
    ap.add_argument('--peer', default=None, metavar='IP',
                    help='후행차(PC) IP — leader 가 V2V leader_state 를 보낼 대상 (없으면 IVS_MODE/_IPS)')
    ap.add_argument('--scenario', default=None, metavar='JSON',
                    help='시나리오 JSON 경로 (GUI 시나리오 편집기 저장본 = 디지털트윈 트랙 인지 타임라인). 없으면 내장 scenario_oval')
    args = ap.parse_args()
    if args.peer:   # leader 송신대상(후행차=PC) 직접 지정 — VILS(Pi leader → PC follower) 용. app.py 와 동일 관용구
        config._IPS[config.mode()]['follower'] = args.peer

    if args.scenario:   # GUI 편집기에서 가상 트랙+장애물에 맞춰 저장한 인지 타임라인
        from scenario import load_scenario
        steps = load_scenario(args.scenario)
        default_dur = max((s.t_ms for s in steps), default=0) + 3000.0   # 마지막 이벤트 + 여유 3s
    else:               # 내장 타원 시나리오 (회귀 테스트용)
        steps = scenario_oval.STEPS
        default_dur = scenario_oval.DURATION_MS
    initial = scenario_oval.INITIAL   # 시작 상태 기본 (JSON t=0 step 으로 덮어쓰기 가능)
    dur_ms = args.duration * 1000.0 if args.duration else default_dur
    out = args.out
    if out is None:
        os.makedirs(LOG_DIR, exist_ok=True)
        out = os.path.join(LOG_DIR, datetime.datetime.now().strftime('%Y%m%d_%H%M%S') + f'_{args.control}.csv')

    key = config.load_key()
    try:
        perception, decision, motion, v2v, tx_holder = build_modules(args.control, steps, initial)
    except OSError as e:
        print(f"[itest] 기동 실패 (소켓/포트: {e}). main.py 가 떠 있거나 포트 점유? 중단.", file=sys.stderr)
        sys.exit(1)

    bus = MessageBus()
    f = open(out, 'w', newline='', encoding='utf-8')
    writer = csv.DictWriter(f, fieldnames=CSV_COLUMNS)
    writer.writeheader()
    recorder = IntegrationRecorder(writer, perception, tx_holder, key)

    sched = Scheduler(PERIOD_S, [perception, decision, motion, v2v, recorder], bus)
    v2v.start(bus)
    scn = args.scenario if args.scenario else 'scenario_oval(내장)'
    print(f"[itest] control={args.control} role=leader peer={v2v._peer} scenario={scn} dur={dur_ms/1000:.1f}s -> {out}")

    def _stopper():                       # 시나리오 길이 + 여유 후 정지 (sched.run() 블로킹)
        time.sleep(dur_ms / 1000.0 + 0.3)
        sched.stop()
    threading.Thread(target=_stopper, daemon=True).start()

    try:
        sched.run()
    finally:
        v2v.stop()
        f.close()
        print(f"[itest] 완료 — cycles={sched.cycles} overruns={sched.overruns} errors={sched.errors}")
        dts = recorder.dts
        if dts:
            devs = [abs(d - 50.0) for d in dts]
            print(f"[itest] 주기: 평균 {sum(dts)/len(dts):.1f}ms (min {min(dts):.0f}/max {max(dts):.0f}), "
                  f"|편차| 평균 {sum(devs)/len(devs):.1f} 최대 {max(devs):.1f}ms, "
                  f"±15ms초과 {sum(1 for d in devs if d > 15.0)}/{len(dts)}")
        print(f"[itest] CSV: {out}")
        print(f'[itest] 검증:  python integration_test/verify_scenario.py "{out}"')
        if sched.errors > 0 and args.control == 'real':
            print("[itest] ⚠ 판단/모션에서 예외 발생 — algorithm 이 새 messages 계약(CRUISE·dist_front_cm)에 "
                  "미정렬일 수 있음 (담당자 정렬 후 errors=0 이어야 함).", file=sys.stderr)


if __name__ == '__main__':
    main()
