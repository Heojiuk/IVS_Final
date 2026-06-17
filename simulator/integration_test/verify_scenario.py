"""통합테스트 검증 — 기록 CSV 를 시나리오(scenario_oval) 기대값과 대조해 PASS/FAIL 리포트.

실행:
    cd simulator
    python integration_test/verify_scenario.py "<CSV경로>"

체크포인트: 시각 구간별 기대 behavior/lane (실제 _decide_leader 기준).
전역검사: 주기 50ms·통신 seq 연속·패킷↔버스 일치·모션 PWM.
"""
import os, sys
sys.path.insert(0, os.path.normpath(os.path.join(os.path.dirname(__file__), '..')))
import _src_path; _src_path.add()

import csv, collections

import scenario_oval

PERIOD_MS = 50.0


def _load(path):
    with open(path, newline='', encoding='utf-8') as f:
        return list(csv.DictReader(f))


def _rows_in(rows, lo, hi):
    out = []
    for r in rows:
        try:
            t = float(r['scn_t_ms'])
        except (ValueError, KeyError):
            continue
        if lo <= t <= hi:
            out.append(r)
    return out


def _majority(values):
    c = collections.Counter(values)
    if not c:
        return '', 0.0
    val, n = c.most_common(1)[0]
    return val, n / len(values)


def check_checkpoints(rows):
    res = []
    for cp in scenario_oval.CHECKPOINTS:
        lo, hi = cp['win']
        win = _rows_in(rows, lo, hi)
        vals = [r.get(cp['field'], '') for r in win]
        meas, ratio = _majority(vals)
        ok = bool(win) and meas == cp['expect'] and ratio >= 0.8   # 전이 1~2 사이클 허용
        res.append({
            'name': f"{cp['name']} [{lo/1000:.1f}~{hi/1000:.1f}s]",
            'expect': cp['expect'],
            'meas': f"{meas}({ratio*100:.0f}%)" if win else "행없음",
            'ok': ok,
        })
    return res


def timing_analysis(rows):
    """주기(dt) 분석 블록을 출력하고, 표 한 줄용 판정 결과를 반환.

    dt = 사이클 간 실측 간격(ms), 목표 50ms. dev = |dt-50| 편차.
    Pi(Linux) 기준 거의 모든 사이클이 ±15ms 이내여야 함 (Windows 는 sleep 해상도로 지터 큼)."""
    dts = [float(r['dt_ms']) for r in rows if r.get('dt_ms') not in ('', None)]
    if not dts:
        print("[주기 분석] dt 데이터 없음\n")
        return {'name': '주기 50ms 안정성', 'expect': '-', 'meas': '데이터없음', 'ok': False}
    n = len(dts)
    devs = sorted(abs(d - PERIOD_MS) for d in dts)
    mean_dt = sum(dts) / n
    mean_dev = sum(devs) / n
    p95 = devs[min(n - 1, int(n * 0.95))]
    le5 = sum(1 for d in devs if d <= 5)
    mid = sum(1 for d in devs if 5 < d <= 15)
    gt15 = sum(1 for d in devs if d > 15)
    print(f"[주기 분석] 목표 {PERIOD_MS:.0f}ms · {n}사이클")
    print(f"  dt     평균 {mean_dt:5.1f}ms   min {min(dts):4.0f}   max {max(dts):4.0f}")
    print(f"  |편차| 평균 {mean_dev:5.1f}ms   p95 {p95:4.1f}   최대 {devs[-1]:4.1f}")
    print(f"  ±5ms이내 {le5}({100*le5/n:.0f}%) | ±5~15ms {mid}({100*mid/n:.0f}%) | ±15ms초과 {gt15}({100*gt15/n:.0f}%)\n")
    ok = gt15 <= max(1, n // 50)   # ≈2% 이내만 허용 (Pi 기준)
    return {'name': '주기 50ms 안정성', 'expect': '±15ms초과 ≲2%',
            'meas': f"초과 {gt15}/{n}, max편차 {devs[-1]:.0f}ms", 'ok': ok}


def check_globals(rows):
    res = []

    seqs = [int(r['tx_seq']) for r in rows if r.get('tx_seq') not in ('', None)]
    gaps = sum(1 for a, b in zip(seqs, seqs[1:]) if ((b - a) & 0xFFFF) != 1)
    res.append({'name': '통신 seq 연속(+1)', 'expect': 'gap 0',
                'meas': f"gap {gaps} ({len(seqs)}개 송신)", 'ok': bool(seqs) and gaps == 0})

    # 패킷 lane = 인지 current_lane (둘 다 같은 사이클 SCENE 출처) — 표본 0이면 미검증(FAIL)
    lane_n = [r for r in rows if r.get('tx_lane') not in ('', None)]
    mism_lane = sum(1 for r in lane_n if r.get('tx_lane') != r.get('current_lane'))
    res.append({'name': '패킷 lane = 인지 current_lane', 'expect': '불일치 0',
                'meas': f"불일치 {mism_lane}/{len(lane_n)}송신" if lane_n else "송신표본 없음",
                'ok': bool(lane_n) and mism_lane == 0})

    # 판단→모션 behavior 전달 (command.behavior → ego.behavior)
    ego_n = [r for r in rows if r.get('ego_behavior') not in ('', None)]
    mism_ce = sum(1 for r in ego_n if r.get('behavior') not in ('', None)
                  and r.get('behavior') != r.get('ego_behavior'))
    res.append({'name': '판단→모션 behavior 전달', 'expect': '불일치 0',
                'meas': f"불일치 {mism_ce}/{len(ego_n)}발행" if ego_n else "발행표본 없음",
                'ok': bool(ego_n) and mism_ce == 0})

    # 모션→통신 직렬화 (ego.behavior 가 패킷에 그대로) — 패킷은 EgoState 를 직렬화함
    beh_n = [r for r in rows if r.get('tx_behavior') not in ('', None)]
    mism_beh = sum(1 for r in beh_n if r.get('tx_behavior') != r.get('ego_behavior'))
    res.append({'name': '패킷 behavior = 모션 ego_behavior', 'expect': '불일치 0',
                'meas': f"불일치 {mism_beh}/{len(beh_n)}송신" if beh_n else "송신표본 없음",
                'ok': bool(beh_n) and mism_beh == 0})

    def _f(v):
        try:
            return float(v)
        except (TypeError, ValueError):
            return None
    bad_thr = sum(1 for r in rows for t in [_f(r.get('throttle_pwm'))]
                  if t is not None and not (0.0 <= t <= 1.0))
    bad_str = sum(1 for r in rows for s in [_f(r.get('steer_pwm'))]
                  if s is not None and not (-1.0 <= s <= 1.0))
    res.append({'name': '모션 PWM 범위(thr 0~1·steer ±1)', 'expect': '범위이탈 0',
                'meas': f"thr이탈 {bad_thr}, steer이탈 {bad_str}", 'ok': bad_thr == 0 and bad_str == 0})
    return res


def main():
    if len(sys.argv) < 2:
        print("사용법: python integration_test/verify_scenario.py <CSV경로>", file=sys.stderr)
        sys.exit(2)
    path = sys.argv[1]
    rows = _load(path)
    print(f"\n=== 통합테스트 검증 — {os.path.basename(path)} ({len(rows)} 사이클) ===\n")
    timing = timing_analysis(rows)   # 주기 상세 블록 출력 + 표 판정 1줄 반환
    print(f"{'항목':<38}{'기대값':<18}{'측정값':<22}결과")
    print('-' * 86)
    npass = 0
    allres = check_checkpoints(rows) + [timing] + check_globals(rows)
    for r in allres:
        mark = 'PASS' if r['ok'] else 'FAIL'
        npass += int(r['ok'])
        print(f"{r['name']:<38}{r['expect']:<18}{r['meas']:<22}{mark}")
    print('-' * 86)
    print(f"합계: {npass}/{len(allres)} PASS, {len(allres) - npass} FAIL\n")
    sys.exit(0 if npass == len(allres) else 1)


if __name__ == '__main__':
    main()
