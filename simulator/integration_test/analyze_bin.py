"""bin 파일 분석 → HTML 보고서 생성.

실행:
    cd simulator
    python integration_test/analyze_bin.py "<bin파일 경로>"

60B STATE 패킷(본문 28B + HMAC 32B)을 파싱해 검증 항목(기대값·측정값·PASS/FAIL)과
거동별 통계를 report.html 로 저장한다.  bin 과 같은 폴더에 저장.
"""
import os, sys, struct, hmac as _hmac, hashlib, html

sys.path.insert(0, os.path.normpath(os.path.join(os.path.dirname(__file__), '..')))
import _src_path; _src_path.add()

from core_module import config
from core_module.v2v import PACKET_LEN, fmt_ms_of_day
from messages import DriveBehavior

_FMT = "!BBBHdIBBffx"


# ── 파싱 ─────────────────────────────────────────────────────────────────────

def _load_packets(path, key):
    """bin 파일에서 60B 패킷을 순서대로 읽어 dict 리스트 반환.  HMAC 실패는 건너뜀."""
    pkts = []
    with open(path, 'rb') as f:
        raw = f.read()
    total = len(raw) // PACKET_LEN
    skipped = 0
    for i in range(total):
        chunk = raw[i * PACKET_LEN:(i + 1) * PACKET_LEN]
        body, mac = chunk[:28], chunk[28:]
        if not _hmac.compare_digest(mac, _hmac.new(key, body, hashlib.sha256).digest()):
            skipped += 1
            continue
        ver, typ, role, seq, t, tx_abs, lane, beh_i, thr, st = struct.unpack(_FMT, body)
        try:
            beh = DriveBehavior(beh_i)
        except ValueError:
            beh = None
        pkts.append({'seq': seq, 't': t, 'tx_abs': tx_abs, 'lane': lane,
                     'beh': beh, 'thr': thr, 'st': st})
    return pkts, total, skipped


# ── 검증 체크 ─────────────────────────────────────────────────────────────────

def _check_seq(pkts):
    seqs = [p['seq'] for p in pkts]
    gaps = sum(1 for a, b in zip(seqs, seqs[1:]) if ((b - a) & 0xFFFF) != 1)
    ok = (len(seqs) > 0) and gaps == 0
    return 'gap=0', f"gap {gaps}/{len(seqs)}패킷", ok


def _check_start_cruise(pkts):
    v = pkts[0]['beh'].name if pkts and pkts[0]['beh'] is not None else 'N/A'
    return 'CRUISE', v, v == 'CRUISE'


def _check_end_cruise(pkts):
    v = pkts[-1]['beh'].name if pkts and pkts[-1]['beh'] is not None else 'N/A'
    return 'CRUISE', v, v == 'CRUISE'


def _check_lc_count(pkts):
    count = sum(1 for a, b in zip(pkts, pkts[1:])
                if (a['beh'] != DriveBehavior.LANE_CHANGE)
                and (b['beh'] == DriveBehavior.LANE_CHANGE))
    return '4회', f"{count}회", count == 4


def _check_stop_exists(pkts):
    found = any(p['beh'] == DriveBehavior.STOP for p in pkts)
    return 'YES', 'YES' if found else 'NO', found


def _check_lc_steer(pkts):
    """LANE_CHANGE 이벤트별 행 반환 — (name, expect, meas, ok) 리스트.
    lane=2→1: 좌조향(steer<0, 기대=-0.5)  lane=1→2: 우조향(steer>0, 기대=+0.5)"""
    events = []
    in_lc = False
    for p in pkts:
        if p['beh'] == DriveBehavior.LANE_CHANGE:
            if not in_lc:
                in_lc = True
                events.append({'lane': p['lane'], 'st': p['st']})
        else:
            in_lc = False

    rows = []
    for i, ev in enumerate(events):
        lf, actual = ev['lane'], ev['st']
        lt = 1 if lf == 2 else 2
        exp = -0.5 if lf == 2 else 0.5
        ok = (exp < 0 and actual < 0) or (exp > 0 and actual > 0)
        rows.append((
            f"LANE_CHANGE #{i+1}  ({lf}→{lt})",
            f"steer {'-0.5 (좌)' if exp < 0 else '+0.5 (우)'}",
            f"steer {actual:+.2f}",
            ok,
        ))
    return rows


def _check_stop_throttle(pkts):
    stop_pkts = [p for p in pkts if p['beh'] == DriveBehavior.STOP]
    bad = sum(1 for p in stop_pkts if abs(p['thr']) > 0.01)
    ok = bool(stop_pkts) and bad == 0
    return '이탈 0', f"이탈 {bad}/{len(stop_pkts)}pkt", ok


def _check_cruise_throttle(pkts):
    cruise_pkts = [p for p in pkts if p['beh'] == DriveBehavior.CRUISE]
    bad = sum(1 for p in cruise_pkts if abs(p['thr'] - 0.6) > 0.01)
    ok = bool(cruise_pkts) and bad == 0
    return '이탈 0', f"이탈 {bad}/{len(cruise_pkts)}pkt", ok


def _check_pwm_range(pkts):
    bad = sum(1 for p in pkts
              if not (0.0 <= p['thr'] <= 1.0) or not (-1.0 <= p['st'] <= 1.0))
    return '이탈 0', f"이탈 {bad}/{len(pkts)}", bad == 0


CHECKS = [
    ('seq 연속(gap=0)',             _check_seq),
    ('시작=CRUISE',                 _check_start_cruise),
    ('종료=CRUISE',                 _check_end_cruise),
    ('LANE_CHANGE 4회',             _check_lc_count),
    ('STOP 존재',                   _check_stop_exists),
    ('LANE_CHANGE steer 방향',      _check_lc_steer),
    ('STOP throttle=0',             _check_stop_throttle),
    ('CRUISE throttle=0.6',         _check_cruise_throttle),
    ('PWM 범위(thr 0~1, steer ±1)', _check_pwm_range),
]


# ── 링크 갭 분석 ─────────────────────────────────────────────────────────────

_STALE_MS = 200.0   # config.LINK_STALE_MS
_LOST_MS  = 500.0   # config.LINK_LOST_MS


def _link_gap_analysis(pkts):
    """tx_abs(송신 절대시각 ms-of-day) 간격으로 수신 측 STALE/LOST 구간을 추정한다.

    반환: {
        'events': [(abs_start_ms, gap_ms, state_str), ...],  STALE/LOST 이벤트
        'n_stale', 'n_lost', 'total_stale_ms', 'total_lost_ms',
        'worst_ms', 'mean_gap_ms', 'n_total_gaps',
    }
    """
    if len(pkts) < 2:
        return None
    events, total_stale, total_lost = [], 0.0, 0.0
    gaps = []
    for a, b in zip(pkts, pkts[1:]):
        g = b['tx_abs'] - a['tx_abs']
        if g < 0:                   # 자정 롤오버 (드묾)
            g += 86_400_000
        gaps.append(g)
        if g >= _LOST_MS:
            events.append((a['tx_abs'], g, 'LOST'))
            total_lost += g
        elif g >= _STALE_MS:
            events.append((a['tx_abs'], g, 'STALE'))
            total_stale += g

    n_stale = sum(1 for *_, s in events if s == 'STALE')
    n_lost  = sum(1 for *_, s in events if s == 'LOST')
    return {
        'events':         events,
        'n_stale':        n_stale,
        'n_lost':         n_lost,
        'total_stale_ms': total_stale,
        'total_lost_ms':  total_lost,
        'worst_ms':       max(gaps),
        'mean_gap_ms':    sum(gaps) / len(gaps),
        'n_total_gaps':   len(gaps),
    }


# ── 통계 ─────────────────────────────────────────────────────────────────────

def _behavior_stats(pkts):
    from collections import Counter, defaultdict
    rows = []
    by_beh = defaultdict(list)
    for p in pkts:
        k = p['beh'].name if p['beh'] is not None else 'UNKNOWN'
        by_beh[k].append(p)
    for beh_name, ps in sorted(by_beh.items()):
        thrs = [p['thr'] for p in ps]
        sts  = [p['st']  for p in ps]
        lanes = Counter(p['lane'] for p in ps)
        rows.append({
            'beh': beh_name,
            'cnt': len(ps),
            'thr_avg': sum(thrs) / len(thrs),
            'thr_min': min(thrs), 'thr_max': max(thrs),
            'st_min': min(sts),   'st_max': max(sts),
            'lanes': dict(lanes),
        })
    return rows


# ── HTML 생성 ──────────────────────────────────────────────────────────────

_PASS_STYLE = 'background:#e8f5e9;color:#1b5e20;font-weight:bold;text-align:center'
_FAIL_STYLE = 'background:#ffebee;color:#b71c1c;font-weight:bold;text-align:center'

_CSS = """
body{font-family:'Segoe UI',sans-serif;margin:0;background:#f0f2f5;color:#222}
.wrap{max-width:1140px;margin:auto;padding:28px}
h1{font-size:1.65em;color:#1a237e;margin-bottom:2px}
.sub{color:#666;font-size:.9em;margin-bottom:20px}
h2{font-size:1.1em;color:#283593;border-left:4px solid #3949AB;padding-left:10px;margin-top:32px}
.card{background:#fff;border-radius:10px;padding:20px;margin:14px 0;box-shadow:0 1px 4px rgba(0,0,0,.08)}
table{width:100%;border-collapse:collapse}
th{background:#3949AB;color:#fff;padding:10px 14px;text-align:left;font-size:.9em}
td{padding:9px 14px;border-bottom:1px solid #eee;font-size:.88em}
tr:last-child td{border-bottom:none}
.pass{color:#1b5e20;font-weight:bold}
.fail{color:#b71c1c;font-weight:bold}
.summary{display:flex;gap:20px;flex-wrap:wrap;margin:12px 0}
.chip{padding:8px 18px;border-radius:20px;font-weight:bold;font-size:.95em}
.chip-pass{background:#e8f5e9;color:#1b5e20}
.chip-fail{background:#ffebee;color:#b71c1c}
.chip-info{background:#e3f2fd;color:#0d47a1}
"""


def _h(s):
    return html.escape(str(s))


def _build_html(bin_path, pkts, results, stats, total_raw, skipped, link=None):
    import datetime
    fname = os.path.basename(bin_path)
    now = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    n_pass = sum(1 for *_, ok in results if ok)
    n_fail = len(results) - n_pass
    verdict = 'PASS' if n_fail == 0 else 'FAIL'
    v_chip = 'chip-pass' if verdict == 'PASS' else 'chip-fail'

    lines = [
        '<!DOCTYPE html><html lang="ko"><head>',
        '<meta charset="UTF-8">',
        '<title>V2V 구동 검증 보고서</title>',
        f'<style>{_CSS}</style>',
        '</head><body><div class="wrap">',
        f'<h1>V2V 구동 검증 보고서</h1>',
        f'<div class="sub">{_h(fname)} &nbsp;·&nbsp; 분석: {now}</div>',
        '<div class="summary">',
        f'<span class="chip chip-info">총 {len(pkts)}패킷 ({total_raw}개 중 {skipped}개 스킵)</span>',
        f'<span class="chip {v_chip}">{verdict} — {n_pass}/{len(results)} 항목</span>',
        '</div>',
    ]

    # ── 검증 항목 ──
    lines += [
        '<h2>검증 항목</h2>',
        '<div class="card" style="padding:0"><table>',
        '<tr><th>항목</th><th>기대값</th><th>측정값</th>'
        '<th style="text-align:center;width:80px">결과</th></tr>',
    ]
    for name, expect, meas, ok in results:
        style = _PASS_STYLE if ok else _FAIL_STYLE
        label = 'PASS' if ok else 'FAIL'
        lines.append(
            f'<tr><td>{_h(name)}</td><td>{_h(expect)}</td>'
            f'<td>{_h(meas)}</td>'
            f'<td style="{style}">{label}</td></tr>'
        )
    lines += ['</table></div>']

    # ── 거동별 통계 ──
    lines += [
        '<h2>거동별 통계</h2>',
        '<div class="card" style="padding:0"><table>',
        '<tr><th>거동</th><th style="text-align:center">패킷 수</th>'
        '<th style="text-align:center">throttle avg [min~max]</th>'
        '<th style="text-align:center">steer [min~max]</th>'
        '<th>차로 분포</th></tr>',
    ]
    for s in stats:
        lines.append(
            f'<tr><td><b>{_h(s["beh"])}</b></td>'
            f'<td style="text-align:center">{s["cnt"]}</td>'
            f'<td style="text-align:center">'
            f'{s["thr_avg"]:.3f} [{s["thr_min"]:.2f}~{s["thr_max"]:.2f}]</td>'
            f'<td style="text-align:center">'
            f'[{s["st_min"]:.2f}~{s["st_max"]:.2f}]</td>'
            f'<td>{_h(s["lanes"])}</td></tr>'
        )
    lines += ['</table></div>']

    # ── 링크 품질 ──
    if link:
        n_lost, n_stale = link['n_lost'], link['n_stale']
        quality = 'GOOD' if n_lost == 0 and n_stale == 0 else ('WARNING' if n_lost == 0 else 'BAD')
        q_chip  = {'GOOD': 'chip-pass', 'WARNING': 'chip-info', 'BAD': 'chip-fail'}[quality]
        lines += [
            '<h2>링크 품질 (TX 간격 기반 추정)</h2>',
            '<div class="card">',
            f'<div class="summary">',
            f'<span class="chip {q_chip}">{quality}</span>',
            f'<span class="chip chip-info">LOST {n_lost}회 / STALE {n_stale}회</span>',
            f'<span class="chip chip-info">평균간격 {link["mean_gap_ms"]:.1f}ms '
            f'/ 최대갭 {link["worst_ms"]:.0f}ms</span>',
            f'<span class="chip chip-info">총 LOST {link["total_lost_ms"]/1000:.1f}s '
            f'/ STALE {link["total_stale_ms"]/1000:.1f}s</span>',
            '</div>',
        ]
        if link['events']:
            lines += [
                '<table style="margin-top:12px">',
                '<tr><th>발생 시각</th><th>갭(ms)</th><th>상태</th></tr>',
            ]
            for abs_ms, gap, state in link['events']:
                color = '#b71c1c' if state == 'LOST' else '#e65100'
                lines.append(
                    f'<tr><td>{_h(fmt_ms_of_day(abs_ms))}</td>'
                    f'<td style="font-weight:bold;color:{color}">{gap:.0f}ms</td>'
                    f'<td style="font-weight:bold;color:{color}">{state}</td></tr>'
                )
            lines += ['</table>']
        else:
            lines += ['<p style="color:#1b5e20">갭 이벤트 없음 — 연속 수신</p>']
        lines += ['</div>']

    lines += ['</div></body></html>']
    return '\n'.join(lines)


# ── 진입점 ────────────────────────────────────────────────────────────────────

def main():
    if len(sys.argv) < 2:
        print("사용법: python integration_test/analyze_bin.py <bin파일>", file=sys.stderr)
        sys.exit(2)
    bin_path = sys.argv[1]
    if not os.path.isfile(bin_path):
        print(f"파일 없음: {bin_path}", file=sys.stderr)
        sys.exit(1)

    key = config.load_key()
    pkts, total_raw, skipped = _load_packets(bin_path, key)
    if not pkts:
        print("[analyze] 유효 패킷 없음 — psk.key 불일치 또는 빈 파일?", file=sys.stderr)
        sys.exit(1)
    print(f"[analyze] {len(pkts)}패킷 로드 (원시 {total_raw}, 스킵 {skipped})")

    results = []
    for check_name, fn in CHECKS:
        ret = fn(pkts)
        if isinstance(ret, list):
            for row in ret:
                name, expect, meas, ok = row
                print(f"  {'PASS' if ok else 'FAIL'}  {name:<38}  기대={expect:<22}  측정={meas}")
                results.append(row)
        else:
            expect, meas, ok = ret
            print(f"  {'PASS' if ok else 'FAIL'}  {check_name:<38}  기대={expect:<22}  측정={meas}")
            results.append((check_name, expect, meas, ok))

    n_pass = sum(1 for *_, ok in results if ok)
    print(f"\n합계: {n_pass}/{len(results)} PASS")

    link = _link_gap_analysis(pkts)
    if link:
        print(f"\n[링크 품질] 평균간격 {link['mean_gap_ms']:.1f}ms / 최대갭 {link['worst_ms']:.0f}ms")
        print(f"  LOST {link['n_lost']}회 (누적 {link['total_lost_ms']/1000:.1f}s) / "
              f"STALE {link['n_stale']}회 (누적 {link['total_stale_ms']/1000:.1f}s)")
        for abs_ms, gap, state in link['events']:
            print(f"  [{state}] {fmt_ms_of_day(abs_ms)}  갭={gap:.0f}ms")

    stats = _behavior_stats(pkts)
    html_out = os.path.join(os.path.dirname(bin_path), 'report.html')
    content = _build_html(bin_path, pkts, results, stats, total_raw, skipped, link)
    with open(html_out, 'w', encoding='utf-8') as f:
        f.write(content)
    print(f"[analyze] 보고서: {html_out}")


if __name__ == '__main__':
    main()
