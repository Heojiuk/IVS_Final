"""버스 로그(.buslog) 후처리 — Pi가 dev 모드에서 기록한 전 토픽 스냅샷을 파싱·내보내기.

실행:
    cd simulator
    python tools/parse_buslog.py "<파일.buslog>"              # CSV 생성 + 요약
    python tools/parse_buslog.py "<파일.buslog>" --csv out.csv  # 출력 경로 지정

바이너리 포맷의 진실원은 src/core_module/bus_logger.py (read_file). 여기선 그것을 불러
CSV로 풀고, behavior 전이·링크 상태 요약을 콘솔에 찍는다. (xlsx/pdf는 추후 확장)
"""
import os, sys, csv, argparse

sys.path.insert(0, os.path.normpath(os.path.join(os.path.dirname(__file__), "..")))
import _src_path; _src_path.add()

from core_module.bus_logger import read_file, FIELD_NAMES
from core_module.v2v import fmt_ms_of_day


def to_csv(records, out_path):
    """레코드 리스트 → CSV. t_abs_ms 는 사람이 읽는 HH:MM:SS.fff 컬럼을 앞에 추가."""
    cols = ["t_wall"] + FIELD_NAMES
    with open(out_path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for r in records:
            row = dict(r)
            row["t_wall"] = fmt_ms_of_day(r["t_abs_ms"])
            w.writerow(row)


def summarize(header, records):
    """콘솔 요약 — 사이클 수·시간 범위·behavior 전이·링크 상태 분포."""
    print(f"\n=== 버스 로그 - role={header['role_name']} v{header['version']} "
          f"({header['count']} 사이클, 레코드 {header['record_size']}B) ===")
    if not records:
        print("레코드 없음")
        return
    t0, t1 = records[0]["t_abs_ms"], records[-1]["t_abs_ms"]
    print(f"시간: {fmt_ms_of_day(t0)} ~ {fmt_ms_of_day(t1)}  ({(t1 - t0) / 1000:.1f}s)")

    # behavior 전이 (이전과 달라진 지점)
    print("\n[behavior 전이]")
    prev = None
    for r in records:
        b = r["behavior"]
        if b != prev:
            extra = ""
            if b == "LANE_CHANGE":
                extra = (f"  (lane {r['current_lane']}→{r['target_lane']}, "
                         f"steer {r['steer_pwm']:+.2f}, "
                         f"front_clear={r['front_clear']}, "
                         f"dist={r['dist_front_cm']})")
            print(f"  {fmt_ms_of_day(r['t_abs_ms'])}  {prev} -> {b}{extra}")
            prev = b

    # 링크 상태 분포
    from collections import Counter
    link = Counter(r["link_state"] for r in records)
    print(f"\n[링크 상태 분포] {dict(link)}")


def main():
    ap = argparse.ArgumentParser(description="버스 로그(.buslog) 파싱·CSV 내보내기")
    ap.add_argument("path", help=".buslog 파일 경로")
    ap.add_argument("--csv", default=None, help="CSV 출력 경로 (기본: 입력과 같은 위치, .csv)")
    ap.add_argument("--no-csv", action="store_true", help="CSV 생성 없이 요약만")
    args = ap.parse_args()

    if not os.path.isfile(args.path):
        print(f"파일 없음: {args.path}", file=sys.stderr)
        sys.exit(1)

    header, records = read_file(args.path)
    summarize(header, records)

    if not args.no_csv:
        out = args.csv or (os.path.splitext(args.path)[0] + ".csv")
        to_csv(records, out)
        print(f"\n[parse] CSV: {out}")


if __name__ == "__main__":
    main()
