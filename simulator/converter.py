"""bin → CSV post-processor.

Usage:
    python converter.py session.bin [output.csv] [--no-verify]
"""
import os, sys, csv, argparse, struct
import _src_path; _src_path.add()

from core_module.v2v import packet_parser, PACKET_LEN, fmt_ms_of_day
from core_module import config
from messages import Role, DriveBehavior, V2VState

COLUMNS = ['seq', 'tx_abs', 'tx_time', 'role', 'lane', 'behavior', 'throttle_pwm', 'steer_pwm']
_BODY_FMT = '!BBBHdIBBffx'
_BODY_LEN = struct.calcsize(_BODY_FMT)


def convert_bin_to_csv(bin_path, csv_path, key=None, verify=True):
    """Parse bin_path (raw 60B packets), write csv_path. Returns count of valid packets."""
    if key is None:
        key = config.load_key()

    count = 0
    with open(bin_path, 'rb') as fin, open(csv_path, 'w', newline='') as fout:
        writer = csv.DictWriter(fout, fieldnames=COLUMNS)
        writer.writeheader()
        while True:
            raw = fin.read(PACKET_LEN)
            if len(raw) < PACKET_LEN:
                break
            try:
                if verify:
                    state = packet_parser(raw, key)
                else:
                    _, _, role, seq, t, tx_abs, lane, beh, thr, st = struct.unpack(_BODY_FMT, raw[:_BODY_LEN])
                    state = V2VState(t_tx=t, tx_abs=tx_abs, role=Role(role), seq=seq,
                                     lane=lane, throttle_pwm=thr, steer_pwm=st,
                                     behavior=DriveBehavior(beh))
            except Exception:
                continue
            writer.writerow({
                'seq': state.seq,
                'tx_abs': state.tx_abs,
                'tx_time': fmt_ms_of_day(state.tx_abs),
                'role': state.role.name,
                'lane': state.lane,
                'behavior': state.behavior.name,
                'throttle_pwm': f'{state.throttle_pwm:.6f}',
                'steer_pwm': f'{state.steer_pwm:.6f}',
            })
            count += 1
    return count


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Convert session.bin to CSV')
    parser.add_argument('bin_file')
    parser.add_argument('csv_file', nargs='?')
    parser.add_argument('--no-verify', action='store_true')
    args = parser.parse_args()

    out = args.csv_file or args.bin_file.replace('.bin', '.csv')
    n = convert_bin_to_csv(args.bin_file, out, verify=not args.no_verify)
    print(f'Written {n} packets → {out}')
