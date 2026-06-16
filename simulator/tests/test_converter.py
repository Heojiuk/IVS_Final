import os, sys, csv, tempfile, struct
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import _src_path; _src_path.add()

from core_module.v2v import packet_generator, PACKET_LEN
from messages import EgoState, Role, DriveBehavior
from converter import convert_bin_to_csv

KEY = b'test-key-32-bytes-padded-with-xx'


def _make_bin(tmp_path, n=3):
    path = os.path.join(tmp_path, 'session.bin')
    with open(path, 'wb') as f:
        for i in range(n):
            ego = EgoState(stamp=float(i), throttle_pwm=0.1*i, steer_pwm=-0.05*i, behavior=DriveBehavior.FOLLOW)
            f.write(packet_generator(ego, lane=1, role=Role.LEADER, seq=i, key=KEY))
    return path


def test_convert_produces_csv(tmp_path):
    bin_path = _make_bin(tmp_path)
    csv_path = os.path.join(tmp_path, 'out.csv')
    count = convert_bin_to_csv(bin_path, csv_path, key=KEY)
    assert count == 3
    with open(csv_path) as f:
        rows = list(csv.DictReader(f))
    assert len(rows) == 3
    assert rows[0]['seq'] == '0'
    assert abs(float(rows[2]['throttle_pwm']) - 0.2) < 1e-5


def test_convert_skips_bad_packets(tmp_path):
    bin_path = os.path.join(tmp_path, 'bad.bin')
    with open(bin_path, 'wb') as f:
        ego = EgoState(stamp=1.0, throttle_pwm=0.5, behavior=DriveBehavior.FOLLOW)
        f.write(packet_generator(ego, 1, Role.LEADER, 1, KEY))
        f.write(bytes(60))  # garbage packet
    count = convert_bin_to_csv(bin_path, os.path.join(tmp_path, 'out.csv'), key=KEY)
    assert count == 1


if __name__ == '__main__':
    import tempfile
    with tempfile.TemporaryDirectory() as d:
        test_convert_produces_csv(d)
        test_convert_skips_bad_packets(d)
    print('OK')
