import os, sys, struct, tempfile, shutil
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import _src_path; _src_path.add()

from logger import SessionRecorder, next_index

LOG_ROOT = os.path.join(os.path.dirname(__file__), '..', '..', 'data', 'log')


def test_next_index_empty(tmp_path):
    assert next_index(str(tmp_path)) == 1


def test_next_index_with_existing(tmp_path):
    os.makedirs(os.path.join(tmp_path, '03_2026-06-16 100000'))
    os.makedirs(os.path.join(tmp_path, '07_2026-06-16 110000_2026-06-16 110500'))
    assert next_index(str(tmp_path)) == 8


def test_session_writes_bin_and_renames(tmp_path):
    rec = SessionRecorder(str(tmp_path))
    rec.start()

    dummy_60b = bytes(60)
    rec.on_packet(dummy_60b, '2026-06-16 140000')  # triggers first_rx_time
    rec.on_packet(dummy_60b, '2026-06-16 140000')

    folder, path = rec.stop('2026-06-16 140100')
    assert os.path.exists(path)
    assert os.path.getsize(path) == 120  # 2 × 60B
    assert '01_2026-06-16 140000_2026-06-16 140100' in folder


if __name__ == '__main__':
    import tempfile, pathlib
    with tempfile.TemporaryDirectory() as d:
        base = pathlib.Path(d)
        test_next_index_empty(base / 'test1')
        test_next_index_with_existing(base / 'test2')
        test_session_writes_bin_and_renames(base / 'test3')
    print('OK')
