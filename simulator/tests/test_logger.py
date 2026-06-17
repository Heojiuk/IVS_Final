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


def _pkt(role):
    """role 바이트(offset 2)만 채운 더미 60B 패킷."""
    return bytes([1, 1, role]) + bytes(57)


def test_session_role_streams_and_rename(tmp_path):
    rec = SessionRecorder(str(tmp_path))
    rec.start('Simulator')

    rec.log(_pkt(1), is_pc=True)    # leader (자기 PC)  → leader_pc
    rec.log(_pkt(1), is_pc=True)
    rec.log(_pkt(2), is_pc=False)   # follower (실차)   → follower

    names = rec.stop('2026-06-16 140100')
    mode_root = os.path.join(str(tmp_path), 'Simulator')

    # 역할별 폴더 2개, 각 폴더 안에 동일명 .bin
    by_role = {}
    for nm in names:
        folder = os.path.join(mode_root, nm)
        binp = os.path.join(folder, nm + '.bin')
        assert os.path.isdir(folder), nm
        assert os.path.exists(binp), binp
        by_role[nm.split('_')[-1] if not nm.endswith('_pc') else 'pc'] = binp

    # leader_pc 폴더: 2 × 60B, follower 폴더: 1 × 60B
    leader_pc = [n for n in names if n.endswith('_leader_pc')]
    follower  = [n for n in names if n.endswith('_follower')]
    assert len(leader_pc) == 1 and len(follower) == 1, names
    assert os.path.getsize(os.path.join(mode_root, leader_pc[0], leader_pc[0] + '.bin')) == 120
    assert os.path.getsize(os.path.join(mode_root, follower[0], follower[0] + '.bin')) == 60
    # 이름 형식: 01_<시작>_2026-06-16 140100_<role>[_pc]
    assert leader_pc[0].startswith('01_') and '2026-06-16 140100' in leader_pc[0]


if __name__ == '__main__':
    import tempfile, pathlib
    with tempfile.TemporaryDirectory() as d:
        base = pathlib.Path(d)
        test_next_index_empty(base / 'test1')
        test_next_index_with_existing(base / 'test2')
        test_session_role_streams_and_rename(base / 'test3')
    print('OK')
