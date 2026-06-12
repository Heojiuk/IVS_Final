"""STATE 코덱 왕복 + HMAC 위변조 검출. 통신팀이 패킷 포맷을 검증하는 예시 테스트.

실행:  cd src && python test_comm.py        (pytest 없이도 동작)
       cd src && python -m pytest test_comm.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))   # src 어디서 실행하든 import 가능

from contracts import EgoState, Role, DriveBehavior   # noqa: E402
from comm import pack_state, unpack_state, PACKET_LEN  # noqa: E402

KEY = b"test-key"


def test_roundtrip():
    ego = EgoState(stamp=12.5, throttle_pwm=0.3, steer_pwm=-0.2, behavior=DriveBehavior.STOP)
    pkt = pack_state(ego, Role.LEADER, 7, KEY)
    assert len(pkt) == PACKET_LEN
    st = unpack_state(pkt, KEY)
    assert st.seq == 7 and st.role == Role.LEADER
    assert st.behavior == DriveBehavior.STOP
    assert abs(st.throttle_pwm - 0.3) < 1e-6
    assert abs(st.steer_pwm - (-0.2)) < 1e-6


def test_tamper_rejected():
    pkt = bytearray(pack_state(EgoState(throttle_pwm=0.5, behavior=DriveBehavior.FOLLOW), Role.FOLLOWER, 1, KEY))
    pkt[9] ^= 0xFF                              # 본문 1바이트 변조
    try:
        unpack_state(bytes(pkt), KEY)
        assert False, "변조 패킷이 통과됨"
    except ValueError:
        pass


if __name__ == "__main__":
    test_roundtrip()
    test_tamper_rejected()
    print("OK: comm 코덱 왕복/위변조 테스트 통과")
