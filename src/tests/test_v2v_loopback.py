"""V2V 통신 프로토콜 End-to-End 검증 — 선행/후행 2노드 UDP 루프백.

test_v2v.py(코덱 단위)와 달리, 실제 소켓으로 STATE를 주고받아
상대차 상태 수신(leader/follower_state)·link_status·HMAC 거부까지 통합 검증한다.

실행:  cd src && python tests/test_v2v_loopback.py
"""
import os
import sys
import time

os.environ["IVS_MODE"] = "loopback"   # 단일 PC 루프백 (V2VModule 생성 전에 설정해야 반영)
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from core_module.bus import MessageBus, Topics              # noqa: E402
from core_module.v2v import V2VModule, packet_generator      # noqa: E402
from core_module import config                                # noqa: E402
from messages import EgoState, Scene, DriveBehavior, LinkState, Role  # noqa: E402


def test_loopback_exchange():
    """선행↔후행이 실제 UDP로 STATE를 주고받아 peer_state·link_status가 채워지는지."""
    lead_bus, foll_bus = MessageBus(), MessageBus()
    lead, foll = V2VModule("leader"), V2VModule("follower")

    # 각 노드가 송신할 자차 상태를 버스에 올림 (없으면 step()이 송신 안 함)
    lead_bus.publish(Topics.EGO_STATE,
                     EgoState(stamp=1.0, throttle_pwm=0.42, steer_pwm=-0.15, behavior=DriveBehavior.CRUISE))
    foll_bus.publish(Topics.EGO_STATE,
                     EgoState(stamp=2.0, throttle_pwm=0.30, steer_pwm=0.05, behavior=DriveBehavior.SLOW))
    lead_bus.publish(Topics.SCENE, Scene(current_lane=1))   # 선행 1차로 → 패킷에 실림
    foll_bus.publish(Topics.SCENE, Scene(current_lane=2))   # 후행 2차로

    lead.start(lead_bus)
    foll.start(foll_bus)
    try:
        for _ in range(6):                 # 6 사이클 송신, RX 스레드가 비동기 수신
            lead.step(lead_bus)
            foll.step(foll_bus)
            time.sleep(0.05)
        time.sleep(0.2)                     # 마지막 패킷 수신 여유

        fs = lead_bus.read(Topics.FOLLOWER_STATE)   # 선행이 받은 후행 상태
        ls = foll_bus.read(Topics.LEADER_STATE)     # 후행이 받은 선행 상태
        lead_link = lead_bus.read(Topics.LINK_STATUS)
        foll_link = foll_bus.read(Topics.LINK_STATUS)

        assert ls is not None, "후행이 선행 STATE 미수신"
        assert fs is not None, "선행이 후행 STATE 미수신"
        assert ls.role == Role.LEADER and abs(ls.throttle_pwm - 0.42) < 1e-6 and ls.behavior == DriveBehavior.CRUISE and ls.lane == 1
        assert fs.role == Role.FOLLOWER and abs(fs.throttle_pwm - 0.30) < 1e-6 and fs.behavior == DriveBehavior.SLOW and fs.lane == 2
        assert ls.t_rx > 0 and fs.t_rx > 0, "수신시각 t_rx 미기록"
        assert ls.seq >= 1 and fs.seq >= 1, "seq 미증가"
        assert lead_link.state == LinkState.ALIVE and foll_link.state == LinkState.ALIVE, "링크 ALIVE 아님"

        print(f"  후행←선행 수신: lane={ls.lane} throttle={ls.throttle_pwm} steer={ls.steer_pwm} behavior={ls.behavior.name} seq={ls.seq} t_rx={'set' if ls.t_rx else 'X'}")
        print(f"  선행←후행 수신: lane={fs.lane} throttle={fs.throttle_pwm} steer={fs.steer_pwm} behavior={fs.behavior.name} seq={fs.seq} t_rx={'set' if fs.t_rx else 'X'}")
        print(f"  링크상태: 선행={lead_link.state.name} 후행={foll_link.state.name}")
    finally:
        lead.stop()
        foll.stop()


def test_wrong_key_rejected():
    """잘못된 키로 만든 패킷은 HMAC 불일치로 폐기 → peer_state 미갱신."""
    import socket
    bus = MessageBus()
    node = V2VModule("leader")   # rx_port=5006, rx_topic=FOLLOWER_STATE
    node.start(bus)
    try:
        bad = packet_generator(EgoState(throttle_pwm=0.9, behavior=DriveBehavior.STOP),
                               1, Role.FOLLOWER, 1, b"WRONG-KEY-not-the-real-psk-32!!!")
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        for _ in range(3):
            s.sendto(bad, ("127.0.0.1", config.for_role("leader")["rx_port"]))
            time.sleep(0.05)
        s.close()
        time.sleep(0.2)
        assert bus.read(Topics.FOLLOWER_STATE) is None, "위조(키 불일치) 패킷이 수용됨!"
        print("  위조 패킷(키 불일치) 폐기 확인 — FOLLOWER_STATE 미갱신")
    finally:
        node.stop()


if __name__ == "__main__":
    test_loopback_exchange()
    test_wrong_key_rejected()
    print("OK: V2V 통신 프로토콜 E2E 검증 통과 (2노드 송수신·peer_state·link·HMAC거부)")
