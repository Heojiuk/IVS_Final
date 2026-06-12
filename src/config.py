"""설정 — 주기·링크 임계값·V2V 포트/주소·PSK. 운영값은 여기 한 곳에서만 바꾼다."""
import os

# ── 루프/링크 ─────────────────────────────────────────────────────────
LOOP_PERIOD_S = 0.05           # 20Hz 주기 (DD-INF-03)
LINK_STALE_MS = 200.0          # 이 이상 미수신 → STALE
LINK_LOST_MS = 500.0           # 이 이상 미수신 → LOST (안전 폴백 트리거)

# ── 역할별 V2V 포트/주소 ──────────────────────────────────────────────
# 쌍 규칙: leader.peer_port == follower.rx_port, leader.rx_port == follower.peer_port
_ROLES = {
    "leader":   {"rx_port": 5006, "peer_port": 5005, "peer_ip": "192.168.0.12"},
    "follower": {"rx_port": 5005, "peer_port": 5006, "peer_ip": "192.168.0.11"},
}


def for_role(role: str) -> dict:
    """역할 설정 사본. 로컬 테스트는 IVS_PEER_IP=127.0.0.1 로 단일 PC 2프로세스 통신."""
    if role not in _ROLES:
        raise ValueError(f"미지원 역할: {role} (leader|follower)")
    cfg = dict(_ROLES[role])
    cfg["peer_ip"] = os.environ.get("IVS_PEER_IP", cfg["peer_ip"])
    return cfg


def load_key() -> bytes:
    """HMAC 사전공유키. src/psk.key 있으면 사용, 없으면 개발용 기본키.

    운영 전 psk.key 를 각 차량에 동일하게 배포(형상관리 제외 — .gitignore).
    """
    path = os.path.join(os.path.dirname(__file__), "psk.key")
    if os.path.exists(path):
        key = open(path, "rb").read().strip()
        if len(key) != 32:                      # ICD: PSK 32B
            raise ValueError(f"psk.key 길이 {len(key)}B ≠ 32B")
        return key
    return b"DEVKEY-INSECURE-CHANGE-ME-32BYTE"   # 32B 개발용 (ICD PSK 32B). 운영 전 psk.key 배포
