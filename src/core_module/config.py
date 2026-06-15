"""설정 — 주기·링크 임계값·V2V 포트/주소·PSK. 운영값은 여기 한 곳에서만 바꾼다.

IP 모드: 환경변수 IVS_MODE 로 선택 (기본 release)
  release  = 문서 대역 192.168.0.x        (실차 배포)
  dev      = 강의실 WiFi 에 연결된 Pi 실측 IP (원격 개발·테스트)
  loopback = 둘 다 127.0.0.1              (단일 PC 2프로세스 테스트)
포트는 모드 무관 동일. peer_ip = 현재 모드의 '상대 역할' IP.
"""

import os
import sys

# ── 루프/링크 ─────────────────────────────────────────────────────────
LOOP_PERIOD_S = 0.05  # 20Hz 주기 (DD-INF-03)
LINK_STALE_MS = 200.0  # 이 이상 미수신 → STALE
LINK_LOST_MS = 500.0  # 이 이상 미수신 → LOST (안전 폴백 트리거)

# ── 역할별 V2V 포트 (모드 무관) ────────────────────────────────────────
# 쌍 규칙: leader.peer_port == follower.rx_port, leader.rx_port == follower.peer_port
_PORTS = {
    "leader": {"rx_port": 5006, "peer_port": 5005},
    "follower": {"rx_port": 5005, "peer_port": 5006},
}

# ── 모드별 차량 IP ─────────────────────────────────────────────────────
_IPS = {
    "release": {"leader": "192.168.0.11", "follower": "192.168.0.12"},      # 문서 대역(실차)
    "dev": {"leader": "192.168.202.91", "follower": "192.168.203.237"},     # 강의실 WiFi (실측 — 실제 Pi 배정에 맞게 확인, 다르면 두 줄 swap)
    "loopback": {"leader": "127.0.0.1", "follower": "127.0.0.1"},           # 단일 PC 테스트
}

_DEV_KEY = b"DEVKEY-INSECURE-CHANGE-ME-32BYTE"  # 32B 개발용. 운영 전 psk.key 배포


def mode() -> str:
    """현재 IP 모드 (환경변수 IVS_MODE, 기본 release).  파라미터 없음"""
    m = os.environ.get("IVS_MODE", "release").lower()
    if m not in _IPS:
        raise ValueError(f"unknown IVS_MODE={m!r} (use release|dev|loopback)")
    return m


def for_role(role: str) -> dict:
    """역할별 V2V 설정 {rx_port, peer_port, peer_ip} 반환. peer_ip = 현재 모드의 '상대 역할' IP.  role='leader'|'follower'"""
    if role not in _PORTS:
        raise ValueError(f"unsupported role: {role} (leader|follower)")
    ips = _IPS[mode()]
    peer = "follower" if role == "leader" else "leader"  # 상대 역할
    return {
        "rx_port": _PORTS[role]["rx_port"],
        "peer_port": _PORTS[role]["peer_port"],
        "peer_ip": ips[peer],
    }


def load_key() -> bytes:
    """HMAC 사전공유키(32B)를 반환한다 — 프로젝트 루트 psk.key(32바이트 바이너리) 있으면 사용, 없으면 개발용 기본키(경고).  파라미터 없음

    운영 전 psk.key 를 각 차량에 동일하게 배포(형상관리 제외 — .gitignore).
    """
    root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # config.py → 프로젝트 루트
    path = os.path.join(root, "psk.key")
    if os.path.exists(path):
        key = open(path, "rb").read()  # 32바이트 바이너리 (strip 금지 — 공백바이트 키 손상 방지)
        if len(key) != 32:  # ICD: PSK 32B
            raise ValueError(f"psk.key length {len(key)}B != 32B")
        return key
    print(
        "[config] WARNING: psk.key not found -> using INSECURE dev key. "
        "Deploy a real key (same on both vehicles) before operation.",
        file=sys.stderr,
    )
    return _DEV_KEY
