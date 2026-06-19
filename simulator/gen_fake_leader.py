"""가상 leader 주행 데이터 생성기.

시뮬레이터 모션 모델(apply_motion_model)과 동일한 공식으로 역산:
  steer_pwm  = Δheading / (K_W * DT)
  throttle_pwm = arc_length / (K_V * DT)

K_V, K_W 는 시뮬레이터 기본값(1.0 / 2.0)과 일치해야 트랙을 정확히 따라감.
"""
import sys, os, math, time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from core_module.v2v import packet_generator
from core_module.config import load_key
from messages import EgoState, DriveBehavior, Role

# ── 설정 ──────────────────────────────────────────────────────────────
LOG_ROOT = os.path.join(os.path.dirname(__file__), '..', 'data', 'log')
N_LAPS   = 3
HZ       = 20
PER_LAP  = 200          # 패킷/랩 (10초 × 20Hz)
DT       = 1.0 / HZ    # 0.05 s

# 트랙 중심선 (노란 선)
RX, RY = 1.25, 1.025

# 시뮬레이터 기본 모션 모델 파라미터
K_V = 1.0
K_W = 2.0


def _oval_heading(theta):
    """반시계방향 접선 방향(rad). dx/dθ = -RX·sin θ, dy/dθ = RY·cos θ"""
    return math.atan2(RY * math.cos(theta), -RX * math.sin(theta))


def _angle_diff(a, b):
    """a - b를 [-π, π] 범위로 정규화"""
    d = a - b
    while d >  math.pi: d -= 2 * math.pi
    while d < -math.pi: d += 2 * math.pi
    return d


# ── 최신 로그 폴더 찾기 ────────────────────────────────────────────────
folders = sorted(
    f for f in os.listdir(LOG_ROOT)
    if os.path.isdir(os.path.join(LOG_ROOT, f))
)
if not folders:
    sys.exit("로그 폴더 없음")

latest_dir = os.path.join(LOG_ROOT, folders[-1])
out_path   = os.path.join(latest_dir, 'leader_fake.bin')
print(f"생성 대상: {out_path}")

# ── θ 배열 미리 계산 (N+1개: 스티어 계산에 i+1 필요) ──────────────────
N      = N_LAPS * PER_LAP
dtheta = 2 * math.pi / PER_LAP
thetas   = [-math.pi / 2 + i * dtheta for i in range(N + 1)]
headings = [_oval_heading(t) for t in thetas]

key    = load_key()
t_base = time.monotonic()

with open(out_path, 'wb') as fh:
    for i in range(N):
        theta = thetas[i]

        # 호 길이 → throttle_pwm
        ds = math.hypot(-RX * math.sin(theta) * dtheta,
                         RY * math.cos(theta) * dtheta)
        throttle_val = max(0.0, min(1.0, ds / (K_V * DT)))

        # 헤딩 변화 → steer_pwm  (규약: 음수=좌회전/양수=우회전, 모션모델 heading-=steer·k_w·dt 와 일관)
        dh = _angle_diff(headings[i + 1], headings[i])
        steer_val = max(-1.0, min(1.0, -dh / (K_W * DT)))

        ego = EgoState(
            stamp        = t_base + i / HZ,
            throttle_pwm = throttle_val,
            steer_pwm    = steer_val,
            behavior     = DriveBehavior.CRUISE,
        )
        pkt = packet_generator(ego, lane=1, role=Role.LEADER, seq=i & 0xFFFF, key=key)
        fh.write(pkt)

print(f"완료: {N}패킷 ({N * 60}B)")
print(f"  throttle 범위: {min(ds/(K_V*DT) for ds in [math.hypot(-RX*math.sin(t)*dtheta, RY*math.cos(t)*dtheta) for t in thetas[:-1]]):.3f} ~ "
      f"{max(ds/(K_V*DT) for ds in [math.hypot(-RX*math.sin(t)*dtheta, RY*math.cos(t)*dtheta) for t in thetas[:-1]]):.3f}")
