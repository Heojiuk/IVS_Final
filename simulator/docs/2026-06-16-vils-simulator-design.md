# VILS Simulator 설계 문서

**날짜**: 2026-06-16  
**작업 디렉토리**: `simulator/`

---

## 1. 개요

실제 Pi 차량이 트랙을 주행하는 동안, PC가 V2V WiFi로 연결되어 상대 차량(leader 또는 follower)을 소프트웨어로 모사하는 **VILS(Vehicle-in-the-Loop Simulation)** 환경.

```
실제 Pi 차량 (트랙 주행)              Simulator PC
  카메라 → 인지 → 판단 → 모션           SimPerception(UI) → 판단 → 모션
              V2V WiFi ←──────────────────→ V2V UDP
```

- Pi = leader → PC가 follower 역할 소프트웨어 실행
- Pi = follower → PC가 leader 역할 소프트웨어 실행

---

## 2. 파일 구성

```
simulator/
├── app.py              — 메인 진입점, tkinter 탭 앱
├── sim_perception.py   — UI 값을 Scene 토픽으로 발행하는 가짜 인지 모듈
├── logger.py           — UDP 패킷 수신·저장 컴포넌트
├── playback.py         — bin 파일 재생 + 트랙 시각화 (탭으로 포함)
└── converter.py        — bin → CSV 후처리 CLI 유틸

data/
└── log/                — 녹화 세션 저장 루트
    └── 01_2026-06-16 143022_2026-06-16 143855/
        └── session.bin
```

`simulator/`는 `src/`의 decision, motion_planning, v2v, bus, config 모듈을 직접 import한다.  
`sys.path`에 `../src` 추가 방식으로 연결.

---

## 3. 60B 패킷 구조

`src/core_module/v2v.py`의 `packet_parser()` / `packet_generator()` 재사용.

| 필드 | 크기 | 설명 |
|------|------|------|
| ver  | 1B   | 버전 |
| type | 1B   | 패킷 종류 (STATE=1) |
| role | 1B   | LEADER=1 / FOLLOWER=2 |
| seq  | 2B   | 일련번호 |
| t_tx | 8B   | 송신 시각 (double, 추후 HH:mm:ss.fff로 교체 예정) |
| lane | 1B   | 현재 차로 (0/1/2) |
| rsv  | 1B   | reserved |
| behavior | 1B | DriveBehavior |
| rsv  | 1B   | reserved |
| throttle_pwm | 4B | -1.0 ~ 1.0 |
| steer_pwm    | 4B | -1.0 ~ 1.0 |
| rsv  | 3B   | reserved (절대시각 HH:mm:ss.fff 예정) |
| HMAC | 32B  | SHA-256 |
| **합계** | **60B** | |

> **주의**: `t_tx` 필드가 `HH:mm:ss.fff` 형식으로 변경되면 파서 동기화 필요.

---

## 4. 녹화 (logger.py)

### 폴더 명명 규칙

```
data/log/{idx:02d}_{first_rx_time}_{stop_time}/session.bin
예) data/log/01_2026-06-16 143022_2026-06-16 143855/session.bin
```

### 흐름

1. **Start 버튼 누름**
   - `data/log/` 스캔 → 최대 인덱스 + 1 계산
   - 임시 폴더 `{idx:02d}_{press_time}` 생성
   - TX 스레드(50ms dummy STATE 송신) + RX 스레드 시작

2. **첫 패킷 수신**
   - `first_rx_time` 메모리 저장
   - `session.bin` 쓰기 시작 (raw 60B 연속 기록)

3. **Stop 버튼 누름**
   - 스레드 정지, 파일 닫기
   - 폴더 → `{idx:02d}_{first_rx_time}_{stop_time}` rename

### TX 내용

VILS 모드에서는 `V2VModule.step()`이 버스의 `EgoState`를 자동으로 읽어 60B 패킷으로 송신한다.  
시뮬레이터의 Decision + MotionPlanning이 계산한 실제 throttle/steer 값이 Pi로 전달된다.  
50ms 주기 송신으로 Pi가 LINK_LOST 판정하지 않도록 유지.

---

## 5. app.py — 탭 구성

### 탭 구조

```
[ Follower | Leader ]   ← 역할 선택 탭
[ Real-time | Playback ]  ← 모드 선택 탭 (역할 탭 안에 포함)
```

- 역할(Follower/Leader) 선택 후 모드(Real-time/Playback) 선택
- Real-time: 실시간 데이터 수신 + 로깅 + 트랙 시각화
- Playback: bin 파일 선택 → 60B 파싱 → 트랙 시각화
- 역할 탭 전환 시 Real-time 모드가 활성화 중이면 전환 불가

### Tab 1: Follower / Tab 2: Leader

두 탭은 역할(role)만 다르고 UI 구조 동일.

#### Scene 제어판 (SimPerception 입력)

| 위젯 | 토픽 필드 | 범위 |
|------|-----------|------|
| 체크박스 | `lane_valid` | True/False |
| 드롭다운 | `current_lane` | 0 / 1 / 2 |
| 슬라이더 | `lane_offset_m` | -0.5 ~ 0.5 m |
| 슬라이더 | `lane_heading_rad` | -π/4 ~ π/4 |
| 슬라이더 | `lane_curvature_1pm` | -2.0 ~ 2.0 |
| 체크박스 | `front_clear` | True/False |
| 슬라이더 | `dist_front_m` | 0 ~ 5.0 m (None 포함) |
| 체크박스 | `stop_signal` | True/False |

#### 버스 모니터 (읽기 전용, 50ms 갱신)

- DriveCommand: behavior, target_lane
- ModeCmd: mode, cause
- EgoState: throttle_pwm, steer_pwm, behavior
- V2VState: 수신된 Pi 상태 (role, seq, lane, throttle, steer, behavior)
- LinkStatus: state, age_rx, last_seq

#### 녹화 상태
- 인덱스, 누적 패킷 수, 경과 시간, Start/Stop 버튼

### 실행 루프

`scheduler.py` 패턴 동일하게 50ms 주기:
```
SimPerception.step() → scene 발행
Decision.step()      → DriveCommand, ModeCmd 발행
MotionPlanning.step() → EgoState 발행
V2VModule.step()     → TX 송신 + LinkStatus 발행
UI 갱신()
```

탭 전환 시 녹화 중이면 전환 불가 (Stop 먼저 요구).

---

## 6. 트랙 시각화

### 트랙 치수 (실측)

| 항목 | 값 |
|------|-----|
| 센터라인 가로 | 2.5 m |
| 센터라인 세로 | 2.05 m |
| 차선 폭 | 20 cm × 2 |
| 라인 색상 | 외곽=초록, 중앙=노란, 내곽=초록 |

### 화면 표현

- 가로(landscape) 방향으로 렌더링
- 스케일: 화면 너비 기준 자동 fit (여백 포함)
- 3개 라인: 외곽 초록 타원, 노란 중간 타원, 내곽 초록 타원 (모두 rounded rectangle)

### 차량 표시

- Pi 차량 (실차): 파란 삼각형 (수신 패킷 기반)
- 시뮬레이터 차량: 빨간 삼각형 (EgoState 기반)
- 삼각형 방향 = 현재 헤딩

### 움직임 모델 (단순 비례)

```python
dt      = t_tx[i+1] - t_tx[i]          # 패킷 간 시간차 (초)
heading += steer_pwm * k_w * dt         # 헤딩 변화 (rad)
x       += throttle_pwm * k_v * cos(heading) * dt  # 픽셀 이동
y       += throttle_pwm * k_v * sin(heading) * dt
```

튜닝 슬라이더: `k_v` (속도 배율), `k_w` (회전 배율)

### 차량 형상 (Top-view)

삼각형 기반 간단 형상:
- Pi 차량 (실차): 파란 삼각형
- 시뮬레이터 차량: 빨간 삼각형
- 삼각형 꼭짓점 방향 = 현재 헤딩

### 모드

**Real-time 모드**
- 수신 패킷 실시간 파싱 → 두 차량 위치 즉시 갱신
- 동시에 `session.bin` 로깅 (4절 흐름과 동일)
- Scene 제어판 + 버스 모니터 + 트랙 뷰 한 화면에 표시
- 마우스 클릭으로 시작 위치 지정 후 Start

**Playback 모드**
- 파일 선택 다이얼로그 → `data/log/` 기본 경로에서 `session.bin` 선택
- 60B 단위로 전체 파싱 후 패킷 리스트 로드
- 트랙 위에 시뮬레이션 재생 (Play/Pause, 속도 1×/2×/4×)
- 마우스 클릭으로 시작 위치 지정 후 Play

공통: 초기 헤딩 = 오른쪽(0°)

---

## 7. converter.py (CLI)

```bash
python converter.py session.bin [output.csv]
```

출력 CSV 컬럼: `seq, t_tx, role, lane, behavior, throttle_pwm, steer_pwm`

HMAC 검증 후 파싱 (키 필요). `--no-verify` 플래그로 검증 스킵 가능.

---

## 8. 네트워크 설정

- Pi의 `peer_ip` → PC IP로 설정 (기존 config.py 활용)
- HMAC 키: v2v.py와 동일 PSK 공유
- 포트: 역할별로 `config.for_role()` 결과 그대로 사용

---

## 9. 미확정 / 추후 작업

- `t_tx` → `HH:mm:ss.fff` 포맷 변경 시 파서 동기화 (v2v.py 수정 대기)
- 통신 끊김(LINK_LOST) 감지 시 자동 Stop 처리 (현재는 수동 Stop)
- CSV 후처리 시각화 (외부 도구 연계)
