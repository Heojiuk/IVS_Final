# src/ — 평면 구조 (파일 = 담당)

> 폴더 없이 한 곳. **파일 이름이 곧 담당**이다. 자기 파일 하나만 열면 된다.

## 누가 어느 파일을 여는가 (6명)

| 담당 | 인원 | 파일 | 할 일 |
|---|---|---|---|
| 통신/인프라 | 1 | `comm.py` `bus.py` `scheduler.py` `config.py` `main.py` | V2V 송수신·버스·루프 (80% 완성) |
| 인지 | 2 | `perception.py` | 카메라(차선·YOLO)+초음파 → `scene` |
| 판단 | 1 | `decision.py` | scene·V2V → `command`·`mode` |
| 모션 | 2 | `motion.py` | command → 제어·구동(GPIO), `ego_state` |
| (공용) | 6 | `contracts.py` | 토픽 데이터 형식 — **모두가 보고, writer만 고침** |

- `contracts.py` 가 6명의 공통 약속(필드·타입·단위). 여기 정의된 dataclass만 버스로 주고받는다.
- `perception.py`·`motion.py` 는 2명이 같이 쓴다. 충돌이 잦아지면 그때 `perception_lane.py` / `perception_object.py` 로 쪼개라(지금은 단순하게 한 파일).

## 호출 구조
```
main()  →  build()  →  Scheduler.run()
                           │ 50ms 마다
                           ├─ perception.step(bus)
                           ├─ decision.step(bus)
                           ├─ motion.step(bus)
                           └─ comm.step(bus)        (송신 TX)
                       comm 의 수신(RX)은 별도 스레드 → 버스에 기록
```
main은 조립 후 스케줄러만 부르고, 스케줄러가 매 50ms 모듈 step()을 순차 호출.

## 버스 구조 (모듈 간 직접 호출 금지, 버스만 경유)
```
perception ──scene──▶ decision ──command──▶ motion ──ego_state──▶ comm
                         ▲                     ▲                    │
                         └── link_status ──────┴── leader_state ────┘
                                       (comm RX 스레드가 버스에 기록)
```
- 쓰기: `bus.publish(Topics.X, data)`   읽기: `bus.read(Topics.X)`
- **토픽은 `Topics`(bus.py)의 7종만**. 그 외 publish/read 시 거부 (DD-INF-01).
- 버스 read 는 초기 사이클에 `None` 일 수 있다 → **소비 측 None 가드 필수**.

| 토픽 | ICD | Writer → Reader |
|---|---|---|
| `perception/scene` | IF-B1 | 인지 → 판단·주행 |
| `decision/command` | IF-B2 | 판단 → 주행·통신 (behavior+target_lane) |
| `decision/mode` | IF-B3 | 판단 → 전 모듈 |
| `motion/ego_state` | IF-B4 | 주행 → 통신 |
| `v2v/leader_state` | IF-B5 | 통신 → 판단·주행 (후행 버스) |
| `v2v/follower_state` | IF-B5 | 통신 → 판단 (선행 버스) |
| `v2v/link_status` | IF-B6 | 통신 → 판단 |

## 실행 / 테스트
```
cd src
python main.py --role leader        # 또는 --role follower
python test_comm.py                 # STATE 코덱 왕복·위변조 테스트
```
로컬 1대에서 통신 테스트: 두 셸에서 `IVS_PEER_IP=127.0.0.1` 로 leader/follower 각각 실행.

## 시나리오별 모듈 수행 시퀀스 (매 50ms 사이클)

**S-1 군집 형성·지속 주행** — 차선 따라 자율주행
```
인지  차선 인식                ─scene→
판단  행동=추종                ─command, mode→
주행  Pure Pursuit·구동 PWM    ─ego_state→ (GPIO/PWM)
통신  STATE 송신
```

**S-2 V2V 추종** — 후행 차량
```
[rx]  V2V 수신                ─leader_state, link_status→ (버스 기록, 비동기)
인지  차선 인식                ─scene→
판단  read leader·link         ─command(행동=추종)→
주행  초음파 거리 보정(듀티)     ─ego_state→
통신  STATE 송신
```

**S-3 전방 장애물 회피·정지**
```
인지  YOLO+초음파 융합          ─scene(front_clear=false, objects)→
판단  회피/정지 + 추돌가드      ─command→
주행  회피 경로 / 정지 듀티      ─구동
```

**S-4 정지선 정지·재출발**
```
인지  정지선/STOP 검출          ─scene(stop_signal=true)→
판단  정지선 전 정지→재출발     ─command(정지→순항)→
주행  듀티 0 → (재출발 시) 순항
```

**안전 — 통신 두절 폴백**
```
통신  link 산출                ─link_status(두절 ≥500ms)→
판단  폴백(직전 명령 유지+서행) + 모드(서행) ─command, mode→
주행  서행 듀티
```

**안전 — 비상정지(ESTOP)**
```
판단  자체 정지 트리거 → 모드(비상정지) ─command, mode→
주행  즉시 듀티 0
```

## 파일
```
main.py        진입점 + 조립 (build → 스케줄러)
scheduler.py   50ms 루프 (DD-INF-03)
bus.py         메시지 버스 + 토픽 7종 (DD-INF-01)
config.py      포트·주기·링크임계값·PSK
contracts.py   토픽 데이터 형식 전부 (ICD IF-B1~B6)
perception.py  step(bus): 센서→scene            [인지]
decision.py    step(bus): scene→command·mode    [판단]
motion.py      step(bus): command→ego_state·구동 [모션]
comm.py        step(bus): ego_state→STATE 송신 + RX 스레드 [통신]
test_comm.py   STATE 코덱 테스트
```
