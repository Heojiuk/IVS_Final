# src/ — core_module(엔진+통신) + algorithm(인지·판단·모션)

> **`core_module/`** = 엔진(버스·루프·설정) + 통신(v2v) — 통신/인프라 1명. **`algorithm/`** = step() 도는 주행 로직(인지·판단·모션).

```
src/
  main.py                          진입점 — 실행은 이것만 (python main.py --role leader)
  contracts.py                     공용 토픽 계약 (6명 공통, ICD와 1:1)
  core_module/                     엔진 + 통신 (통신/인프라 1명)
    bus.py  scheduler.py  config.py   엔진(버스·루프·설정)
    v2v.py                         통신 V2V 송수신
  algorithm/                       주행 알고리즘 = 매 50ms step() 도는 모듈
    perception.py                  인지 (2명)
    decision.py                    판단 (1명)
    motion_planning.py             모션 (2명)
  tests/test_v2v.py
```

## 누가 어느 파일을 여는가 (6명)

| 담당 | 인원 | 파일 | 할 일 |
|---|---|---|---|
| 통신/인프라 | 1 | `core_module/` + `main.py` | V2V 송수신·버스·루프 (80% 완성) |
| 인지 | 2 | `algorithm/perception.py` | 카메라(차선·YOLO)+초음파 → `scene` |
| 판단 | 1 | `algorithm/decision.py` | scene·V2V → `command`·`mode` |
| 모션 | 2 | `algorithm/motion_planning.py` | command → 제어·구동(GPIO), `ego_state` |
| (공용) | 6 | `contracts.py` | 토픽 데이터 형식 — **모두가 보고, writer만 고침** |

- `core_module/` = 엔진(bus·scheduler·config) + 통신(v2v) 한 사람 소유. `algorithm/` = 인지·판단·모션 주행 로직.
- `contracts.py` 가 6명의 공통 약속(필드·타입·단위). 여기 정의된 dataclass만 버스로 주고받는다.
- 한 모듈이 2명(인지·모션)이면 충돌 잦을 때 그 모듈만 폴더로 쪼개라(예: `algorithm/perception/lane.py`·`object.py`).

## 호출 구조
```
main()  →  build()  →  Scheduler.run()
                           │ 50ms 마다
                           ├─ perception.step(bus)
                           ├─ decision.step(bus)
                           ├─ motion.step(bus)
                           └─ v2v.step(bus)         (송신 TX)
                       v2v 의 수신(RX)은 별도 스레드 → 버스에 기록
```
main은 조립 후 스케줄러만 부르고, 스케줄러가 매 50ms 모듈 step()을 순차 호출.

## 버스 구조 (모듈 간 직접 호출 금지, 버스만 경유)
```
perception ──scene──▶ decision ──command──▶ motion ──ego_state──▶ v2v
                         ▲                     ▲                    │
                         └── link_status ──────┴── leader_state ────┘
                                       (v2v RX 스레드가 버스에 기록)
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
python tests/test_v2v.py            # STATE 코덱 왕복·위변조 테스트
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
main.py                       진입점 + 조립 (build → 스케줄러). 실행은 이것만.
contracts.py                  토픽 데이터 형식 전부 (ICD IF-B1~B6) — 공용
core_module/bus.py            메시지 버스 + 토픽 7종 (DD-INF-01)
core_module/scheduler.py      50ms 루프 (DD-INF-03)
core_module/config.py         포트·주기·링크임계값·PSK (DD-INF-02)
core_module/v2v.py            step(bus): ego_state→STATE 송신 + RX 스레드 [통신]
algorithm/perception.py       step(bus): 센서→scene            [인지]
algorithm/decision.py         step(bus): scene→command·mode    [판단]
algorithm/motion_planning.py  step(bus): command→ego_state·구동 [모션]
tests/test_v2v.py             STATE 코덱 테스트
```
