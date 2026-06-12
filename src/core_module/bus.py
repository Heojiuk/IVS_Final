"""메시지 버스 — 공유메모리 pub/sub (DD-INF-01, ICD IF-B1~B6)

모듈은 정의된 토픽(Topics)에만 publish/read 한다. 모듈 간 직접 호출 금지의 유일한 통로.
단일 프로세스 공유메모리 + threading.Lock. 데이터 형식은 contracts.py 참조.
"""
import threading


class Topics:
    """버스 토픽 정의 — 정의된 토픽만 사용 가능 (ICD 3장)"""
    SCENE = "perception/scene"             # IF-B1 인지 결과   (인지 → 판단·주행)
    COMMAND = "decision/command"           # IF-B2 주행 명령   (판단 → 주행·통신)
    MODE = "decision/mode"                 # IF-B3 시스템 모드 (판단 → 전 모듈)
    EGO_STATE = "motion/ego_state"         # IF-B4 자차 상태   (주행 → 통신)
    LEADER_STATE = "v2v/leader_state"      # IF-B5 선행차 상태 (통신 → 판단·주행, 후행 버스)
    FOLLOWER_STATE = "v2v/follower_state"  # IF-B5 후행차 상태 (통신 → 판단, 선행 버스)
    LINK_STATUS = "v2v/link_status"        # IF-B6 링크 상태   (통신 → 판단)


# 버스가 인식하는 전체 토픽 (이 외 토픽 publish/read 시 거부)
ALL_TOPICS = [
    Topics.SCENE, Topics.COMMAND, Topics.MODE, Topics.EGO_STATE,
    Topics.LEADER_STATE, Topics.FOLLOWER_STATE, Topics.LINK_STATUS,
]


class MessageBus:
    def __init__(self):
        """버스 생성 — 정의된 토픽마다 빈(None) 슬롯을 만든다.  파라미터 없음"""
        self._lock = threading.Lock()
        self._data = {t: None for t in ALL_TOPICS}   # 정의된 토픽 슬롯만 생성

    def publish(self, topic, data):
        """토픽 슬롯에 최신값을 덮어쓴다(미정의 토픽이면 KeyError).  topic=토픽명(Topics 상수), data=그 토픽의 dataclass"""
        if topic not in self._data:
            raise KeyError(f"미정의 토픽: {topic}")    # 스키마 계약 (DD-INF-01)
        with self._lock:
            self._data[topic] = data                  # 최신값 덮어쓰기

    def read(self, topic):
        """토픽의 최신값을 반환한다(아직 없으면 None, 미정의 토픽이면 KeyError).  topic=토픽명(Topics 상수)"""
        if topic not in self._data:
            raise KeyError(f"미정의 토픽: {topic}")
        with self._lock:
            return self._data[topic]                  # 없으면 None
