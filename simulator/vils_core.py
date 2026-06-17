"""VILSEngine: runs the 50ms scheduler loop using real src modules."""
import _src_path; _src_path.add()

from core_module.bus import MessageBus, Topics
from algorithm.decision import DecisionModule
from algorithm.motion_planning import MotionModule
from messages import Role
from logger import RecordableV2VModule


class VILSEngine:
    """Wraps src modules and runs one step per tick().

    role: 'follower' or 'leader' (simulator's role)
    on_packet_cb: called with raw 60B bytes for each verified RX packet
    """

    def __init__(self, role, on_packet_cb=None, on_hmac_fail_cb=None,
                 on_tx_cb=None, use_local_control=True):
        self._bus = MessageBus()
        self._v2v = RecordableV2VModule(role, on_packet_cb, on_hmac_fail_cb, on_tx_cb)
        # DecisionModule and MotionModule expect Role enum, not a string
        role_enum = Role.LEADER if role == "leader" else Role.FOLLOWER
        # 판단·모션: 로컬 추종 제어(기본) ↔ 실제 src 모듈(브리지). 타 팀 구현 완료 시 False 로 전환.
        self._use_local_control = use_local_control
        if use_local_control:
            from sim_algorithm.decision import LocalDecisionModule
            from sim_algorithm.motion_planning import LocalMotionModule
            self._decision = LocalDecisionModule(role_enum)
            self._motion = LocalMotionModule(role_enum)
        else:
            self._decision = DecisionModule(role_enum)   # 타 팀 STUB (현재 PWM 0)
            self._motion = MotionModule(role_enum)
        self._started = False
        self._sim_perception = None

    def set_record_callback(self, cb):
        self._v2v.set_record_callback(cb)

    def start(self, sim_perception):
        self._sim_perception = sim_perception
        self._v2v.start(self._bus)
        self._started = True

    def tick(self):
        if not self._started:
            return
        self._sim_perception.step(self._bus)
        self._decision.step(self._bus)
        self._motion.step(self._bus)
        self._v2v.step(self._bus)

    def stop(self):
        if self._started:
            self._v2v.stop()
            self._started = False

    def bus_snapshot(self):
        def safe_read(topic):
            try:
                return self._bus.read(topic)
            except Exception:
                return None
        return {
            'command':  safe_read(Topics.COMMAND),
            'mode':     safe_read(Topics.MODE),
            'ego':      safe_read(Topics.EGO_STATE),
            'pi_state': safe_read(Topics.LEADER_STATE) or safe_read(Topics.FOLLOWER_STATE),
            'link':     safe_read(Topics.LINK_STATUS),
        }
