"""
Member 5 — pytest test suite
Tests core logic without requiring a live ROS2 network.
Run: pytest tests/test_intersection.py -v
"""

import sys, os, types, math

# ── ROS2 stub setup ──────────────────────────────────────────────────────── #
def _stub(name):
    m = types.ModuleType(name)
    sys.modules[name] = m
    return m

_rclpy        = _stub('rclpy')
_node         = _stub('rclpy.node')
_cbg          = _stub('rclpy.callback_groups')
_stub('geometry_msgs'); _geom_msg = _stub('geometry_msgs.msg')
_stub('std_msgs');      _std_msg  = _stub('std_msgs.msg')
_stub('std_srvs');      _srvs     = _stub('std_srvs.srv')
_stub('visualization_msgs'); _viz = _stub('visualization_msgs.msg')

class _Logger:
    def info(self,*a,**k): pass
    def warn(self,*a,**k): pass
    def error(self,*a,**k): pass

class _FakePub:
    published = []
    def publish(self,msg): _FakePub.published.append(msg)

class _FakeTimer:
    def cancel(self): pass

class _FakeClient:
    def service_is_ready(self): return False

class _FakeNode:
    def get_logger(self): return _Logger()
    def get_clock(self): return type('C',(),{'now': lambda s: type('N',(),{'to_msg': lambda s: None})()})()
    def create_publisher(self,*a,**k): return _FakePub()
    def create_subscription(self,*a,**k): pass
    def create_service(self,*a,**k): pass
    def create_client(self,*a,**k): return _FakeClient()
    def create_timer(self,*a,**k): return _FakeTimer()

_node.Node = _FakeNode
_cbg.ReentrantCallbackGroup = type('RCG',(),{})
_rclpy.init = lambda **kw: None
_rclpy.spin = lambda n: None
_rclpy.shutdown = lambda: None
_rclpy.node = _node
_rclpy.callback_groups = _cbg

# Stub message types
def _msg_cls(name, **defaults):
    return type(name, (), defaults)

_P = lambda: type('Pos',(),{'x':0.0,'y':0.0,'z':0.0})()
_O = lambda: type('Ori',(),{'w':1.0})()
_H = lambda: type('Hdr',(),{'frame_id':'','stamp':None})()
_Pose = lambda: type('Pose',(),{'position':_P(),'orientation':_O()})()

class PoseStamped:
    def __init__(self): self.header=_H(); self.pose=_Pose()
class PoseArray:
    def __init__(self): self.header=_H(); self.poses=[]
class Pose:
    def __init__(self): self.position=_P(); self.orientation=_O()
class Twist:
    def __init__(self): self.linear=_P(); self.angular=_P()

_geom_msg.PoseStamped = PoseStamped
_geom_msg.PoseArray   = PoseArray
_geom_msg.Pose        = Pose
_geom_msg.Twist       = Twist

class String:
    def __init__(self): self.data = ''
class Bool:
    def __init__(self): self.data = False
_std_msg.String = String
_std_msg.Bool   = Bool

class SetBool:
    class Request:
        def __init__(self): self.data = False
    class Response:
        def __init__(self): self.success = False; self.message = ''
_srvs.SetBool = SetBool

class Marker:
    ADD=0; CUBE=1; SPHERE=2; CYLINDER=3
    def __init__(self):
        self.header=_H(); self.ns=''; self.id=0
        self.type=0; self.action=0
        self.scale=_P(); self.color=type('C',(),{'r':0,'g':0,'b':0,'a':0})()
        self.pose=_Pose()
class MarkerArray:
    def __init__(self): self.markers=[]
_viz.Marker      = Marker
_viz.MarkerArray = MarkerArray

# ── Add src to path ──────────────────────────────────────────────────────── #
BASE = os.path.join(os.path.dirname(__file__), '..', 'src')
for pkg in ['vehicle_control','traffic_light_ctrl','pedestrian_sim','intersection_manager']:
    p = os.path.join(BASE, pkg)
    if p not in sys.path:
        sys.path.insert(0, p)


# ═══════════════════════════════ TESTS ══════════════════════════════════════ #

def test_traffic_light_phase_cycle():
    """cycle_phase() should advance RED→GREEN→YELLOW→RED."""
    from traffic_light_ctrl.traffic_light_node import TrafficLightNode, PHASES
    node = TrafficLightNode.__new__(TrafficLightNode)
    node.phase_index = 0
    node.phase_pub   = _FakePub()
    node.timer       = _FakeTimer()
    node.get_logger  = lambda: _Logger()
    node.get_clock   = _FakeNode().get_clock
    node.create_timer = lambda *a,**k: _FakeTimer()

    node.cycle_phase()
    assert PHASES[node.phase_index] == 'GREEN'
    node.cycle_phase()
    assert PHASES[node.phase_index] == 'YELLOW'
    node.cycle_phase()
    assert PHASES[node.phase_index] == 'RED'


def test_set_phase_forces_green():
    """/set_phase with data=True must force phase to GREEN."""
    from traffic_light_ctrl.traffic_light_node import TrafficLightNode, PHASES
    node = TrafficLightNode.__new__(TrafficLightNode)
    node.phase_index = 0   # RED
    node.phase_pub   = _FakePub()
    node.get_logger  = lambda: _Logger()
    node.get_clock   = _FakeNode().get_clock

    req = SetBool.Request(); req.data = True
    resp = SetBool.Response()
    result = node.set_phase_callback(req, resp)
    assert result.success is True
    assert PHASES[node.phase_index] == 'GREEN'


def test_manager_denies_during_emergency():
    """arbitrate_access must deny when emergency_active=True."""
    from intersection_manager.intersection_manager_node import IntersectionManagerNode
    node = IntersectionManagerNode.__new__(IntersectionManagerNode)
    node.emergency_active = True
    node.current_phase    = 'GREEN'
    node.get_logger       = lambda: _Logger()

    req = SetBool.Request(); req.data = True
    resp = SetBool.Response()
    result = node.arbitrate_access(req, resp)
    assert result.success is False
    assert 'emergency' in result.message.lower()


def test_manager_grants_on_green():
    """arbitrate_access grants when phase=GREEN and no emergency."""
    from intersection_manager.intersection_manager_node import IntersectionManagerNode
    node = IntersectionManagerNode.__new__(IntersectionManagerNode)
    node.emergency_active = False
    node.current_phase    = 'GREEN'
    node.get_logger       = lambda: _Logger()

    req = SetBool.Request(); req.data = True
    resp = SetBool.Response()
    result = node.arbitrate_access(req, resp)
    assert result.success is True


def test_pedestrian_triggers_estop():
    """trigger_emergency_stop() must publish Bool True."""
    from pedestrian_sim.pedestrian_sim_node import PedestrianSimNode
    node = PedestrianSimNode.__new__(PedestrianSimNode)
    pub = _FakePub()
    node.estop_pub  = pub
    node.get_logger = lambda: _Logger()
    _FakePub.published.clear()
    node.trigger_emergency_stop()
    assert len(_FakePub.published) == 1
    assert _FakePub.published[0].data is True


def test_vehicle_stops_without_access():
    """Vehicles must not move when estop is active."""
    from vehicle_control.vehicle_control_node import VehicleControlNode
    node = VehicleControlNode.__new__(VehicleControlNode)
    node.estop  = True
    node.vehicles = [
        {'x': -5.0, 'y': 0.0, 'start': [-10.0, 0.0],
         'end': [10.0, 0.0], 'speed': 0.5},
    ]
    node.pose_pub = _FakePub()
    node.vel_pub  = _FakePub()
    node.get_logger = lambda: _Logger()
    node.get_clock  = _FakeNode().get_clock

    x_before = node.vehicles[0]['x']
    node.publish_state()
    assert node.vehicles[0]['x'] == x_before, "Vehicle must not move when estop is active"
