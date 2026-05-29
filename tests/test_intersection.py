"""
Member 5 — pytest test suite
Tests core logic without requiring a live ROS2 network.
Run: python3 -m pytest tests/test_intersection.py -v
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
_stub('builtin_interfaces'); _bi  = _stub('builtin_interfaces.msg')

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
class Float32:
    def __init__(self): self.data = 0.0
class Int32MultiArray:
    def __init__(self): self.data = []

_std_msg.String        = String
_std_msg.Bool          = Bool
_std_msg.Float32       = Float32
_std_msg.Int32MultiArray = Int32MultiArray

class SetBool:
    class Request:
        def __init__(self): self.data = False
    class Response:
        def __init__(self): self.success = False; self.message = ''
_srvs.SetBool = SetBool

class Marker:
    ADD=0; CUBE=1; SPHERE=2; CYLINDER=3; TEXT_VIEW_FACING=9; DELETEALL=3
    def __init__(self):
        self.header=_H(); self.ns=''; self.id=0
        self.type=0; self.action=0
        self.scale=_P(); self.color=type('C',(),{'r':0,'g':0,'b':0,'a':0})()
        self.pose=_Pose(); self.lifetime=None; self.text=''
class MarkerArray:
    def __init__(self): self.markers=[]
_viz.Marker      = Marker
_viz.MarkerArray = MarkerArray

class Duration:
    def __init__(self, sec=0, nanosec=0): self.sec=sec; self.nanosec=nanosec
_bi.Duration = Duration

# ── Add src to path ──────────────────────────────────────────────────────── #
BASE = os.path.join(os.path.dirname(__file__), '..', 'src')
for pkg in ['vehicle_control','traffic_light_ctrl','pedestrian_sim',
            'intersection_manager','speed_advisor','collision_detector']:
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
    """trigger_emergency_stop(active=True) must publish Bool True."""
    from pedestrian_sim.pedestrian_sim_node import PedestrianSimNode
    node = PedestrianSimNode.__new__(PedestrianSimNode)
    pub = _FakePub()
    node.estop_pub  = pub
    node.get_logger = lambda: _Logger()
    _FakePub.published.clear()
    node.trigger_emergency_stop(True)   
    assert len(_FakePub.published) == 1
    assert _FakePub.published[0].data is True

def test_pedestrian_danger_detection():
    """simulate_pedestrians must detect when a pedestrian is inside danger zone."""
    from pedestrian_sim.pedestrian_sim_node import PedestrianSimNode, DANGER_RADIUS
    node = PedestrianSimNode.__new__(PedestrianSimNode)
    # Place one pedestrian clearly inside danger zone
    node.positions = [[0.1, 0.1], [5.0, 5.0]]
    node.obs_pub    = _FakePub()
    node.estop_pub  = _FakePub()
    node.get_logger = lambda: _Logger()
    node.get_clock  = _FakeNode().get_clock
    _FakePub.published.clear()
    node.current_phase = 'GREEN'   # add this line so pedestrians can move
    node.simulate_pedestrians()
    # estop_pub should have received Bool True
    estop_msgs = [m for m in _FakePub.published if hasattr(m, 'data') and m.data is True]
    assert len(estop_msgs) >= 1, "Emergency stop must fire when pedestrian is in danger zone"
    

def test_pedestrian_freezes_on_red():
    """Pedestrians must not move when traffic phase is RED."""
    from pedestrian_sim.pedestrian_sim_node import PedestrianSimNode
    node = PedestrianSimNode.__new__(PedestrianSimNode)
    node.positions     = [[3.0, 3.0], [5.0, 5.0]]
    node.current_phase = 'RED'
    node.obs_pub       = _FakePub()
    node.estop_pub     = _FakePub()
    node.get_logger    = lambda: _Logger()
    node.get_clock     = _FakeNode().get_clock
    _FakePub.published.clear()

    pos_before = [list(p) for p in node.positions]
    node.simulate_pedestrians()
    pos_after  = [list(p) for p in node.positions]

    assert pos_before == pos_after, "Pedestrians must not move during RED phase"    


def test_vehicle_stops_without_access():
    """Vehicles must not move when estop is active."""
    from vehicle_control.vehicle_control_node import VehicleControlNode
    node = VehicleControlNode.__new__(VehicleControlNode)
    node.estop  = True
    node.vehicles = [
        {'x': -5.0, 'y': 0.0, 'start': [-10.0, 0.0],
         'end': [10.0, 0.0], 'speed': 0.5,
         'access_requested': False, 'access_granted': False},
    ]
    node.pose_pub = _FakePub()
    node.vel_pub  = _FakePub()
    node.get_logger = lambda: _Logger()
    node.get_clock  = _FakeNode().get_clock

    x_before = node.vehicles[0]['x']
    node.publish_state()
    assert node.vehicles[0]['x'] == x_before, "Vehicle must not move when estop is active"


def test_speed_advisor_phase_red():
    """compute_advisory() must publish 0.0 when phase is RED."""
    from speed_advisor.speed_advisor_node import SpeedAdvisorNode
    node = SpeedAdvisorNode.__new__(SpeedAdvisorNode)
    node.current_phase  = 'RED'
    node.vehicle_poses  = []
    node.speed_limit    = 0.5
    node.advisor_active = True
    node.advisory_pub   = _FakePub()
    node.get_logger     = lambda: _Logger()
    _FakePub.published.clear()
    node.compute_advisory()
    assert _FakePub.published[-1].data == 0.0


def test_speed_advisor_phase_yellow():
    """compute_advisory() must publish half speed when phase is YELLOW."""
    from speed_advisor.speed_advisor_node import SpeedAdvisorNode
    node = SpeedAdvisorNode.__new__(SpeedAdvisorNode)
    node.current_phase  = 'YELLOW'
    node.vehicle_poses  = []
    node.speed_limit    = 0.5
    node.advisor_active = True
    node.advisory_pub   = _FakePub()
    node.get_logger     = lambda: _Logger()
    _FakePub.published.clear()
    node.compute_advisory()
    assert _FakePub.published[-1].data == 0.25


def test_speed_advisor_phase_green():
    """compute_advisory() must publish full speed when phase is GREEN."""
    from speed_advisor.speed_advisor_node import SpeedAdvisorNode
    node = SpeedAdvisorNode.__new__(SpeedAdvisorNode)
    node.current_phase  = 'GREEN'
    node.vehicle_poses  = []
    node.speed_limit    = 0.5
    node.advisor_active = True
    node.advisory_pub   = _FakePub()
    node.get_logger     = lambda: _Logger()
    _FakePub.published.clear()
    node.compute_advisory()
    assert _FakePub.published[-1].data == 0.5


def test_collision_detector_no_danger():
    """check_proximity() must return empty danger list when all vehicles are far."""
    from collision_detector.collision_detector_node import CollisionDetectorNode
    node = CollisionDetectorNode.__new__(CollisionDetectorNode)
    node.vehicle_poses    = [(10.0, 0.0), (0.0, 10.0)]
    node.pedestrian_poses = [(0.0, 0.0)]   # far from both vehicles
    node.warning_active   = False
    node.warning_pub      = _FakePub()
    node.slow_pub         = _FakePub()
    node.marker_pub       = _FakePub()
    node.get_logger       = lambda: _Logger()
    _FakePub.published.clear()
    node.check_proximity()
    # First published message is /collision_warning — should have no danger indices
    warning_msg = _FakePub.published[0]
    assert warning_msg.data == [], "No vehicles should be in danger"


def test_collision_detector_danger():
    """check_proximity() must include vehicle index when within DANGER_DISTANCE."""
    from collision_detector.collision_detector_node import (
        CollisionDetectorNode, DANGER_DISTANCE)
    node = CollisionDetectorNode.__new__(CollisionDetectorNode)
    # Place vehicle 0 very close to the pedestrian
    node.vehicle_poses    = [(0.5, 0.0), (10.0, 0.0)]
    node.pedestrian_poses = [(0.0, 0.0)]
    node.warning_active   = False
    node.warning_pub      = _FakePub()
    node.slow_pub         = _FakePub()
    node.marker_pub       = _FakePub()
    node.get_logger       = lambda: _Logger()
    node.get_clock        = _FakeNode().get_clock
    _FakePub.published.clear()
    node.check_proximity()
    warning_msg = _FakePub.published[0]
    assert 0 in warning_msg.data, "Vehicle 0 should be flagged as dangerous"
    assert 1 not in warning_msg.data, "Vehicle 1 should not be flagged"
