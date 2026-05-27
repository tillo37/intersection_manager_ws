#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseArray, Pose, Twist
from std_msgs.msg import Bool, String, Float32, Int32MultiArray
from std_srvs.srv import SetBool
import math

NUM_VEHICLES = 3  # change this to add more

VEHICLE_CONFIGS = [
    {'start': [-10.0,  0.0], 'end': [ 10.0,  0.0], 'speed': 0.5},
    {'start': [  0.0,-10.0], 'end': [  0.0, 10.0], 'speed': 0.4},
    {'start': [ 10.0,  0.0], 'end': [-10.0,  0.0], 'speed': 0.3},
]

# Distance from intersection centre at which vehicles request access (M4)
ACCESS_ZONE = 2.0   # metres


class VehicleControlNode(Node):

    def __init__(self):
        super().__init__('vehicle_control_node')

        # Callback group allows service calls inside timer callbacks
        self.cb_group = ReentrantCallbackGroup()

        self.vehicles = [
            {
                'x': cfg['start'][0], 'y': cfg['start'][1],
                'start': cfg['start'], 'end': cfg['end'],
                'speed': cfg['speed'],
                'access_requested': False,   # M4: has sent a grant_access request
                'access_granted':   False,   # M4: has been allowed to enter
            }
            for cfg in VEHICLE_CONFIGS[:NUM_VEHICLES]
        ]

        self.phase = 'RED'
        self.create_subscription(String, '/traffic/phase', self.on_phase, 10)

        self.pose_pub = self.create_publisher(PoseArray, '/vehicle/pose', 10)
        self.vel_pub  = self.create_publisher(Twist,     '/vehicle/velocity', 10)

        # Subscribe to emergency stop (M3 zone-based)
        self.estop = False
        self.create_subscription(Bool, '/emergency_stop', self.on_estop, 10)

        # Subscribe to collision warning (M7) — set of vehicle indices to stop
        self.stopped_vehicles = set()
        self.create_subscription(
            Int32MultiArray, '/collision_warning', self.on_collision, 10)

        # Subscribe to collision slow (M7) — set of vehicle indices to halve speed
        self.slow_vehicles = set()
        self.create_subscription(
            Int32MultiArray, '/collision_slow', self.on_slow, 10)

        # Subscribe to speed advisory (M6) — default full speed until first message
        self.advisory_speed = 0.5
        self.create_subscription(Float32, '/speed_advisory', self.on_advisory, 10)

        # Service client for M4 intersection access
        self.access_client = self.create_client(
            SetBool, '/grant_access', callback_group=self.cb_group)

        self.timer = self.create_timer(
            0.1, self.publish_state, callback_group=self.cb_group)

        self.get_logger().info(f'VehicleControlNode started with {NUM_VEHICLES} vehicles')

    # ── Callbacks ─────────────────────────────────────────────────────────────
    def on_estop(self, msg):
        self.estop = msg.data

    def on_collision(self, msg):
        self.stopped_vehicles = set(msg.data)

    def on_slow(self, msg):
        self.slow_vehicles = set(msg.data)

    def on_phase(self, msg):
        self.phase = msg.data

    def on_advisory(self, msg):
        self.advisory_speed = msg.data

    # ── Main timer ────────────────────────────────────────────────────────────
    def publish_state(self):
        pa = PoseArray()
        pa.header.stamp    = self.get_clock().now().to_msg()
        pa.header.frame_id = 'map'

        for i, v in enumerate(self.vehicles):
            tx, ty   = v['end']
            dx, dy   = tx - v['x'], ty - v['y']
            dist_end = math.hypot(dx, dy)
            dist_centre = math.hypot(v['x'], v['y'])

            if dist_end < 0.5:
                # Reached destination — reset to start and clear access state
                v['x'], v['y'] = v['start']
                v['access_requested'] = False
                v['access_granted']   = False

            elif not self.estop and i not in self.stopped_vehicles and self.advisory_speed > 0.0:
                # Compute speed — M6 advisory + M7 slow zone
                speed = min(v['speed'], self.advisory_speed)
                if i in self.slow_vehicles:
                    speed *= 0.5

                if dist_centre <= ACCESS_ZONE and not v['access_granted']:
                    # At intersection threshold — ask M4 for permission, wait
                    if not v['access_requested']:
                        self._request_access(i)
                    # Do not move until access is granted
                else:
                    # Either far from intersection OR access already granted — move
                    v['x'] += speed * 0.1 * (dx / dist_end)
                    v['y'] += speed * 0.1 * (dy / dist_end)

            p = Pose()
            p.position.x = v['x']
            p.position.y = v['y']
            p.orientation.w = 1.0
            pa.poses.append(p)

        self.pose_pub.publish(pa)

        vel = Twist()
        vel.linear.x = 0.0 if self.estop else self.vehicles[0]['speed']
        self.vel_pub.publish(vel)

    # ── M4 access request ─────────────────────────────────────────────────────
    def _request_access(self, vehicle_idx: int):
        """Send an async /grant_access request to the intersection manager (M4)."""
        v = self.vehicles[vehicle_idx]
        v['access_requested'] = True

        if not self.access_client.service_is_ready():
            self.get_logger().warn(
                f'[V{vehicle_idx}] /grant_access not ready — will retry')
            v['access_requested'] = False
            return

        req = SetBool.Request()
        req.data = True
        future = self.access_client.call_async(req)
        future.add_done_callback(
            lambda f, idx=vehicle_idx: self._access_response(f, idx))
        self.get_logger().info(f'[V{vehicle_idx}] Access requested from M4')

    def _access_response(self, future, vehicle_idx: int):
        """Handle the async response from M4's /grant_access service."""
        v = self.vehicles[vehicle_idx]
        try:
            response = future.result()
            if response.success:
                v['access_granted'] = True
                self.get_logger().info(
                    f'[V{vehicle_idx}] Access GRANTED — {response.message}')
            else:
                v['access_requested'] = False   # allow retry next tick
                self.get_logger().warn(
                    f'[V{vehicle_idx}] Access DENIED — {response.message}')
        except Exception as e:
            v['access_requested'] = False
            self.get_logger().error(f'[V{vehicle_idx}] Service call failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = VehicleControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
