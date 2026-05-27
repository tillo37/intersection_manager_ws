#!/usr/bin/env python3
"""
Member 4 — Central Intersection Manager
Brain of the system: subscribes to all topics, decides right-of-way,
handles emergencies, overrides traffic light, publishes RViz markers.
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseArray
from std_msgs.msg import String, Bool
from std_srvs.srv import SetBool
from visualization_msgs.msg import Marker, MarkerArray
import math


INTERSECTION_RADIUS = 2.0   # metres — approach alert zone


class IntersectionManagerNode(Node):

    def __init__(self):
        super().__init__('intersection_manager_node')
        self.cb_group = ReentrantCallbackGroup()

        # State cache
        self.current_phase    = 'RED'
        self.emergency_active = False
        self.vehicle_poses = []
        self.obstacle_poses   = []

        # Subscriptions
        self.create_subscription(PoseArray, '/vehicle/pose',
                                 self.vehicle_pose_cb, 10)
        self.create_subscription(String,      '/traffic/phase',
                                 self._phase_cb, 10)
        self.create_subscription(PoseArray,   '/obstacles/pose',
                                 self._obstacles_cb, 10)
        self.create_subscription(Bool,        '/emergency_stop',
                                 self._estop_cb, 10)

        # Services
        self.grant_srv  = self.create_service(
            SetBool, '/grant_access', self.arbitrate_access)
        self.phase_client = self.create_client(
            SetBool, '/set_phase', callback_group=self.cb_group)

        # RViz markers publisher
        self.marker_pub = self.create_publisher(
            MarkerArray, '/viz/markers', 10)

        # Publish markers at 2 Hz
        self.create_timer(0.5, self._publish_markers)

        self.get_logger().info('IntersectionManagerNode started')

    # ------------------------------------------------------------------ #
    # ROS Function 1: vehicle_pose_cb                                      #
    # ------------------------------------------------------------------ #
   # Replace vehicle_pose_cb with:
    def vehicle_pose_cb(self, msg: PoseArray):
        self.vehicle_poses = [(p.position.x, p.position.y) for p in msg.poses]
        for (x, y) in self.vehicle_poses:
            dist = math.hypot(x, y)
            if dist <= INTERSECTION_RADIUS:
                self.get_logger().info(
                    f'[MANAGER] Vehicle within approach zone: dist={dist:.2f} m  phase={self.current_phase}')

    # ------------------------------------------------------------------ #
    # ROS Function 2: arbitrate_access                                     #
    # ------------------------------------------------------------------ #
    def arbitrate_access(self, request: SetBool.Request,
                         response: SetBool.Response) -> SetBool.Response:
        """Service Server — exposes /grant_access (SetBool).
        Checks phase + emergency state, grants or denies right-of-way."""
        if self.emergency_active:
            response.success = False
            response.message = 'DENIED — emergency stop is active'
            self.get_logger().warn('[MANAGER] Access denied: emergency active')
        elif self.current_phase == 'GREEN':
            response.success = True
            response.message = 'GRANTED — phase is GREEN'
            self.get_logger().info('[MANAGER] Access granted')
        else:
            response.success = False
            response.message = f'DENIED — phase is {self.current_phase}'
            self.get_logger().info(
                f'[MANAGER] Access denied: phase={self.current_phase}')
        return response

    # ------------------------------------------------------------------ #
    # Internal callbacks                                                   #
    # ------------------------------------------------------------------ #
    def _phase_cb(self, msg: String):
        self.current_phase = msg.data

    def _obstacles_cb(self, msg: PoseArray):
        self.obstacle_poses = [(p.position.x, p.position.y) for p in msg.poses]

    def _estop_cb(self, msg: Bool):
        if msg.data and not self.emergency_active:
            self.emergency_active = True
            self.get_logger().error('[MANAGER] EMERGENCY STOP received — halting all vehicles')
            self._force_phase_green()
        elif not msg.data:
            self.emergency_active = False

    def _force_phase_green(self):
        """Calls /set_phase to clear the intersection."""
        if not self.phase_client.service_is_ready():
            self.get_logger().warn('/set_phase service not ready')
            return
        req = SetBool.Request()
        req.data = True
        future = self.phase_client.call_async(req)
        future.add_done_callback(lambda f: self.get_logger().info(
            f'Force-green result: {f.result().message}'))

    # ------------------------------------------------------------------ #
    # RViz markers                                                         #
    # ------------------------------------------------------------------ #
    def _publish_markers(self):
        ma = MarkerArray()

        # Intersection zone sphere
        zone = Marker()
        zone.header.frame_id = 'map'
        zone.header.stamp = self.get_clock().now().to_msg()
        zone.ns = 'intersection'
        zone.id = 0
        zone.type = Marker.CYLINDER
        zone.action = Marker.ADD
        zone.scale.x = INTERSECTION_RADIUS * 2
        zone.scale.y = INTERSECTION_RADIUS * 2
        zone.scale.z = 0.05
        zone.color.a = 0.3
        if self.emergency_active:
            zone.color.r = 1.0
        elif self.current_phase == 'GREEN':
            zone.color.g = 1.0
        elif self.current_phase == 'YELLOW':
            zone.color.r, zone.color.g = 1.0, 1.0
        else:
            zone.color.r = 0.8
        zone.pose.orientation.w = 1.0
        ma.markers.append(zone)

        # Vehicle marker
        for i, (vx, vy) in enumerate(self.vehicle_poses):
            vm = Marker()
            vm.header.frame_id = 'map'
            vm.header.stamp = self.get_clock().now().to_msg()
            vm.ns = 'vehicle'
            vm.id = 1 + i
            vm.type = Marker.CUBE
            vm.action = Marker.ADD
            vm.pose.position.x = vx
            vm.pose.position.y = vy
            vm.pose.orientation.w = 1.0
            vm.scale.x = vm.scale.y = vm.scale.z = 0.5
            vm.color.r = vm.color.g = 0.9
            vm.color.a = 1.0
            ma.markers.append(vm)

        # Pedestrian markers
        for i, (px, py) in enumerate(self.obstacle_poses):
            pm = Marker()
            pm.header.frame_id = 'map'
            pm.header.stamp = self.get_clock().now().to_msg()
            pm.ns = 'pedestrians'
            pm.id = 10 + i
            pm.type = Marker.SPHERE
            pm.action = Marker.ADD
            pm.pose.position.x = px
            pm.pose.position.y = py
            pm.pose.orientation.w = 1.0
            pm.scale.x = pm.scale.y = pm.scale.z = 0.3
            pm.color.b = 1.0
            pm.color.a = 1.0
            ma.markers.append(pm)

        self.marker_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = IntersectionManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
