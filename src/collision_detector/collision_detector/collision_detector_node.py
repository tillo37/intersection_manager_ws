#!/usr/bin/env python3
"""
collision_detector_node.py
--------------------------
Member 7 — Collision Detector
Package : collision_detector
Node    : collision_detector_node

Monitors every vehicle-pedestrian pair independently.

  dist < DANGER_DISTANCE  (1 m) → red    sphere in RViz, that vehicle stops
  dist < WARNING_DISTANCE (2 m) → yellow sphere in RViz, that vehicle still moves
  otherwise               — no marker, no stop

Only the vehicles actually in danger (< 1 m) are stopped via /collision_warning.
The intersection zone colour (managed by M4) is NOT affected — zone and collision
detection are fully independent visual systems.

ROS2 Functions:
  1. check_proximity()   — Timer callback (10 Hz) — scans all vehicle-ped pairs
  2. broadcast_warning() — Publisher — /collision_warning (Int32MultiArray)
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration

WARNING_DISTANCE = 2.0   # metres — yellow sphere, vehicle keeps moving
DANGER_DISTANCE  = 1.0   # metres — red sphere,    vehicle stops


class CollisionDetectorNode(Node):
    """
    Per-vehicle proximity detector.
    Stops only the vehicles in danger, shows colour-coded spheres in RViz.
    Does not interfere with the intersection zone or global emergency stop.
    """

    def __init__(self):
        super().__init__('collision_detector_node')

        # ── State ─────────────────────────────────────────────────────────────
        self.vehicle_poses    = []
        self.pedestrian_poses = []
        self.warning_active   = False

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(
            PoseArray, '/vehicle/pose',   self._vehicle_cb,    10)
        self.create_subscription(
            PoseArray, '/obstacles/pose', self._pedestrian_cb, 10)

        # ── Publishers ────────────────────────────────────────────────────────
        # Indices of vehicles to stop (DANGER < 1 m)
        self.warning_pub = self.create_publisher(
            Int32MultiArray, '/collision_warning',     10)
        # Indices of vehicles to slow to 50% (WARNING 1–2 m)
        self.slow_pub    = self.create_publisher(
            Int32MultiArray, '/collision_slow',        10)
        # Colour-coded spheres at hazard midpoints
        self.marker_pub  = self.create_publisher(
            MarkerArray,     '/viz/collision_markers', 10)

        # ── Timer — 10 Hz ─────────────────────────────────────────────────────
        self.create_timer(0.1, self.check_proximity)

        self.get_logger().info(
            f'CollisionDetectorNode started — '
            f'warning={WARNING_DISTANCE} m  danger={DANGER_DISTANCE} m')

    # ──────────────────────────────────────────────────────────────────────────
    # ROS Function 1 — check_proximity  (Timer callback, 10 Hz)
    # ──────────────────────────────────────────────────────────────────────────
    def check_proximity(self):
        """
        Timer callback at 10 Hz.

        For each vehicle finds the closest pedestrian and classifies it:
          dist <  DANGER_DISTANCE  → DANGER:  red sphere,    vehicle stops
          dist <  WARNING_DISTANCE → WARNING: yellow sphere, vehicle moves
          dist >= WARNING_DISTANCE → SAFE:    no sphere,     vehicle moves

        Calls broadcast_warning() with the list of DANGER vehicle indices.
        Vehicle_control (M1) stops only those vehicles; others keep moving.
        The intersection zone colour is unaffected.
        """
        vehicle_stats = []   # (index, vx, vy, min_dist, closest_ped_pos)

        for vi, (vx, vy) in enumerate(self.vehicle_poses):
            min_dist = float('inf')
            closest  = None
            for px, py in self.pedestrian_poses:
                d = math.hypot(vx - px, vy - py)
                if d < min_dist:
                    min_dist = d
                    closest  = (px, py)
            vehicle_stats.append((vi, vx, vy, min_dist, closest))

        danger_indices  = [vi for vi, _, _, d, _ in vehicle_stats
                           if d < DANGER_DISTANCE]
        slow_indices    = [vi for vi, _, _, d, _ in vehicle_stats
                           if DANGER_DISTANCE <= d < WARNING_DISTANCE]

        if danger_indices and not self.warning_active:
            self.warning_active = True
            self.get_logger().error(
                f'[COLLISION] Vehicle(s) {danger_indices} in danger — stopping them.')
        elif not danger_indices and self.warning_active:
            self.warning_active = False
            self.get_logger().info('[COLLISION] All clear — vehicles may resume.')

        self.broadcast_warning(danger_indices, slow_indices)
        self._publish_markers(vehicle_stats)

    # ──────────────────────────────────────────────────────────────────────────
    # ROS Function 2 — broadcast_warning  (Publisher, /collision_warning)
    # ──────────────────────────────────────────────────────────────────────────
    def broadcast_warning(self, danger_indices: list, slow_indices: list):
        """
        Publishes per-vehicle stop and slow signals.

        /collision_warning (Int32MultiArray) — vehicles to stop  (< 1 m)
        /collision_slow    (Int32MultiArray) — vehicles to slow  (1–2 m, 50% speed)

        Vehicle_control (M1) checks each vehicle's index independently:
          in danger_indices → stop completely
          in slow_indices   → move at 50% of normal speed
          otherwise         → move normally
        """
        stop_msg = Int32MultiArray()
        stop_msg.data = danger_indices
        self.warning_pub.publish(stop_msg)

        slow_msg = Int32MultiArray()
        slow_msg.data = slow_indices
        self.slow_pub.publish(slow_msg)

    # ── Internal helpers ───────────────────────────────────────────────────────
    def _vehicle_cb(self, msg: PoseArray):
        self.vehicle_poses = [(p.position.x, p.position.y) for p in msg.poses]

    def _pedestrian_cb(self, msg: PoseArray):
        self.pedestrian_poses = [(p.position.x, p.position.y) for p in msg.poses]

    def _publish_markers(self, vehicle_stats):
        """
        Publish a sphere at the midpoint between each vehicle and its
        closest pedestrian when within WARNING_DISTANCE:
          yellow — within WARNING_DISTANCE but outside DANGER_DISTANCE
          red    — within DANGER_DISTANCE
        Scale is 0.5 m (half-metre sphere sitting on the ground).
        """
        ma = MarkerArray()
        for vi, vx, vy, min_dist, closest in vehicle_stats:
            if min_dist >= WARNING_DISTANCE or closest is None:
                continue
            px, py = closest
            m = Marker()
            m.header.frame_id    = 'map'
            m.header.stamp       = self.get_clock().now().to_msg()
            m.ns                 = 'collision'
            m.id                 = vi
            m.type               = Marker.SPHERE
            m.action             = Marker.ADD
            m.pose.position.x    = (vx + px) / 2.0
            m.pose.position.y    = (vy + py) / 2.0
            m.pose.position.z    = 0.25   # half of scale → sits on ground
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.5
            m.color.a = 0.9
            if min_dist < DANGER_DISTANCE:
                m.color.r, m.color.g, m.color.b = 1.0, 0.0, 0.0   # red
            else:
                m.color.r, m.color.g, m.color.b = 1.0, 0.5, 0.0   # orange
            m.lifetime = Duration(sec=0, nanosec=200000000)          # 0.2 s
            ma.markers.append(m)

        self.marker_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = CollisionDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
