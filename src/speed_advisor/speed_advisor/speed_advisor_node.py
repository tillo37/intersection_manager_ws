#!/usr/bin/env python3
"""
speed_advisor_node.py
---------------------
Member 6 — Speed Advisor
Package : speed_advisor
Node    : speed_advisor_node

Watches the current traffic phase and vehicle positions, then publishes
a recommended driving speed on /speed_advisory at 2 Hz.
When a vehicle is approaching the intersection during a non-GREEN phase
a warning is logged.  A service /set_speed_limit lets an operator enable
or disable the advisor at runtime.

ROS2 Functions:
  1. compute_advisory() — Timer callback (2 Hz) — publishes /speed_advisory
  2. set_limit_callback() — Service Server /set_speed_limit (SetBool)
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from std_srvs.srv import SetBool
from geometry_msgs.msg import PoseArray

# Distance from intersection centre at which a vehicle is "approaching"
APPROACH_ZONE = 5.0        # metres

# Default maximum speed allowed through the intersection
DEFAULT_SPEED_LIMIT = 0.5  # m/s  (matches vehicle_control default)


class SpeedAdvisorNode(Node):
    """
    Publishes a system-wide recommended speed based on the traffic phase
    and vehicle proximity to the intersection.
    """

    def __init__(self):
        super().__init__('speed_advisor_node')

        # ── State ─────────────────────────────────────────────────────────────
        self.current_phase  = 'RED'
        self.vehicle_poses  = []               # list of (x, y) tuples
        self.speed_limit    = DEFAULT_SPEED_LIMIT
        self.advisor_active = True             # toggled via /set_speed_limit

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(
            String,    '/traffic/phase', self._phase_cb,   10)
        self.create_subscription(
            PoseArray, '/vehicle/pose',  self._vehicle_cb, 10)

        # ── Publisher ─────────────────────────────────────────────────────────
        self.advisory_pub = self.create_publisher(Float32, '/speed_advisory', 10)

        # ── Service Server ────────────────────────────────────────────────────
        self.create_service(SetBool, '/set_speed_limit', self.set_limit_callback)

        # ── Timer — 2 Hz ──────────────────────────────────────────────────────
        self.create_timer(0.5, self.compute_advisory)

        self.get_logger().info(
            f'SpeedAdvisorNode started — speed_limit={self.speed_limit} m/s')

    # ──────────────────────────────────────────────────────────────────────────
    # ROS Function 1 — compute_advisory  (Publisher, 2 Hz timer)
    # ──────────────────────────────────────────────────────────────────────────
    def compute_advisory(self):
        """
        Timer callback at 2 Hz.

        Decides the recommended speed for all vehicles based on the current
        traffic phase:
          GREEN  → speed_limit          (proceed normally)
          YELLOW → speed_limit * 0.5   (slow approach)
          RED    → 0.0 m/s             (full stop)

        If the advisor is disabled (via service) the full speed_limit is
        always published regardless of phase.

        Also logs a warning whenever a vehicle is within APPROACH_ZONE metres
        of the intersection and the phase is not GREEN.
        """
        if not self.advisor_active:
            advisory = self.speed_limit
        elif self.current_phase == 'GREEN':
            advisory = self.speed_limit
        elif self.current_phase == 'YELLOW':
            advisory = self.speed_limit * 0.5
        else:                          # RED
            advisory = 0.0

        # Warn if a vehicle is close to the intersection on a non-GREEN phase
        if self.advisor_active and self.current_phase != 'GREEN':
            for (x, y) in self.vehicle_poses:
                dist = math.hypot(x, y)
                if dist < APPROACH_ZONE:
                    self.get_logger().warn(
                        f'[ADVISOR] Vehicle {dist:.1f} m from intersection — '
                        f'phase={self.current_phase}  advisory={advisory:.2f} m/s')

        msg = Float32()
        msg.data = float(advisory)
        self.advisory_pub.publish(msg)

    # ──────────────────────────────────────────────────────────────────────────
    # ROS Function 2 — set_limit_callback  (Service Server, /set_speed_limit)
    # ──────────────────────────────────────────────────────────────────────────
    def set_limit_callback(self, request: SetBool.Request,
                           response: SetBool.Response) -> SetBool.Response:
        """
        Service handler for /set_speed_limit (SetBool).

          request.data = True  → enable the advisor  (phase-based speed limits)
          request.data = False → disable the advisor  (always publish full speed)

        Returns success=True and a message describing the new state.
        """
        self.advisor_active = request.data
        state = 'ENABLED' if request.data else 'DISABLED'
        response.success = True
        response.message = f'Speed advisor {state} — limit={self.speed_limit} m/s'
        self.get_logger().info(f'[ADVISOR] {response.message}')
        return response

    # ── Internal callbacks ─────────────────────────────────────────────────────
    def _phase_cb(self, msg: String):
        self.current_phase = msg.data

    def _vehicle_cb(self, msg: PoseArray):
        self.vehicle_poses = [(p.position.x, p.position.y) for p in msg.poses]


def main(args=None):
    rclpy.init(args=args)
    node = SpeedAdvisorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
