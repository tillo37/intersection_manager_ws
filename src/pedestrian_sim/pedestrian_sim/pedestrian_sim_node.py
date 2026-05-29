#!/usr/bin/env python3
"""
Member 3 — Pedestrian & Obstacle Simulation
Simulates 6 pedestrians moving near the intersection.
Fires emergency stop when any pedestrian enters the danger zone (radius 1.0 m).

ROS Functions:
  1. simulate_pedestrians()   — Timer callback (2 Hz), publishes /obstacles/pose
  2. trigger_emergency_stop() — Publishes Bool to /emergency_stop
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Bool, String
import math, random


DANGER_RADIUS = 1.0   # metres from origin (intersection centre)
NUM_PEDS      = 6


class PedestrianSimNode(Node):

    def __init__(self):
        super().__init__('pedestrian_sim_node')

        # Random-walk state — each pedestrian starts far from centre
        self.positions = [
            [random.choice([-1, 1]) * random.uniform(3, 8),
             random.choice([-1, 1]) * random.uniform(3, 8)]
            for _ in range(NUM_PEDS)
        ]

        # Current traffic phase — pedestrians freeze on RED/YELLOW
        self.current_phase = 'RED'

        # Publishers
        self.obs_pub   = self.create_publisher(PoseArray, '/obstacles/pose', 10)
        self.estop_pub = self.create_publisher(Bool,      '/emergency_stop',  10)

        # Subscription — listen to traffic light
        self.create_subscription(String, '/traffic/phase', self._phase_cb, 10)

        # 2 Hz simulation tick
        self.timer = self.create_timer(0.5, self.simulate_pedestrians)
        self.get_logger().info('PedestrianSimNode started')

    # ── ROS Function 1: simulate_pedestrians ─────────────────────────────────
    def simulate_pedestrians(self):
        """Timer callback at 2 Hz. Moves each pedestrian (only on GREEN),
        publishes /obstacles/pose, calls trigger_emergency_stop()."""
        pa = PoseArray()
        pa.header.stamp    = self.get_clock().now().to_msg()
        pa.header.frame_id = 'map'

        in_danger = False

        for i, pos in enumerate(self.positions):

            if self.current_phase != 'GREEN':
                # Freeze — publish current position without moving
                p = Pose()
                p.position.x    = pos[0]
                p.position.y    = pos[1]
                p.orientation.w = 1.0
                pa.poses.append(p)
                continue

            # Random walk with 20% bias toward centre for demo-able emergency stops
            if random.random() < 0.2:
                toward_x = -pos[0] * 0.1
                toward_y = -pos[1] * 0.1
            else:
                toward_x = random.uniform(-0.3, 0.3)
                toward_y = random.uniform(-0.3, 0.3)
            pos[0] += toward_x
            pos[1] += toward_y

            # Clamp to arena bounds
            pos[0] = max(-8.0, min(8.0, pos[0]))
            pos[1] = max(-8.0, min(8.0, pos[1]))

            p = Pose()
            p.position.x    = pos[0]
            p.position.y    = pos[1]
            p.orientation.w = 1.0
            pa.poses.append(p)

            # Check danger zone
            dist = math.hypot(pos[0], pos[1])
            if dist < DANGER_RADIUS:
                in_danger = True

        self.obs_pub.publish(pa)
        self.trigger_emergency_stop(in_danger)

    # ── ROS Function 2: trigger_emergency_stop ───────────────────────────────
    def trigger_emergency_stop(self, active: bool):
        """ Publishes emergency stop state to /emergency_stop (Bool).
        active=True  → pedestrian inside danger zone, halt all vehicles.
        active=False → zone clear, vehicles may resume.
        """
        msg = Bool()
        msg.data = active
        self.estop_pub.publish(msg)
        if active:
            self.get_logger().warn(
                'EMERGENCY STOP: pedestrian in danger zone (radius < 1.0 m)!')
        else:
            self.get_logger().info('Danger zone clear — resuming normal operation.')

    def _phase_cb(self, msg: String):
        """Subscription — receives /traffic/phase.
        Pedestrians freeze on RED/YELLOW, walk on GREEN."""
        self.current_phase = msg.data
        self.get_logger().info(f'Phase updated: {self.current_phase}')


def main(args=None):
    rclpy.init(args=args)
    node = PedestrianSimNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
