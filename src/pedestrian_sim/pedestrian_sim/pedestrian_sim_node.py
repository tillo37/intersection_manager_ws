#!/usr/bin/env python3
"""
Member 3 — Pedestrian & Obstacle Simulation
Simulates 3 pedestrians moving near the intersection.
Fires emergency stop when any pedestrian enters the danger zone (radius 1.5 m).
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Bool
import math, random


DANGER_RADIUS = 1.0   # metres from origin (intersection centre)
NUM_PEDS      = 6


class PedestrianSimNode(Node):

    def __init__(self):
        super().__init__('pedestrian_sim_node')

        # Simple random-walk state for each pedestrian
        self.positions = [
            [random.choice([-1, 1]) * random.uniform(3, 8),
            random.choice([-1, 1]) * random.uniform(3, 8)]
            for _ in range(NUM_PEDS)
        ]

        self.obs_pub   = self.create_publisher(PoseArray, '/obstacles/pose', 10)
        self.estop_pub = self.create_publisher(Bool,      '/emergency_stop',  10)

        # 2 Hz simulation tick
        self.timer = self.create_timer(0.5, self.simulate_pedestrians)
        self.get_logger().info('PedestrianSimNode started')

    # ROS Function 1: simulate_pedestrians
    def simulate_pedestrians(self):
        """Publisher — every 0.5 s, publishes random pedestrian positions to
        /obstacles/pose (PoseArray) for collision detection by the manager."""
        pa = PoseArray()
        pa.header.stamp = self.get_clock().now().to_msg()
        pa.header.frame_id = 'map'

        in_danger = False
        for i, pos in enumerate(self.positions):
            # Random walk step
            pos[0] += random.uniform(-0.3, 0.3)
            pos[1] += random.uniform(-0.3, 0.3)
            # Clamp to arena
            pos[0] = max(-8.0, min(8.0, pos[0]))
            pos[1] = max(-8.0, min(8.0, pos[1]))

            p = Pose()
            p.position.x = pos[0]
            p.position.y = pos[1]
            p.orientation.w = 1.0
            pa.poses.append(p)

            dist = math.hypot(pos[0], pos[1])
            if dist < DANGER_RADIUS:
                in_danger = True

        self.obs_pub.publish(pa)

        msg = Bool()
        msg.data = in_danger
        self.estop_pub.publish(msg)
        if in_danger:
            self.get_logger().warn(
                'EMERGENCY STOP: pedestrian in danger zone (radius < 1.5 m)!')

    # ROS Function 2: trigger_emergency_stop
    def trigger_emergency_stop(self):
        """Publisher — publishes Bool True to /emergency_stop when a pedestrian
        enters the danger zone. Logs a warning to the ROS console."""
        msg = Bool()
        msg.data = True
        self.estop_pub.publish(msg)
        self.get_logger().warn(
            'EMERGENCY STOP: pedestrian in danger zone (radius < 1.5 m)!')


def main(args=None):
    rclpy.init(args=args)
    node = PedestrianSimNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
