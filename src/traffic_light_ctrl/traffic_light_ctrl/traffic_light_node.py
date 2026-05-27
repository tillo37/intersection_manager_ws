#!/usr/bin/env python3
"""
Member 2 — Traffic Light Controller
Cycles RED → GREEN → YELLOW automatically.
Exposes /set_phase service for emergency overrides.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool

PHASES = ['RED', 'GREEN', 'YELLOW']
DURATIONS = {'RED': 5.0, 'GREEN': 5.0, 'YELLOW': 2.0}


class TrafficLightNode(Node):

    def __init__(self):
        super().__init__('traffic_light_node')
        self.phase_index = 0
        self.phase_pub = self.create_publisher(String, '/traffic/phase', 10)
        self.set_phase_srv = self.create_service(
            SetBool, '/set_phase', self.set_phase_callback)
        self.timer = self.create_timer(5.0, self.cycle_phase)
        self.get_logger().info('TrafficLightNode started — initial phase: RED')
        # Publish immediately on start
        self._publish_phase()

    # ROS Function 1: cycle_phase
    def cycle_phase(self):
        """Publisher — advances traffic phase in rotation every ~5 s,
        publishes current phase to /traffic/phase (String)."""
        self.phase_index = (self.phase_index + 1) % len(PHASES)
        current = PHASES[self.phase_index]
        # Reschedule timer for this phase's duration
        self.timer.cancel()
        self.timer = self.create_timer(DURATIONS[current], self.cycle_phase)
        self._publish_phase()
        self.get_logger().info(f'Traffic light -> {current}')

    # ROS Function 2: set_phase_callback
    def set_phase_callback(self, request, response):
        """Service Server — exposes /set_phase (SetBool).
        When manager calls with data=True, forces phase to GREEN for emergency clearance."""
        if request.data:
            self.phase_index = PHASES.index('GREEN')
            self._publish_phase()
            response.success = True
            response.message = 'Phase forced to GREEN for emergency clearance'
            self.get_logger().warn('EMERGENCY: phase forced to GREEN by manager')
        else:
            response.success = False
            response.message = 'No action: request.data was False'
        return response

    def _publish_phase(self):
        msg = String()
        msg.data = PHASES[self.phase_index]
        self.phase_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
