#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, Twist
from std_msgs.msg import Bool, String
import math

NUM_VEHICLES = 3  # change this to add more

# Each vehicle: [start_x, start_y, speed]
# Replace VEHICLE_CONFIGS and the vehicle dict with this:

VEHICLE_CONFIGS = [
    {'start': [-10.0,  0.0], 'end': [ 10.0,  0.0], 'speed': 0.5},
    {'start': [  0.0,-10.0], 'end': [  0.0, 10.0], 'speed': 0.4},
    {'start': [ 10.0,  0.0], 'end': [-10.0,  0.0], 'speed': 0.3},
]
class VehicleControlNode(Node):

    def __init__(self):
        super().__init__('vehicle_control_node')

       # In __init__, replace self.vehicles with:
        self.vehicles = [
            {'x': cfg['start'][0], 'y': cfg['start'][1],
            'start': cfg['start'], 'end': cfg['end'],
            'speed': cfg['speed']}
            for cfg in VEHICLE_CONFIGS[:NUM_VEHICLES]
]
        # In __init__, add:
        self.phase = 'RED'
        self.create_subscription(String, '/traffic/phase', self.on_phase, 10)
        
        self.pose_pub = self.create_publisher(PoseArray, '/vehicle/pose', 10)
        self.vel_pub  = self.create_publisher(Twist,     '/vehicle/velocity', 10)

        # Subscribe to emergency stop
        self.estop = False
        self.create_subscription(Bool, '/emergency_stop', self.on_estop, 10)

        self.timer = self.create_timer(0.1, self.publish_state)
        self.get_logger().info(f'VehicleControlNode started with {NUM_VEHICLES} vehicles')

    def on_estop(self, msg):
        self.estop = msg.data

    def on_phase(self, msg):
        self.phase = msg.data
        #self.get_logger().info(f'Traffic phase changed to: {self.phase}')

    def publish_state(self):
        pa = PoseArray()
        pa.header.stamp = self.get_clock().now().to_msg()
        pa.header.frame_id = 'map'

        for v in self.vehicles:
            tx, ty = v['end']
            dx, dy = tx - v['x'], ty - v['y']
            dist = math.hypot(dx, dy)

            # Move toward intersection (origin) if not stopped
            if dist < 0.5:
            # Reached end, reset to start
                v['x'], v['y'] = v['start']
            elif not self.estop and self.phase == 'GREEN':
                v['x'] += v['speed'] * 0.1 * (dx / dist)
                v['y'] += v['speed'] * 0.1 * (dy / dist)

            p = Pose()
            p.position.x = v['x']
            p.position.y = v['y']
            p.orientation.w = 1.0
            pa.poses.append(p)

        self.pose_pub.publish(pa)

        vel = Twist()
        vel.linear.x = 0.0 if self.estop else self.vehicles[0]['speed']
        self.vel_pub.publish(vel)


def main(args=None):
    rclpy.init(args=args)
    node = VehicleControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()