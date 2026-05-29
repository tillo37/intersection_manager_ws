#!/usr/bin/env python3
"""
Member 3 — Pedestrian & Obstacle Simulation
Simulates 6 pedestrians moving near the intersection.
Fires emergency stop when any pedestrian enters the danger zone (radius 1.0 m).

ROS Functions:
  1. simulate_pedestrians()   — Timer callback (2 Hz), publishes /obstacles/pose
  2. trigger_emergency_stop() — Publishes Bool to /emergency_stop

Topics published:
  /obstacles/pose   (geometry_msgs/PoseArray) — all pedestrian positions
  /emergency_stop   (std_msgs/Bool)           — True when any ped in danger zone

Topics subscribed:
  /traffic/phase    (std_msgs/String)         — pedestrians freeze on RED/YELLOW
"""

import rclpy                            # Python client library for ROS2 —
                                        # connects this script to the ROS2 network
from rclpy.node import Node             # base class every ROS2 node must inherit
from geometry_msgs.msg import PoseArray, Pose   # message types for 2D/3D positions
from std_msgs.msg import Bool, String           # primitive message types
import math                             # for Euclidean distance (math.hypot)
import random                           # for random-walk movement model


# ── Constants ────────────────────────────────────────────────────────────────
DANGER_RADIUS = 1.0    # metres — if any pedestrian is closer than this to the
                       # intersection centre (0,0), an emergency stop fires
NUM_PEDS      = 6      # number of pedestrians in the simulation


class PedestrianSimNode(Node):
    """
    Simulates NUM_PEDS pedestrians using a 2D random walk model.

    Each pedestrian moves freely when the traffic phase is GREEN.
    On RED or YELLOW they freeze — pedestrians should not be crossing
    while vehicles have right of way.

    Every 0.5 s the node:
      1. Updates positions (random walk with 20% centre-bias for demo realism)
      2. Publishes all positions on /obstacles/pose
      3. Checks whether any pedestrian is within DANGER_RADIUS of the centre
      4. Publishes the emergency stop state on /emergency_stop (heartbeat pattern)
    """

    def __init__(self):
        super().__init__('pedestrian_sim_node')

        # Initialise pedestrian positions — each starts between 3 and 8 metres
        # from the centre, randomly in any quadrant, so no one starts in danger
        self.positions = [
            [random.choice([-1, 1]) * random.uniform(3, 8),
             random.choice([-1, 1]) * random.uniform(3, 8)]
            for _ in range(NUM_PEDS)
        ]

        # Track the current traffic phase so pedestrians know when to freeze
        self.current_phase = 'RED'

        # ── Publishers ───────────────────────────────────────────────────────
        # Queue size 10 — keeps last 10 messages if subscribers are slow
        self.obs_pub   = self.create_publisher(PoseArray, '/obstacles/pose', 10)
        self.estop_pub = self.create_publisher(Bool,      '/emergency_stop',  10)

        # ── Subscription ─────────────────────────────────────────────────────
        # Listen to the traffic light so pedestrians can react to phase changes
        self.create_subscription(String, '/traffic/phase', self._phase_cb, 10)

        # ── Timer: 2 Hz simulation tick (every 0.5 seconds) ──────────────────
        self.timer = self.create_timer(0.5, self.simulate_pedestrians)

        self.get_logger().info('PedestrianSimNode started — '
                               f'{NUM_PEDS} pedestrians, '
                               f'danger radius = {DANGER_RADIUS} m')

    # ── ROS Function 1: simulate_pedestrians ─────────────────────────────────
    def simulate_pedestrians(self):
        """
        Timer callback — fires every 0.5 s (2 Hz).

        Moves each pedestrian (only during GREEN phase), builds a PoseArray
        of all current positions, publishes it, then calls
        trigger_emergency_stop() with the current danger state.

        Movement model — random walk with centre bias:
          - 80% of steps: uniform random step in [-0.3, 0.3] m on each axis
          - 20% of steps: small nudge toward (0,0) — makes demo emergency
            stops occur within a reasonable simulation window
          Positions are clamped to [-8, 8] m so pedestrians stay in the arena.

        Heartbeat pattern: emergency stop is published every tick regardless
        of whether the state changed. If a message is lost on the network,
        the correct state is re-sent within 0.5 s — standard practice for
        safety-critical ROS2 signals.
        """
        pa = PoseArray()
        pa.header.stamp    = self.get_clock().now().to_msg()
        pa.header.frame_id = 'map'   # all positions are in the global map frame

        in_danger = False   # will be set True if any pedestrian enters danger zone

        for pos in self.positions:

            if self.current_phase != 'GREEN':
                # Pedestrians freeze on RED/YELLOW — publish position unchanged
                p = Pose()
                p.position.x    = pos[0]
                p.position.y    = pos[1]
                p.orientation.w = 1.0   # unit quaternion = no rotation
                pa.poses.append(p)
                continue

            # ── Random walk step ─────────────────────────────────────────────
            if random.random() < 0.2:
                # 20% chance: drift slightly toward intersection centre
                # Creates organic-looking behaviour and ensures emergency stops
                # occur during demos without needing to wait a long time
                step_x = -pos[0] * 0.1
                step_y = -pos[1] * 0.1
            else:
                # 80% chance: fully random step (uniform distribution)
                step_x = random.uniform(-0.3, 0.3)
                step_y = random.uniform(-0.3, 0.3)

            pos[0] += step_x
            pos[1] += step_y

            # Clamp to arena bounds — keeps pedestrians visible in RViz
            pos[0] = max(-8.0, min(8.0, pos[0]))
            pos[1] = max(-8.0, min(8.0, pos[1]))

            # Build Pose message for this pedestrian
            p = Pose()
            p.position.x    = pos[0]
            p.position.y    = pos[1]
            p.orientation.w = 1.0
            pa.poses.append(p)

            # ── Danger zone check ────────────────────────────────────────────
            # math.hypot(x, y) = sqrt(x² + y²) — Euclidean distance from origin
            # The intersection centre is at (0, 0) in the map frame
            dist = math.hypot(pos[0], pos[1])
            if dist < DANGER_RADIUS:
                in_danger = True   # at least one pedestrian too close

        # Publish all pedestrian positions for the intersection manager and
        # collision detector to consume
        self.obs_pub.publish(pa)

        # Delegate emergency stop publishing to the dedicated method —
        # single responsibility: this method handles movement only,
        # trigger_emergency_stop handles the safety signal
        self.trigger_emergency_stop(in_danger)

    # ── ROS Function 2: trigger_emergency_stop ───────────────────────────────
    def trigger_emergency_stop(self, active: bool):
        """
        Publishes the emergency stop state to /emergency_stop (Bool).

        Called every simulation tick (heartbeat pattern):
          active = True  → pedestrian inside DANGER_RADIUS — halt all vehicles
          active = False → danger zone clear — vehicles may resume

        Publishing False explicitly (not just silence) is important:
        the intersection manager uses this to know when it is safe to
        clear the emergency state and resume normal operation.

        Args:
            active: True if any pedestrian is within DANGER_RADIUS of origin.
        """
        msg = Bool()
        msg.data = active
        self.estop_pub.publish(msg)

        if active:
            self.get_logger().warn(
                f'EMERGENCY STOP: pedestrian in danger zone '
                f'(radius < {DANGER_RADIUS} m)!')
        else:
            self.get_logger().info(
                'Danger zone clear — resuming normal operation.')

    # ── Internal callback ─────────────────────────────────────────────────────
    def _phase_cb(self, msg: String):
        """
        Subscription callback for /traffic/phase.
        Updates current_phase so simulate_pedestrians knows whether to move.
        Pedestrians freeze on RED and YELLOW — they only cross on GREEN.
        """
        self.current_phase = msg.data
        self.get_logger().info(f'[PedSim] Phase updated → {self.current_phase}')


def main(args=None):
    rclpy.init(args=args)           # initialise ROS2 runtime
    node = PedestrianSimNode()
    rclpy.spin(node)                # keep node alive, dispatch timer callbacks
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
