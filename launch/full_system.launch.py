"""
full_system.launch.py
---------------------
Member 5 — Master Launch File
Starts ALL four ROS2 nodes in one command.
Usage:  ros2 launch launch/full_system.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Return a LaunchDescription that starts every system node."""

    # Static TF: publishes the map frame so RViz can display markers
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
        output='screen',
    )

    # Member 1: Vehicle Control
    vehicle_control_node = Node(
        package='vehicle_control',
        executable='vehicle_control_node',
        name='vehicle_control_node',
        output='screen',
    )

    # Member 2: Traffic Light Controller
    traffic_light_node = Node(
        package='traffic_light_ctrl',
        executable='traffic_light_node',
        name='traffic_light_node',
        output='screen',
    )

    # Member 3: Pedestrian Simulation
    pedestrian_sim_node = Node(
        package='pedestrian_sim',
        executable='pedestrian_sim_node',
        name='pedestrian_sim_node',
        output='screen',
    )

    # Member 4: Intersection Manager (brain)
    intersection_manager_node = Node(
        package='intersection_manager',
        executable='intersection_manager_node',
        name='intersection_manager_node',
        output='screen',
    )

    # Member 6: Speed Advisor
    speed_advisor_node = Node(
        package='speed_advisor',
        executable='speed_advisor_node',
        name='speed_advisor_node',
        output='screen',
    )

    # Member 7: Collision Detector
    collision_detector_node = Node(
        package='collision_detector',
        executable='collision_detector_node',
        name='collision_detector_node',
        output='screen',
    )

    return LaunchDescription([
        static_tf,
        vehicle_control_node,
        traffic_light_node,
        pedestrian_sim_node,
        intersection_manager_node,
        speed_advisor_node,
        collision_detector_node,
    ])
