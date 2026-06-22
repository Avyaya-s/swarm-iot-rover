"""
sim_launch.py — Simulation Mode Launch File
============================================
Use this when testing WITHOUT the physical ESP32 rover.

Starts:
    1. rosbridge_websocket   — JSON WebSocket gateway on Port 9090
    2. mapper_node           — Occupancy grid ray-tracing mapper
    3. room_simulator        — Virtual 3m×3m room, publishes /odom + /sensor/*

Do NOT run rover_bridge_node alongside this — it would conflict on the same topics.

Usage:
    ros2 launch swarm_bot sim_launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource


def generate_launch_description():

    # 1. rosbridge_websocket (Port 9090)
    rosbridge_dir = get_package_share_directory('rosbridge_server')
    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(rosbridge_dir, 'launch', 'rosbridge_websocket_launch.xml')
        )
    )

    # 2. mapper_node — subscribes to /odom + /sensor/* → publishes /map
    mapper = Node(
        package='swarm_bot',
        executable='mapper_node',
        name='mapper_node',
        output='screen'
    )

    # 3. room_simulator — publishes /odom + /sensor/* (replaces rover_bridge_node)
    simulator = Node(
        package='swarm_bot',
        executable='room_simulator',
        name='room_simulator',
        output='screen'
    )

    return LaunchDescription([
        rosbridge_launch,
        mapper,
        simulator,
    ])