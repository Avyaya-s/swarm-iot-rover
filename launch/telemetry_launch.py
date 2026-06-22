"""
telemetry_launch.py — Real Robot Mode Launch File
==================================================
Use this when the physical ESP32 rover is connected over Wi-Fi.

Starts:
    1. rosbridge_websocket   — JSON WebSocket gateway on Port 9090
    2. rover_bridge_node     — UDP listener (Port 5000) + odometry + sensor publishers
    3. mapper_node           — Occupancy grid ray-tracing mapper

Do NOT run room_simulator alongside this — it would conflict on the same topics.

Usage:
    ros2 launch swarm_bot telemetry_launch.py
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

    # 2. rover_bridge_node — UDP ingestion, odometry, TF, sensor publishers
    rover_bridge = Node(
        package='swarm_bot',
        executable='rover_bridge_node',
        name='rover_bridge_node',
        output='screen'
    )

    # 3. mapper_node — subscribes to /odom + /sensor/* → publishes /map
    mapper = Node(
        package='swarm_bot',
        executable='mapper_node',
        name='mapper_node',
        output='screen'
    )

    return LaunchDescription([
        rosbridge_launch,
        rover_bridge,
        mapper,
    ])