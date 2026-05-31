#!/usr/bin/env python3
"""
Motors-only bringup for WARDEN driving.

Starts just turtlebot3_node (OpenCR/Dynamixel driver on /dev/ttyACM0). It does NOT
start the LDS lidar driver, because that driver probes CP2102 serial ports to
autodetect the lidar and will grab the ESP32's /dev/ttyUSB0 (both are 10c4:ea60
CP2102s). Skipping it keeps the ESP32 port free for warden_host.py.

turtlebot3_node declares a statically-typed 'namespace' parameter with no default,
so it must be passed explicitly (the official bringup does this for you).

Run:  ros2 launch ~/navx_scripts/warden/drive_only.launch.py
Then: ros2 run joy joy_node   and   python3 warden_host.py
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_share = get_package_share_directory('turtlebot3_bringup')
    param_file = os.path.join(bringup_share, 'param', 'humble', 'burger.yaml')
    # Waffle Pi -> swap to 'waffle_pi.yaml'.

    tb3_node = Node(
        package='turtlebot3_node',
        executable='turtlebot3_ros',
        arguments=['-i', '/dev/ttyACM0'],
        parameters=[
            param_file,
            {'namespace': ''},   # required, no default in this build
        ],
        output='screen',
    )

    return LaunchDescription([tb3_node])