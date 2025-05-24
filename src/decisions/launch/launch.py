import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Locate the ekf.yaml in your decisions package
    ekf_config_file = "/mnt/shared/weedy_ros/src/decisions/decisions/config/ekf.yaml"

    return LaunchDescription([
        Node(
            package='utils',
            executable='uart',
            name='uart',
            output='screen',
        ),
        Node(
            package='decisions',
            executable='decisions',
            name='decisions',
            output='screen',
            prefix='/mnt/shared/weedy_ros/src/decisions/decisions/venv/bin/python3'
        ),
        Node(
            package='locomotion',
            executable='controller',
            name='controller',
            output='screen',
            prefix='/mnt/shared/weedy_ros/src/locomotion/locomotion/venv/bin/python3',
        ),
        Node(
            package='locomotion',
            executable='odometry',
            name='odometry',
            output='screen',
            prefix='/mnt/shared/weedy_ros/src/locomotion/locomotion/venv/bin/python3',
        ),
        Node(
            package='locomotion',
            executable='localization',
            name='localization',
            output='screen',
            prefix='/mnt/shared/weedy_ros/src/locomotion/locomotion/venv/bin/python3',
        ),
        Node(
            package='locomotion',
            executable='bno085_imu',
            name='bno085_imu',
            output='screen',
            prefix='/mnt/shared/weedy_ros/src/locomotion/locomotion/venv/bin/python3',
        ),
        Node(
            package='decisions',
            executable='teleop',
            name='teleop',
            output='screen',
        ),
        # Node(
        #     package='foxglove_bridge',
        #     executable='foxglove_bridge',
        # )

        # Add the EKF node from robot_localization
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_file]
        ),
    ])
