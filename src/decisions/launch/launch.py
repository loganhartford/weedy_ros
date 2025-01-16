from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
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
            prefix='/mnt/shared/weedy_ros/src/locomotion/locomotion/pwm_venv/bin/python3',
        ),
    ])
