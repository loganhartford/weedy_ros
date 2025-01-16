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
            package='camera_package',
            executable='camera',
            name='camera',
            output='screen'
        ),
        Node(
            package='neopixel_ring',
            executable='neopixel_ring',
            name='neopixel_ring',
            output='screen',
            prefix='/mnt/shared/weedy_ros/src/neopixel_ring/neopixel_ring/venv/bin/python3',
        ),
        Node(
            package='locomotion',
            executable='controller',
            name='controller',
            output='screen',
            prefix='/mnt/shared/weedy_ros/src/locomotion/locomotion/pwm_venv/bin/python3',
        ),
        Node(
            package='uart',
            executable='uart',
            name='uart',
            output='screen',
        ),
    ])
