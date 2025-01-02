from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='controller',
            executable='controller',
            name='controller',
            output='screen'
        ),
        # Node(
        #     package='gpio',
        #     executable='button_publisher',
        #     name='button_publisher',
        #     output='screen'
        # ),
        Node(
            package='camera_package',
            executable='camera',
            name='camera',
            output='screen'
        ),
        # Node(
        #     package='gpio',
        #     executable='led_subscriber',
        #     name='led_subscriber',
        #     output='screen'
        # ),
        Node(
            package='inference',
            executable='inference',
            name='inference',
            output='screen',
            prefix='/mnt/shared/weedy_ros/src/inference/inference/yolo_env/bin/python3',
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
            executable='locomotion',
            name='locomotion',
            output='screen',
        ),
        # Node(
        #     package='homography',
        #     executable='homography',
        #     name='homography',
        #     output='screen',
        # ),
        Node(
            package='uart',
            executable='uart',
            name='uart',
            output='screen',
        ),
    ])
