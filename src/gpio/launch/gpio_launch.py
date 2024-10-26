from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gpio',
            executable='button_publisher',
            name='button_publisher',
            output='screen'
        ),
        Node(
            package='camera_package',
            executable='camera',
            name='camera',
            output='screen'
        ),
        Node(
            package='gpio',
            executable='led_subscriber',
            name='led_subscriber',
            output='screen'
        ),
        Node(
            package='inference',
            executable='inference',
            name='inference',
            output='screen'
        ),
    ])
