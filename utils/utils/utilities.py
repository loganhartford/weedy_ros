from geometry_msgs.msg import Quaternion
from math import atan2, sqrt, sin, cos, pi as M_PI

def create_quaternion_from_yaw(yaw: float) -> Quaternion:
    return Quaternion(
        x=0.0,
        y=0.0,
        z=sin(yaw / 2.0),
        w=cos(yaw / 2.0)
    )

def calculate_pose_error(pose, goal):
    """pose is PoseStamped.pose, goal is point msg"""
    angular_error = atan2(goal.y - pose.position.y, goal.x - pose.position.x) - pose.orientation.z
    angular_error = normalize_angle(angular_error)

    linear_error = sqrt((goal.x - pose.position.x) ** 2 + (goal.y - pose.position.y) ** 2)

    # Overshoot
    if pose.position.x > goal.x:
        linear_error = -linear_error

    return linear_error, angular_error

def normalize_angle(angle):
    if angle <= -M_PI:
        angle += 2*M_PI
    elif angle >= M_PI:
        angle -= 2*M_PI

    return angle
