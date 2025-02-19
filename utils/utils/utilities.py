from geometry_msgs.msg import Quaternion
from math import atan2, sqrt, sin, cos, pi as M_PI

def create_quaternion_from_yaw(yaw: float) -> Quaternion:
    """
    Create a quaternion representing a rotation around the Z-axis by the given yaw.

    Args:
        yaw (float): Yaw angle in radians.

    Returns:
        Quaternion: Quaternion representing the yaw rotation.
    """
    return Quaternion(
        x=0.0,
        y=0.0,
        z=sin(yaw / 2.0),
        w=cos(yaw / 2.0)
    )

def calculate_pos_error(current_pose, goal_pose):
    """
    Calculate the linear and angular error between the current and goal poses.

    Note:
        This function assumes that the current_pose's orientation.z holds the yaw value
        (in radians) directly.

    Args:
        current_pose: Pose with position and orientation (yaw in orientation.z).
        goal_pose: Pose with position.

    Returns:
        tuple: (linear_error, angular_error)
    """
    # Extract current and goal positions
    current_x = current_pose.position.x
    current_y = current_pose.position.y
    goal_x = goal_pose.position.x
    goal_y = goal_pose.position.y

    # Use the yaw from the current_pose (assuming it's stored in orientation.z)
    yaw = normalize_angle(current_pose.orientation.z)

    # Calculate the angle from the current position to the goal
    angle_to_goal = atan2(goal_y - current_y, goal_x - current_x)
    angular_error = normalize_angle(angle_to_goal - yaw)

    linear_error = sqrt((goal_x - current_x) ** 2 + (goal_y - current_y) ** 2)

    # Invert linear error based on a condition (e.g., relative x-positions)
    if current_x > goal_x:
        linear_error = -linear_error

    return linear_error, angular_error

def normalize_angle(angle: float) -> float:
    """
    Normalize an angle to the range [-pi, pi].

    Args:
        angle (float): Angle in radians.

    Returns:
        float: Normalized angle in radians.
    """
    angle = angle % (2 * M_PI)
    if angle > M_PI:
        angle -= 2 * M_PI
    return angle
