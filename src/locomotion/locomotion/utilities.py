from geometry_msgs.msg import Quaternion, Twist, PoseStamped
from nav_msgs.msg import Odometry

from math import atan2, sqrt, pi as M_PI, sin, cos

def create_quaternion_from_yaw(yaw):
    return Quaternion(
        x=0.0,
        y=0.0,
        z=sin(yaw / 2.0),
        w=cos(yaw / 2.0)
    )

def calculate_pos_error(current_pose, goal_pose):
    goal_x = goal_pose.position.x
    goal_y = goal_pose.position.y
    current_x = current_pose.position.x
    current_y = current_pose.position.y
    theta = 2 * atan2(current_pose.orientation.z, current_pose.orientation.w)
    
    # Map theta between -pi and pi
    theta = normalize_angle(theta) 

    # theta = (theta + M_PI) % (2 * M_PI) - M_PI 
    angle_to_goal = atan2(goal_y - current_y, goal_x - current_x)
    angular_error = angle_to_goal - theta
    angular_error = normalize_angle(angular_error)

    linear_error = sqrt((goal_y - current_y) ** 2 + (goal_x - current_x) ** 2)
    
    return linear_error, angular_error

def normalize_angle(angle):
    angle = angle % (2 * M_PI)
    if angle > M_PI:
        angle -= 2 * M_PI
    return angle