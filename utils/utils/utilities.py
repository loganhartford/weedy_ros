from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float32MultiArray, MultiArrayLayout, MultiArrayDimension, UInt8MultiArray
from math import atan2, sqrt, sin, cos, pi as M_PI
import numpy as np

import utils.robot_params as rp

def create_quaternion_from_yaw(yaw: float) -> Quaternion:
    return Quaternion(
        x=0.0,
        y=0.0,
        z=sin(yaw / 2.0),
        w=cos(yaw / 2.0)
    )

def calculate_positioning_error(pose, goal):
    return goal[0] - pose.position.x

def calculate_linear_error(pose, goal):
    linear_error = sqrt((goal[0] - pose.position.x) ** 2 + (goal[1] - pose.position.y) ** 2)

    return linear_error

def calculate_angular_error(pose, goal):
    angular_error = atan2(goal[1] - pose.position.y, goal[0] - pose.position.x) - pose.orientation.z
    angular_error = normalize_angle(angular_error)

    return angular_error

def normalize_angle(angle):
    if angle <= -M_PI:
        angle += 2*M_PI
    elif angle >= M_PI:
        angle -= 2*M_PI

    return angle

def two_d_array_to_float32_multiarray(array):
    array = np.array(array).astype(np.float32)
    msg = Float32MultiArray()
    msg.data = array.flatten().tolist()

    msg.layout = MultiArrayLayout()
    msg.layout.dim.append(MultiArrayDimension(label="rows", size=array.shape[0], stride=array.shape[1]))
    msg.layout.dim.append(MultiArrayDimension(label="cols", size=array.shape[1], stride=1))

    return msg

def float32_multi_array_to_two_d_array(msg):
    if not msg.layout.dim:
        return []
    rows = msg.layout.dim[0].size
    cols = msg.layout.dim[1].size

    return [msg.data[i * cols:(i + 1) * cols] for i in range(rows)]

def package_removal_command(position):
    msg = UInt8MultiArray()
    msg.data = [rp.weed_removal_byte, (int(position) >> 8) & 0xFF, int(position) & 0xFF]
    return msg