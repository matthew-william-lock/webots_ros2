import math
import numpy as np


def quat2euler(qw, qx, qy, qz):
    """
    Quaternion to Euler angles q = [qw qx qy qz]
    returns [roll, pitch, yaw]
    """
   
    # q = [qw qx qy qz]
    roll = math.atan2(2*(qw*qx + qy*qz), 1 - 2*(qx**2 + qy**2))
    pitch = math.asin(2*(qw*qy - qz*qx))
    yaw = math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy**2 + qz**2))
    return roll, pitch, yaw

def euler2quat(roll, pitch, yaw):
    """Euler to quaternion [roll, pitch, yaw]
    retuns q = [qw qx qy qz]
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)

    qw = cy * cr * cp + sy * sr * sp
    qx = cy * sr * cp - sy * cr * sp
    qy = cy * cr * sp + sy * sr * cp
    qz = sy * cr * cp - cy * sr * sp
    return qw, qx, qy, qz