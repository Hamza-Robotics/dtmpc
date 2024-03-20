#!/usr/bin/env python3
#20/3 14:40
from math import cos, sin, atan2, asin
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np



def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

def quaternion_to_euler(quaternion):
    roll = atan2(2 * (quaternion.w * quaternion.x + quaternion.y * quaternion.z), 1 - 2 * (quaternion.x**2 + quaternion.y**2))
    pitch = asin(2 * (quaternion.w * quaternion.y - quaternion.z * quaternion.x))
    yaw = atan2(2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y), 1 - 2 * (quaternion.y**2 + quaternion.z**2))
    return roll, pitch, yaw

def convert_to_path_stamped(self,traj):
    path=Path()
    for i in range(len(traj)):
        pose = PoseStamped()
        pose.pose.position.x = traj[i,0]
        pose.pose.position.y = traj[i,1]

        pose.pose.orientation= \
        euler_to_quaternion(0, 0, traj[i,2])
        pose.header.frame_id="map"
        pose.header.stamp=self.get_clock().now().to_msg()
        path.poses.append(pose)

    path.header.frame_id="map"
    path.header.stamp=self.get_clock().now().to_msg()
    return path

path=Path()
len(path.poses)
def convert_to_numpy(path):
    len(path.poses)
    numpy_traj = np.zeros((len(path.poses), 3))
    traj=path.poses
    for i in range(len(traj)):
        numpy_traj[i, 0] = traj[i].pose.position.x
        numpy_traj[i, 1] = traj[i].pose.position.y
        roll, pitch, yaw = quaternion_to_euler(traj[i].pose.orientation)
        numpy_traj[i, 2] = yaw
    return numpy_traj