#!/usr/bin/env python3
from math import sin, cos, pi

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
from traj_gen import trajectory_genertor
import matplotlib.pyplot as plt
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class ArraySubscriber(Node):
    def __init__(self):
        super().__init__('trajectory_generator')
        self.subscription_waypoint = self.create_subscription(Float64MultiArray,'dtmpc/waypoint/robot1',self.waypoint_callback,10)
        self.subscription_state = self.create_subscription(PoseStamped,'robot1/pose',self.state_callback,10)
        
        self.gen=trajectory_genertor(max_v=1,prediction_Length=10,frequency=10,nodes=100)
        self.waypoints = np.array([[0.001,0.001,0],[0.001,0.001,0],[0.003,0.003,0]]).reshape(-1, 3)
        self.gen.waypoint=(self.waypoints)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.publisher_feasbile = self.create_publisher(Path, 'dtmpc/robot1/trajectory_feasbile', 10)
        self.publisher_full = self.create_publisher(Path, 'dtmpc/robot1/trajectory_full', 10)
        self.x=np.array([[0,0,0]])
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
    
    def timer_callback(self):
        if self.waypoints[0][0] != 190000091:
            path,path_feasible,index=self.gen.trajectory(self.x)

            path_msg=self.convert_to_path_stamped(path)
            path_feasbile_msg=self.convert_to_path_stamped(path_feasible)
            self.publisher_full.publish(path_msg)
            self.publisher_feasbile.publish(path_feasbile_msg)

    def waypoint_callback(self, msg):
        data = msg.data
        self.waypoints = np.array(data).reshape(-1, 3)
        self.gen.waypoint=(self.waypoints)
        self.get_logger().info('I heard: "%s"' % self.waypoints)
        
    def state_callback(self, msg):
        self.x=np.array([[msg.pose.position.x,msg.pose.position.y,0]])
        self._logger.info('I heard: "%s"' % self.x)
def main(args=None):
    rclpy.init(args=args)
    array_subscriber = ArraySubscriber()
    rclpy.spin(array_subscriber)
    array_subscriber.destroy_node()
    rclpy.shutdown()

def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

if __name__ == '__main__':
    main()
