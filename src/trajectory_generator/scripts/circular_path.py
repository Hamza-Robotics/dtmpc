#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import String
import yaml
from geometry_msgs.msg import PointStamped, PoseStamped, Vector3
from dtmpc_interface.msg import Trajectory
from nav_msgs.msg import Path
class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.begin_time = self.get_clock().now()
        self.publisher_ = self.create_publisher(PointStamped, 'trajectory/point', 10)
        self.publisher_path = self.create_publisher(Path, 'trajectory/viz', 10)
        self.publisher_trajectory = self.create_publisher(Trajectory, 'trajectory/profile', 10)

        with open('src/mpc/config/mpc_params.yaml') as file:
            yamlfile = yaml.safe_load(file)
        frequency=yamlfile['Prediction_Frequency']
        prediction_Length=yamlfile['Prediction_Length']
        self.N=int(prediction_Length*frequency)
        self.samplingtime=1/frequency
    def timer_callback(self):
        t = (self.get_clock().now() - self.begin_time).nanoseconds / 1e9
        x,y=self.trJ(t)
        point_msg = PointStamped()
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.header.frame_id = 'map'
        point_msg.point.x = x
        point_msg.point.y = y
        self.publisher_.publish(point_msg)
        path=Path()
        path.header.stamp = self.get_clock().now().to_msg() 
        trajectory=Trajectory()
        velocities = []

        path.header.frame_id = 'map'
        for i in range(0,self.N):
            point_msg = PoseStamped()
            x,y=self.trJ(t+i*self.samplingtime)
            point_msg.pose.position.x=x
            point_msg.pose.position.y=y

            vel=Vector3()
            vel.x=3*(np.pi/25)*np.cos(np.pi*(t+i*self.samplingtime))
            vel.y=-3*(np.pi/25)*np.sin(np.pi*(t+i*self.samplingtime))
            velocities.append(vel)

            path.poses.append(point_msg)

        trajectory.path=path
        trajectory.velocities=velocities
        
        self.publisher_path.publish(path)
        self.publisher_trajectory.publish(trajectory)
    def trJ(self,t):
        x=3*np.sin(np.pi*t/25)
        y=3*np.cos(np.pi*t/25)
        return x,y
def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()