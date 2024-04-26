#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import String
import yaml
from geometry_msgs.msg import PointStamped, PoseStamped, Vector3
from dtmpc_interface.msg import Trajectory
from nav_msgs.msg import Path
import time
import sympy as sp
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
        self.x,self.y,self.xd,self.yd=self.trajectory_robot1()
        with open('src/mpc/config/dnmpc_params.yaml') as file:
            yamlfile = yaml.safe_load(file)
        frequency=yamlfile['Prediction_Frequency']
        prediction_Length=30
        self.N=int(prediction_Length*frequency)
        self.samplingtime=1/frequency
    def timer_callback(self):
        t = time.time()
        x,y,xd,yd=self.trJ(t)
        point_msg = PointStamped()
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.header.frame_id = 'map'
        point_msg.point.x = x
        point_msg.point.y = y        

        self.publisher_.publish(point_msg)
        path=Path()
        path.header.stamp = self.get_clock().now().to_msg() 
        trajectory=Trajectory()
        velocities = Path()

        path.header.frame_id = 'map'
        for i in range(0,self.N):
            point_msg = PoseStamped()
            vel_msg = PoseStamped()
            x,y,yd,xd=self.trJ(t+i*self.samplingtime)
            point_msg.pose.position.x=x
            point_msg.pose.position.y=y

            vel_msg.pose.position.x=xd
            vel_msg.pose.position.y=yd
            

  
            velocities.poses.append(vel_msg)
            path.poses.append(point_msg)

        trajectory.path=path
        trajectory.velocities=velocities
        
        self.publisher_path.publish(path)
        self.publisher_trajectory.publish(trajectory)
    
    
    def trJ(self,t):
        x=self.x(t)
        y=self.y(t)
        xd=self.xd(t)
        yd=self.yd(t)
        return x,y,xd,yd    
    
    def trajectory_robot1(self):  
        
        t=sp.symbols('t')    
        x=3*sp.sin(sp.pi*t/25)
        y=3*sp.cos(sp.pi*t/25)
        xd=3*sp.cos(sp.pi*t/25)*(sp.pi/25)
        yd=-3*sp.sin(sp.pi*t/25)*(sp.pi/25)
        
        x = sp.lambdify(t, x)
        y = sp.lambdify(t, y)
        xd = sp.lambdify(t, xd)
        yd = sp.lambdify(t, yd)
        

        return x,y ,xd,yd
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