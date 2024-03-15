#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped



class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('pseodu_state_publising')
        self.publisher_ = self.create_publisher(PoseStamped, 'pose', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
    
        self.waypoint = np.array([[-3.7,-2.1, 0,0,0],
                         [-5,-1.650, 0.0,0.0,0.0], 
                         [-6.21,0.4, -np.pi/2,-np.pi/2,0],
                         [-3.1,2.3,-np.pi,-np.pi,0],
                        [-1.25,-1.74,-np.pi/2-np.pi,-np.pi/2-np.pi,0],
                         [3.2, -4.1, -np.pi,-np.pi,0],
                         [11,-5.8, -np.pi,-np.pi,0],
                         [12.08,-0.93, -np.pi/2,-np.pi/2,0],
                         [12.05,-5.4, -np.pi/2,-np.pi/2,0],
                         [2.6,-2.8, -np.pi/2,-np.pi/2,0]])

    def timer_callback(self):
        msg = PoseStamped()
        msg.pose.position.x=0.0
        msg.pose.position.y=0.0
        msg.pose.position.z=0.0
        msg.pose.orientation.x=1.0
        msg.pose.orientation.y=0.0
        msg.pose.orientation.z=0.0
        msg.pose.orientation.w=0.0
        msg.header.frame_id="map"
        msg.header.stamp=self.get_clock().now().to_msg()
        self.publisher_.publish(msg)


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