#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, Twist, PoseStamped
from std_msgs.msg import String
import socket
import xml.etree.ElementTree as ET
import time


class Publisher_():
    def __init__(self,publisher,id):
        self.publisher=publisher    
        self.id=id
    

  
class MinimalPublisher(Node):



    def __init__(self):
        super().__init__('publish_vicon_state')
        self.publisher_1 = Publisher_(self.create_publisher(PoseStamped, 'robot1/pose', 10),"robot1")
        self.publisher_2 = Publisher_(self.create_publisher(PoseStamped, 'robot2/pose', 10),"robot2")
        self.publisher_3 = Publisher_(self.create_publisher(PoseStamped, 'robot3/pose', 10),"robot3")
        self.publishers_=[self.publisher_1,self.publisher_2,self.publisher_3]
        timer_period = 0.005  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):

        k=0
        for i in range(3):
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.pose.position.x = i+0.0
            msg.pose.position.y = 0.0
            msg.pose.position.z = 0.0
            msg.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            self.publishers_[k].publisher.publish(msg)
            k+=1

            print("Timer callback   ")

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