#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path   
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist, PointStamped
import numpy as np  

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Path, 'dtmpc/formation/robot1to2', 10)
        self.publisher_2 = self.create_publisher(Path,'dtmpc/formation/robot1to3', 10)
        
        self.subscription_state1 = self.create_subscription(PoseStamped,'robot1/pose',self.robot_callback,10)
        self.subscription_state2 = self.create_subscription(PoseStamped,'robot2/pose',self.state_callback2,10)
        self.subscription_state2 = self.create_subscription(PoseStamped,'robot3/pose',self.state_callback3,10)

        self.timer = self.create_timer(1/15, self.timer_callback)
        self.i = 0
        self.pos2= np.array([0.0, 0.0]).reshape(1, 2)
        self.pos3= np.array([0.0, 0.0]).reshape(1, 2)
        self.robot=np.array([0.0,0.0]).reshape(1,2)
    def timer_callback(self):

        path_1=Path()
        path_2=Path()
        pose1=PoseStamped()
        pose2=PoseStamped()
        pose3=PoseStamped()
        
        
        path_1.header.frame_id='map'
        path_1.header.stamp=self.get_clock().now().to_msg()
        path_2.header.frame_id='map'
        path_2.header.stamp=self.get_clock().now().to_msg()
        pose1.header.frame_id='map'
        pose2.header.frame_id='map'
        pose3.header.frame_id='map'
        pose1.pose.position.x=self.robot[0,0]
        pose1.pose.position.y=self.robot[0,1]
        
        pose2.pose.position.x=self.pos3[0,0]
        pose2.pose.position.y=self.pos3[0,1]
        
        pose3.pose.position.x=self.pos2[0,0]
        pose3.pose.position.y=self.pos2[0,1]
        
        
        
        path_1.poses.append(pose1)
        path_1.poses.append(pose2)

        path_2.poses.append(pose1)
        path_2.poses.append(pose3)
        self.publisher_2.publish(path_2)
        self.publisher_.publish(path_1)

        


    def state_callback2(self,msg):
        self.pos2 = np.array([msg.pose.position.x, msg.pose.position.y]).reshape(1, 2)
    def state_callback3(self,msg):
        self.pos3 = np.array([msg.pose.position.x, msg.pose.position.y]).reshape(1, 2)
    def robot_callback(self,msg):
        self.robot=np.array([msg.pose.position.x,msg.pose.position.y]).reshape(1,2)

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