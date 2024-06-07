#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, Twist, PoseStamped
from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')

        
        self.subscription_1 = self.create_subscription(PoseStamped, 'robot1/pose', self.robot1_callback, 10)
        self.subscription_2 = self.create_subscription(PoseStamped, 'robot2/pose', self.robot2_callback, 10)
        self.subscription_3 = self.create_subscription(PoseStamped, 'robot3/pose', self.robot3_callback, 10)
        self.robot1=[0,0,0]
        self.robot2=[1,0,0]
        self.robot3=[2,0,0]

        
    def robot1_callback(self, msg):
        self.robot1[0]=msg.pose.position.x
        self.robot1[1]=msg.pose.position.y
        self.robot1[2]=msg.pose.position.z

    def robot2_callback(self, msg):
        self.robot2[0]=msg.pose.position.x
        self.robot2[1]=msg.pose.position.y
        self.robot2[2]=msg.pose.position.z

    def robot3_callback(self, msg):
        self.robot3[0]=msg.pose.position.x
        self.robot3[1]=msg.pose.position.y
        self.robot3[2]=msg.pose.position.z

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()