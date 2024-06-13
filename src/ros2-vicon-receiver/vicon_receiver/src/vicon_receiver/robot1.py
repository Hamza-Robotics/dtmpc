#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from vicon_receiver.msg import Position
from std_msgs.msg import String
from geometry_msgs.msg import Quaternion, Twist, PoseStamped

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('vicon_pkg_to_ros2_pose')
        self.subscriber=self.create_subscription(Position, '/vicon/robot1/robot1', self.listener_callback, 10)
        self.publisher_ = self.create_publisher(PoseStamped, 'robot1/pose', 10)

    def listener_callback(self, msg):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = msg.x_trans/1000
        pose_msg.pose.position.y = msg.y_trans/1000
        pose_msg.pose.position.z = msg.z_trans/1000
        pose_msg.pose.orientation.x = msg.x_rot
        pose_msg.pose.orientation.y = msg.y_rot
        pose_msg.pose.orientation.z = msg.z_rot
        pose_msg.pose.orientation.w = msg.w
        self.publisher_.publish(pose_msg)
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