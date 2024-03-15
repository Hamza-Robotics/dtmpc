#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('trajectory_generator')
        self.publisher_ = self.create_publisher(Path, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg=Path()
        for i in range(19):
            pose=PoseStamped()
            pose.pose.position.x=i*0.1
            pose.pose.position.y=i*0.01
            pose.pose.orientation.w=1.0
            pose.pose.orientation.y=0.0
            pose.pose.orientation.z=0.0
            pose.pose.orientation.x=0.0
            pose.header.frame_id="map"
            pose.header.stamp=self.get_clock().now().to_msg()

            msg.poses.append(pose)
        msg.header.frame_id="map"
        msg.header.stamp=self.get_clock().now().to_msg()
        self.publisher_.publish(msg)
        #self.get_logger().info('Publishing: "%s"' % msg.data)

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

print("HELLO")