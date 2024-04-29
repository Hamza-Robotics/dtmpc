#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import String



class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('test_obstacle_node')
        self.subscription = self.create_subscription(MarkerArray,'marker_array',self.listener_callback,10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        msg = msg.markers
        for marker in msg:
            #print(f"x: {marker.pose.position.x}, y: {marker.pose.position.y}, radius: {marker.scale.x}")
            pass


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