#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import socket

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('moror_commander_robot1')
        self.subscriber_=self.create_subscription(Twist, '/robot1/cmd_vel', self.listener_callback, 10)

        # Set up UDP socket
        self.udp_ip = "192.168.1.68"  # IP address of the target machine
        self.udp_port = 5005        # Port number
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def listener_callback(self, msg):
        linear_x = msg.linear.x
        self.get_logger().info('I heard: "%f"' % linear_x)
        
        # Send the data as an array
        data = [msg.linear.x, msg.angular.z]
        data_bytes = str(data).encode('utf-8')
        self.sock.sendto(data_bytes, (self.udp_ip, self.udp_port))

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
