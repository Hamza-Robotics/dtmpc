#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import socket
import time
import ast

class UdpToCmdVel(Node):

    def __init__(self):
        super().__init__('udp_to_cmd_vel')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Set up UDP socket
        self.udp_ip = "192.168.1.68" # IP address to listen on
        self.udp_port = 5005       # Port number to listen on
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udp_ip, self.udp_port))
        self.sock.setblocking(False)

        self.last_received_time = time.time()

        # Timer to check for zero input publishing
        self.timer = self.create_timer(0.1, self.check_last_received)

    def check_last_received(self):
        current_time = time.time()
        if current_time - self.last_received_time > 2.0:
            # Publish zero input Twist message
            zero_twist = Twist()
            self.publisher_.publish(zero_twist)
            self.get_logger().info('Published zero Twist message due to timeout')

    def recv_udp(self):
        try:
            data, _ = self.sock.recvfrom(1024)  # Buffer size is 1024 bytes
            self.last_received_time = time.time()
            self.get_logger().info(f'Received UDP message: {data}')
            try:
                data_list = ast.literal_eval(data.decode('utf-8'))
                twist_msg = Twist()
                twist_msg.linear.x = data_list[0]
                twist_msg.angular.z = data_list[1]
                self.publisher_.publish(twist_msg)
                self.get_logger().info(f'Published Twist message: {twist_msg}')
             
            except (ValueError, IndexError) as e:
                self.get_logger().error(f'Error parsing UDP message: {e}')
        except BlockingIOError:
            # No data received
            pass

def main(args=None):
    rclpy.init(args=args)

    udp_to_cmd_vel = UdpToCmdVel()

    while rclpy.ok():
        udp_to_cmd_vel.recv_udp()
        rclpy.spin_once(udp_to_cmd_vel, timeout_sec=0.1)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    udp_to_cmd_vel.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()