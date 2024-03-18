#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from model_predictive_control import MPC



class Mpc_Controller(Node):

    def __init__(self):
        super().__init__('mpccontrollerrobot1')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.mpc = MPC()
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    mpc_Controller = Mpc_Controller()

    rclpy.spin(mpc_Controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mpc_Controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()