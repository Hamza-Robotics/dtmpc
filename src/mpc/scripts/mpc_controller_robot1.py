#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from std_msgs.msg import String
from model_predictive_control import MPC


class Mpc_Controller(Node):

    def __init__(self):
        super().__init__('mpccontrollerrobot1')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.MPC = MPC()
        self.timer = self.create_timer(1/10, self.control_loop)
        self.subscription_trajectory = self.create_subscription(Path,'trailer/trajectory',self.trajectory_callback,10)
    def control_loop(self):
        self.MPC.set_controller_trajectory(traj)
        u,x=self.MPC.controller(x)

    def trajectory_callback(self,msg):
        pass

    


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