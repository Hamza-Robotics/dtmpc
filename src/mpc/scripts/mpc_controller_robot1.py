#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import Path
from std_msgs.msg import String
from TNMPC import TNMPC as NMPC
from conversion_functions import path2numpy, quaternion_to_euler, numpy2path
from geometry_msgs.msg import PoseStamped, Twist

import matplotlib.pyplot as plt
class Mpc_Controller(Node):

    def __init__(self):
        super().__init__('mpccontrollerrobot1')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.MPC = NMPC()
        self.timer = self.create_timer(1/10, self.control_loop)
        self.timer2 = self.create_timer(1/10, self.plot)
        self.subscription_trajectory = self.create_subscription(Path,'/dtmpc/robot1/trajectory_feasbile',self.trajectory_callback,10)
        self.subscription_state = self.create_subscription(PoseStamped,'robot1/pose',self.state_callback,10)
        self.publisher_solutionx = self.create_publisher(Path, 'dtmpc/robot1/mpc/solution', 10)
        self.publisher_twist = self.create_publisher(Twist, 'robot1/cmd_vel', 10)
        self.x_received=False
        self.traj_received=False
        self.traj=np.array([[0,0,0],[0,0,0]]).reshape(2,3)
        self.x_solution=np.zeros((105,9))


        plt.ion()

        # Create a figure
        self.fig, self.ax = plt.subplots()

    def plot(self):
        # Assuming self.x_solution is already defined and contains the data

        # Clear the previous plot
        self.ax.clear()

        # Plot each column separately
        names=["e_o","e_d"]
        for i in range(len(self.x_solution.T)):  # Loop through each column
            #column_data = self.x_solution[:, -2 + i]  # Extract data for the ith column
            column_data = self.x_solution[:, i]  # Extract data for the ith column
            self.ax.plot(column_data, label='x['+str(i)+']')  # Plot the data    

        # Add labels and legend
        self.ax.set_xlabel('Index')
        self.ax.set_ylabel('Value')
        self.ax.legend()
        self.ax.set_title('Last Two Columns Plotted Separately')

 
        self.fig.canvas.draw()

        # Show plot
        plt.pause(0.01) 


    def control_loop(self):
        if self.x_received and self.traj_received:
            self.get_logger().info('Control loop')
            self.get_logger().info(str(self.MPC.N))
            u,self.x_solution=self.MPC.controller(self.x,self.traj)
            print(np.shape(self.x_solution))
            if False:
                path_x=numpy2path(self,self.x)
                self.publisher_solutionx.publish(path_x)
                Twist_msg=Twist()
                Twist_msg.linear.x=u[0,0]
                Twist_msg.angular.z=u[0,1]
                self.publisher_twist.publish(Twist_msg)
    def trajectory_callback(self,msg):
    
        self.traj=path2numpy(msg)
        #self._logger.info(f"Trajectory dimensions: {np.shape(traj)}")
        self.traj_received=True
    def state_callback(self,msg):
        rpy=quaternion_to_euler(msg.pose.orientation)
        self.x=np.array([msg.pose.position.x,msg.pose.position.y,rpy[2]]).reshape(1,3)
        self.x_received=True
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