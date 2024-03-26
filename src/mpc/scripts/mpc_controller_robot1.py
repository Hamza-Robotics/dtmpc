#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import Path
from std_msgs.msg import String
from TNMPC import TNMPC as NMPC
from conversion_functions import path2numpy, quaternion_to_euler, numpy2path
from geometry_msgs.msg import PoseStamped, Twist
from dtmpc_interface.msg import Trajectory
import matplotlib.pyplot as plt
from std_msgs.msg import Float64MultiArray
class Mpc_Controller(Node):
    def __init__(self):
        super().__init__('mpccontrollerrobot1')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.MPC = NMPC()
        self.timer = self.create_timer(1/10, self.control_loop)
        self.timer2 = self.create_timer(1, self.plot)
        self.subscription_state = self.create_subscription(PoseStamped,'robot1/pose',self.state_callback,10)
        self.publisher_solutionx = self.create_publisher(Path, 'dtmpc/robot1/mpc/solution', 10)
        self.publisher_twist = self.create_publisher(Twist, 'robot1/cmd_vel', 10)
        self.solution_u = self.create_publisher(Float64MultiArray, "dtmpc/robot1/u_solution", 10)
        self.subscription_trajectory = self.create_subscription(Trajectory, 'trajectory/profile', self.trajectory_callback, 10)
        self.x_received=False
        self.traj_received=False
        self.traj=np.array([[0,0,0],[0,0,0]]).reshape(2,3)
        self.velocities=np.array([[0,0],[0,0]]).reshape(2,2)
        self.x_solution=np.zeros((105,9))
        self.begin_time=(self.get_clock().now())
        self.solutionplot_eo=[]
        self.solutionplot_ed=[]
        self.t=[]
        plt.ion()

        # Create a figure
        self.fig, self.ax = plt.subplots()

    def plot(self):
        t = (self.get_clock().now() - self.begin_time).nanoseconds / 1e9
        # Assuming self.x_solution is already defined and contains the data
        self.t.append(t)
        self.solutionplot_eo.append(self.x_solution[0, 0])
        self.solutionplot_ed.append(self.x_solution[0, 1])

        # Plot the data
        self.ax.plot(self.t, self.solutionplot_eo, label="e_o")
        self.ax.plot(self.t, self.solutionplot_ed, label="e_d")

        # Add labels and legend
        self.ax.set_xlabel('Time')
        self.ax.set_ylabel('Value')

        # Only show the legend once 
        if not self.ax.get_legend():
            self.ax.legend()
        
        self.ax.set_title('Last Two Columns Plotted Together')

        # Ensure that the lists are only 100 long
        self.t = self.t[-100:]
        self.solutionplot_eo = self.solutionplot_eo[-100:]
        self.solutionplot_ed = self.solutionplot_ed[-100:]


        self.fig.canvas.draw()

        # Show plot
        plt.pause(0.01)


    def control_loop(self):
        if self.x_received and self.traj_received:
            self.get_logger().info('Control loop')
            u,self.x_solution=self.MPC.controller(self.x,self.traj, self.velocities)
            msg_u=Float64MultiArray()
            msg_u.data=self.MPC.u_list.flatten().tolist()
            self.solution_u.publish(msg_u)

            #if self.MPC.solver_status==0:
            if True:
                #path_x=numpy2path(self,self.x)
                #self.publisher_solutionx.publish(path_x)
                Twist_msg=Twist()
                Twist_msg.linear.x=u[0,0]
                Twist_msg.angular.z=u[0,1]
                self.publisher_twist.publish(Twist_msg)
    def trajectory_callback(self,msg):
        t=Trajectory() 
        msg.velocities
        velocities = []
        for i in range(len(msg.velocities)):
            x=msg.velocities[i].x
            y=msg.velocities[i].y

            velocities.append([x,y])

        self.velocities=((np.asarray(velocities)))
        self.traj=path2numpy(msg.path)
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