#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import Path
from std_msgs.msg import String
from ATNMPC import NMPC as NMPC
#from NMPC  import NMPC
from visualization_msgs.msg import MarkerArray, Marker

import time
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
        self.obstacles = []
        for i in range(self.MPC.numberofobs):
            self.obstacles.append([209,200,0.2])
        self.timer = self.create_timer(1/50, self.control_loop)
        self.subscription_state = self.create_subscription(PoseStamped,'robot1/pose',self.state_callback,10)
        self.publisher_solutionx = self.create_publisher(Path, 'dtmpc/robot1/mpc/solution', 10)
        self.publisher_twist = self.create_publisher(Twist, 'robot1/cmd_vel', 10)
        self.solution_u = self.create_publisher(Float64MultiArray, "dtmpc/robot1/u_solution", 10)
        self.subscription_trajectory = self.create_subscription(Trajectory, 'trajectory/profile', self.trajectory_callback, 10)
        self.subscription_obstacle = self.create_subscription(MarkerArray,'marker_array',self.obstacle_extractor,10)
        self.x=np.array([10,10,0]).reshape(1,3)
        self.x_received=False
        self.traj_received=False
        self.traj=np.array([[0,0,0],[0,0,0]]).reshape(2,3)
        self.velocities=np.array([[0,0],[0,0]]).reshape(2,2)
        self.x_solution=np.zeros((105,9))
        self.begin_time=(self.get_clock().now())
        self.solutionplot_eo=[]
        self.solutionplot_ed=[]
        self.t=[]


    def trajectory_make(self,Ts,N):
        traj=[]
        traj_dot=[]
        s=1
        for i in range(0,N):
            t=(Ts*i+time.time())
            
            x=3*np.sin(np.pi*t/25)
            y=3*np.cos(np.pi*t/25)
            xd=3*np.cos(np.pi*t/25)*(np.pi/25)
            yd=-3*np.sin(np.pi*t/25)*(np.pi/25)
            traj.append([x,y])  
            traj_dot.append([xd,yd])
        return np.asanyarray(traj),np.asanyarray(traj_dot)
    
    def control_loop(self):
        if self.x_received and self.traj_received:
            self.get_logger().info('Control loop')
            xr, xd = self.trajectory_make(0.1,self.MPC.N)  # Assuming trajectory() returns x values
            obs=[[2,6,0.5]]

            u,self.x_solution=self.MPC.controller(self.x,xr,xd,self.obstacles)

            #u,self.x_solution=self.MPC.controller(self.x,self.traj,self.velocities,obs)

            msg_u=Float64MultiArray()
            msg_u.data=self.MPC.u_list.flatten().tolist()
            self.solution_u.publish(msg_u)
            path_x=numpy2path(self,self.x_solution[:,0:3])
            self.publisher_solutionx.publish(path_x)
            #if self.MPC.solver_status==0:
            if True:

                Twist_msg=Twist()
                Twist_msg.linear.x=u[0,0]
                Twist_msg.angular.z=u[0,1]
                self.publisher_twist.publish(Twist_msg)
    
    def trajectory_callback(self,msg):
        t=Trajectory() 
        self.velocities=path2numpy(msg.velocities)
        self.traj=path2numpy(msg.path)
        #self._logger.info(f"Trajectory dimensions: {np.shape(traj)}")
        self.traj_received=True
    def state_callback(self,msg):
        rpy=quaternion_to_euler(msg.pose.orientation)
        self.x=np.array([msg.pose.position.x,msg.pose.position.y,rpy[2]]).reshape(1,3)

        self.x_received=True
    

    def obstacle_extractor(self, msg):
        msg = msg.markers
        obstacles=[]
        for marker in msg:
            #print(f"x: {marker.pose.position.x}, y: {marker.pose.position.y}, radius: {marker.scale.x}")
            obstacles.append([marker.pose.position.x,marker.pose.position.y,0.5])
            
        # Sort obstacles based on their distance from the current position
        sorted_obstacles = sorted(obstacles, key=lambda obs: np.linalg.norm(obs[:2] - self.x[0, :2]))

        # Get the closest n obstacles
        n = self.MPC.numberofobs

        # Get the number of obstacles needed
        self.obstacles =sorted_obstacles[:n]
        
        print(f"Obstacles: {self.obstacles}")

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