#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import Path
from std_msgs.msg import String
from ATNMPC import NMPC as NMPC
#from NMPC  import NMPC
from visualization_msgs.msg import MarkerArray, Marker
import sympy as sp
import time
from conversion_functions import path2numpy, quaternion_to_euler, numpy2path, path2numpy
from geometry_msgs.msg import PoseStamped, Twist, PointStamped
import matplotlib.pyplot as plt
from robot_system import get_trajectory 
from std_msgs.msg import Float64MultiArray
import time
import pickle
robot= 'robot1'
neighbor_robot1 = 'robot2'
neighbor_robot2 = 'robot3'
class Mpc_Controller(Node):
    def __init__(self):
        super().__init__('mpccontroller'+robot)
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.MPC = NMPC(robot)
        self.obstacles = []
        for i in range(self.MPC.numberofobs):
            self.obstacles.append([2,2,0.2])
        self.timer = self.create_timer(1/self.MPC.frequency, self.control_loop)
        self.subscription_state = self.create_subscription(PoseStamped,robot+'/pose',self.state_callback,10)
        self.subscription_state1 = self.create_subscription(PoseStamped,neighbor_robot1+'/pose',self.state_callback1,10)
        self.subscription_state2 = self.create_subscription(PoseStamped,neighbor_robot2+'/pose',self.state_callback2,10)
        self.subscription_obstacle = self.create_subscription(MarkerArray,'dtmpc/obstacle_list',self.obstacle_extractor,10)
  
        self.subscription_solution1= self.create_subscription(Path,'dtmpc/'+neighbor_robot1+'/mpc/solution',self.state_callback_robot1,10)
        self.subscription_solution2= self.create_subscription(Path,'dtmpc/'+neighbor_robot2+'/mpc/solution',self.state_callback_robot2,10)
        #self.publishing_timer = self.create_timer(1., self.datasaver)  # Change 100 to your desired frequency (Hz)    


        #self.MPC.N=105

    
        self.publisher_solutionx = self.create_publisher(Path, 'dtmpc/'+robot+'/mpc/solution', 10)
        self.publisher_twist = self.create_publisher(Twist, robot+'/cmd_vel', 10)
        self.solution_u = self.create_publisher(Float64MultiArray, "dtmpc/"+robot+"/u_solution", 10)
        self.publisher_point = self.create_publisher(PointStamped, 'dtmpc/trajectory/'+robot+'/point', 10)
        self.publisher_path = self.create_publisher(Path, 'dtmpc/trajectory/'+robot+'/viz', 10)
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
        self.x_traj,self.y_traj,self.xd_traj,self.yd_traj=get_trajectory(robot)
        self.xr=np.zeros((3,1))
        self.xd=np.zeros((3,1))
        self.u1_send=0
        self.u2_send=0
        self.pos1 = np.array([0,0,0.1]).reshape(1, 3)
        self.pos2 = np.array([0,0,0.1]).reshape(1, 3)
        self.robot1_pos = np.ones((self.MPC.N, 3)) * 0.1
        self.robot2_pos = np.ones((self.MPC.N, 3)) * 0.1
        self.robot1_pos_temp = np.ones((self.MPC.N, 3)) * 0.1
        self.robot2_pos_temp = np.ones((self.MPC.N, 3)) * 0.1
        self.xdot=np.zeros((self.MPC.N,2))
        self.time_robot1=0
        self.loop_frequency = 10
        self.position_loop= self.create_timer(1/10, self.pos_loop)
        self.outputlist=[]

    def datasaver(self):
        self.outputlist.append([self.x[0,:],self.xr[0,:],np.array([self.u1_send,self.u2_send]) ,time.time()])
        def pickle_data():
            with open('data/scenario1/data_'+robot+'_.pickle', 'wb') as file:
                print("Saved")
                pickle.dump(self.outputlist, file)
            #self.outputlist=[]
        self.create_timer(180.0, pickle_data)


    def convert_trajectory(self, trajectory):
        converted_trajectory = []
        if robot=='robot1':
            for point in trajectory:
                x = point[0]
                y = point[1]
                converted_trajectory.append([x, y])
        if robot=='robot2':
            for point in trajectory:                
                phi=np.rad2deg(-50)
                x = np.sin(phi-point[2]) + point[0]
                y = np.cos(phi-point[2]) + point[1]
                converted_trajectory.append([x, y])
        if robot=='robot3':
            for point in trajectory:
                phi=np.rad2deg(50)
                x = -np.sin(phi-point[2]) + point[0]
                y = -np.cos(phi-point[2]) + point[1]
                converted_trajectory.append([x, y])
        return np.array(converted_trajectory)

    def pos_loop(self):
        current_time = self.get_clock().now().to_msg().sec
        time_diff=current_time-self.time_robot1
        if time_diff>2 or robot=='robot1':
            #self.robot1_pos = np.tile(self.pos1, (self.MPC.N, 1))
            #self.robot2_pos = np.tile(self.pos2, (self.MPC.N, 1))
            pass
        if time_diff>2:
            self.robot1_pos = np.tile(self.pos1, (self.MPC.N, 1))
            self.robot2_pos = np.tile(self.pos2, (self.MPC.N, 1))
            self.xdot=np.zeros((self.MPC.N,2))
            
        if time_diff<2 and robot=='robot2':
            self.robot1_pos = self.robot1_pos_temp
            self.robot2_pos_temp = np.ones((self.MPC.N, 3)) * 0.1
       
        if time_diff<2 and robot=='robot3':
            self.robot1_pos = self.robot1_pos_temp
            self.robot2_pos = self.robot2_pos_temp
  
    def state_callback_robot1(self,msg):
        current_time = self.get_clock().now().to_msg().sec
        self.time_robot1= msg.header.stamp.sec
        self.robot1_pos_temp=path2numpy(msg)
        pass
        
    def state_callback_robot2(self,msg):
        #self.robot2_pos=path2numpy(msg)
        self.robot2_pos_temp=path2numpy(msg)
        pass
    
    def trJ(self,t):
        x=self.x_traj(t)
        y=self.y_traj(t)
        xd=self.xd_traj(t)
        yd=self.yd_traj(t)
        return x,y,xd,yd    

    def trajectory_make(self,Ts,N):
        if robot=='robot1':
            path=Path()
            path.header.stamp = self.get_clock().now().to_msg()
            path.header.frame_id = 'map'
            traj=[]
            traj_dot=[]
            current_time=time.time()
            s=1
            for i in range(0,N):
                t=(Ts*i+current_time)   
                
                x=self.x_traj(t)
                y=self.y_traj(t)
                xd= self.xd_traj(t)
                yd= self.yd_traj(t)
                point_msg = PoseStamped()
                point_msg.pose.position.x=x
                point_msg.pose.position.y=y
                path.poses.append(point_msg)
                traj.append([x,y])  
                traj_dot.append([xd,yd])
            point = PointStamped()
            point.header.stamp = self.get_clock().now().to_msg()
            point.header.frame_id = 'map'
            point.point.x = self.x_traj(current_time)
            point.point.y = self.y_traj(current_time)
            self.publisher_point.publish(point)
            self.publisher_path.publish(path)
                
            return np.asanyarray(traj),np.asanyarray(traj_dot)
        else:
            path=Path()
            path.header.stamp = self.get_clock().now().to_msg()
            path.header.frame_id = 'map'
            traj=[]
            traj_dot=[]
            current_time=time.time()
            s=1
            xd=self.convert_trajectory(self.robot1_pos)
            current_time = self.get_clock().now().to_msg().sec
            time_diff=current_time-self.time_robot1
            for i in range(0,N):
                
                t=(Ts*i+current_time)   
                xdot= self.xd_traj(t)
                ydot= self.yd_traj(t)
                
                point_msg = PoseStamped()
                point_msg.pose.position.x=xd[i,0]
                point_msg.pose.position.y=xd[i,1]
                path.poses.append(point_msg)
                traj.append([xd[i,0],xd[i,1]])  
                if time_diff<2:
                    traj_dot.append([xdot, ydot])
                else:       
                    traj_dot.append([0, 0])    

            point = PointStamped()
            point.header.stamp = self.get_clock().now().to_msg()
            point.header.frame_id = 'map'
            point.point.x = xd[0,0]
            point.point.y = xd[0,1]
            self.publisher_point.publish(point)
            self.publisher_path.publish(path)
                
            return np.asanyarray(traj),np.asanyarray(traj_dot)
    
    def control_loop(self):
        if self.x_received:
            self.get_logger().info('Control loop')
            self.xr, self.xd = self.trajectory_make(1/self.MPC.frequency,self.MPC.N)  # Assuming trajectory() returns x values
            obs=[[2,6,0.5]]

            u,self.x_solution=self.MPC.controller(self.x,self.xr,self.xd,self.obstacles,self.robot1_pos,self.robot2_pos,self.MPC.frequency)

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
                self.u1_send=u[0,0]
                Twist_msg.angular.z=u[0,1]
                self.u2_send=u[0,1]
                self.publisher_twist.publish(Twist_msg)  

    def state_callback1(self,msg):
        self.pos1 = np.array([msg.pose.position.x, msg.pose.position.y, quaternion_to_euler(msg.pose.orientation)[2]]).reshape(1, 3)
    
    def state_callback2(self,msg):
        self.pos2 = np.array([msg.pose.position.x, msg.pose.position.y, quaternion_to_euler(msg.pose.orientation)[2]]).reshape(1, 3)
        pass
        
    def state_callback(self,msg):
        rpy=quaternion_to_euler(msg.pose.orientation)
        self.x=np.array([msg.pose.position.x,msg.pose.position.y,rpy[2]]).reshape(1,3)
        self.x_received=True

    def obstacle_extractor(self, msg):
        msg = msg.markers
        obstacles=[]
        for marker in msg:
            #print(f"x: {marker.pose.position.x}, y: {marker.pose.position.y}, radius: {marker.scale.x}")
            obstacles.append([marker.pose.position.x,marker.pose.position.y,marker.scale.x/2])
            
        # Sort obstacles based on their distance from the current position
        sorted_obstacles = sorted(obstacles, key=lambda obs: np.linalg.norm(obs[:2] - self.x[0, :2]))

        # Get the closest n obstacles
        n = self.MPC.numberofobs

        # Get the number of obstacles needed
        self.obstacles =sorted_obstacles[:n]
        

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