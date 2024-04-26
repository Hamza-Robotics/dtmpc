#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose
from conversion_functions import euler_to_quaternion, numpy2path
from geometry_msgs.msg import PoseStamped, Twist
import time
from ATNMPC import NMPC as NMPC

# Rest of the code...

class Robot():
    def __init__(self):
        self.x=0
        self.y=0
        self.th0=0
        self.v=0
        self.th_d=0

        self.hz=50

    def propagate(self,v,th_d,hz):
        self.th0=self.th0+(th_d)*(1/(hz))
        self.x=self.x+np.cos(self.th0)*v*(1/(hz))
        self.y=self.y+np.sin(self.th0)*v*(1/(hz))
    
    def get_state(self):
        return np.array([[self.x,self.y,self.th0]])
class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.robot=Robot()
        self.publisher_ = self.create_publisher(Path, 'topicw', 10)
        self.pose_publish = self.create_publisher(PoseStamped, 'topic2', 10)
        self.publisher_solutionx = self.create_publisher(Path, 'dtmpc/robot1/mpc/solution', 10)
        self.subscription = self.create_subscription(PoseStamped, 'state', self.pose_callback, 10)
        self.publisher_twist = self.create_publisher(Twist, 'robot1/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.i = 0
        self.robot.hz=10
        self.timer = self.create_timer(1/20, self.publishtrajectory)
        self.timer = self.create_timer(1/20, self.controller)
        
        self.MPC = NMPC()
        self.v=1
        self.thd=0.5
        self.x,self.y,self.th0=0,0,0
        self.xd,self.xd_dot=self.trajectory(0.1,100)
        

    def trajectory(self,Ts,N):
        traj=[]
        traj_dot=[]
        s=1
        for i in range(0,N):
            t=(Ts*i+time.time())
            
            x=3*np.sin(np.pi*t/25) +5
            y=3*np.cos(np.pi*t/25)+5
            xd=3*np.cos(np.pi*t/25)*(np.pi/25)
            yd=-3*np.sin(np.pi*t/25)*(np.pi/25)
            traj.append([x,y])  
            traj_dot.append([xd,yd])
        return np.asanyarray(traj),np.asanyarray(traj_dot)


    def controller(self):
        obs=[[20,6,0.5]]
        u,self.x_solution=self.MPC.controller(np.array([[self.x,self.y,self.th0]]),self.xd,self.xd_dot,obs)
        self.v=u[0,0]
        self.thd=u[0,1]
        path_x=numpy2path(self,self.x_solution[:,0:3])
        self.publisher_solutionx.publish(path_x)
        Twist_msg=Twist()
        Twist_msg.linear.x=u[0,0]
        Twist_msg.angular.z=u[0,1]
        self.publisher_twist.publish(Twist_msg)
        
    def pose_callback(self, msg):  
        self.x=msg.pose.position.x
        self.y=msg.pose.position.y
        q=msg.pose.orientation
        r,p,y=euler_to_quaternion(q.x,q.y,q.z,q.w)
        self.th0=y
 
    def publishtrajectory(self):
        self.xd,self.xd_dot=self.trajectory(0.1,100)
        path=numpy2path(self,self.xd)
        self.publisher_.publish(path)







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