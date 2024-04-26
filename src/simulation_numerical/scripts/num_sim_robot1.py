#!/usr/bin/env python3
from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion, Twist
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, PoseStamped, TransformStamped
import yaml
import numpy as np
class Robot():
    def __init__(self):
        self.x=0.0
        self.y=0.0
        self.th0=0.0
        self.v=0.0
        self.th_d=0.0

        self.hz=10

    def propagate(self,v,th_d,hz):
        self.th0=self.th0+(th_d)*(1/(hz))
        self.x=self.x+np.cos(self.th0)*v*(1/(hz))
        self.y=self.y+np.sin(self.th0)*v*(1/(hz))
    
    def get_state(self):
        return np.array([[self.x,self.y,self.th0]])
class StatePublisher(Node):

    def __init__(self):
        self.robot_name="robot1"
        self.x=0.0
        self.y=0.00
        self.th0=0.0
        self.v=0
        self.thd=0
        self.robot=Robot()
        with open('src/mpc/config/dnmpc_params.yaml') as file:
            yamlfile = yaml.safe_load(file)
        self.hz=60

        super().__init__(self.robot_name+'_statepublisher')

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, self.robot_name+'/joint_states', qos_profile)
        self.publish_state = self.create_publisher(PoseStamped, self.robot_name+'/pose', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.pose_publish = self.create_publisher(PoseStamped, 'topic2', 10)
        self.subscription = self.create_subscription(Twist,self.robot_name+'/cmd_vel',self.integrator,10)


        
        #self.nodeName = self.get_name()
        #self.get_logger().info("{0} started".format(self.nodeName))
        self.publishing_timer = self.create_timer(1.0 / self.hz, self.publisher_loop)  # Change 100 to your desired frequency (Hz)    

        self.timer = self.create_timer(1/self.robot.hz, self.publishstate)
        self.robot=Robot()
        self.pose = PoseStamped()
        self.pose.header.frame_id = 'map'
        self.joint_state = JointState()
        self.rviz_pose = TransformStamped()
        self.rviz_pose.header.frame_id = 'map'
        self.rviz_pose.child_frame_id = self.robot_name+'/base_link'
    def publishstate(self):
        self.robot.propagate(self.v,self.thd, self.robot.hz)
        pose = self.robot.get_state()
   
        pose_msg = PoseStamped()  # Fix: Add missing import for PoseStamped
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = pose[0,0]
        pose_msg.pose.position.y = pose[0,1]
        pose_msg.pose.position.z = 0.0
        q=euler_to_quaternion(0,0,pose[0,2])    
        pose_msg.pose.orientation=q
        #self.publisher_.publish(msg)
        self.pose_publish.publish(pose_msg)

    def integrator(self, msg):
        linear = msg.linear
        angular = msg.angular

        # Log the data using ROS 2 Python logger
        #self.get_logger().info("Linear: [x:"+str(msg.linear.x)+", y: "+str(msg.linear.y)+", z: "+str(msg.linear.z)+"], Angular: [x: "+str(msg.angular.x)+", y:"+str(msg.angular.y)+" , z: "+str(msg.angular.z)+"]")
        self.v=msg.linear.x
        self.thd=msg.angular.z


    def publisher_loop(self):
        x=self.robot.get_state()
        self.x=x[0,0]
        self.y=x[0,1]
        self.th0=x[0,2]
        now = self.get_clock().now()
        self.joint_state.header.stamp = now.to_msg()
        self.joint_state.name = [self.robot_name+'/wheel_right_joint', self.robot_name+'/wheel_left_joint']
        self.joint_state.position = [0.0,0.0]
        # (moving in a circle with radius=2)
        self.pose.header.stamp = now.to_msg()
        self.pose.pose.position.x = self.x
        self.pose.pose.position.y = self.y
        self.pose.pose.position.z = 0.0
        self.pose.pose.orientation= \
            euler_to_quaternion(0, 0, self.th0) # roll,pitch,yaw
        

        self.rviz_pose.transform.translation.x = self.x
        self.rviz_pose.transform.translation.y = self.y
        self.rviz_pose.transform.translation.z = 0.0
        self.rviz_pose.transform.rotation = \
            euler_to_quaternion(0, 0, self.th0) # roll,pitch,yaw


        # send the joint state and transform
        self.joint_pub.publish(self.joint_state)
        self.publish_state.publish(self.pose)
        self.broadcaster.sendTransform(self.rviz_pose)



def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)



def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = StatePublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()