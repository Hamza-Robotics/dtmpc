#!/usr/bin/env python3
from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion, Twist, PoseStamped
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped
from nav_msgs.msg import Path
import numpy as np
import yaml
import time 
class StatePublisher(Node):

    def __init__(self):
        self.robot_name="robot1"
        self.x=2.4
        self.y=2.00
        self.th0=0.0
        self.hz=10
        self.linear_x=0
        self.angular_z=0
        with open('src/mpc/config/dnmpc_params.yaml') as file:
            yamlfile = yaml.safe_load(file)
        self.communication_range = yamlfile['communication_range']
        super().__init__(self.robot_name+'_statepublisher')

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, self.robot_name+'/joint_states', qos_profile)
        self.publish_transforms = self.create_publisher(TransformStamped, self.robot_name+'/transformed_stamped', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.publish_state= self.create_publisher(PoseStamped, self.robot_name+'/pose', qos_profile)
        self.subscription = self.create_subscription(Twist,self.robot_name+'/cmd_vel',self.integrator,10)
        self.subscription  # prevent unused variable warning
        self.publish_comrange = self.create_publisher(Path, 'dtmpc/'+self.robot_name+'/communication_range', 10)

        
        #self.nodeName = self.get_name()
        #self.get_logger().info("{0} started".format(self.nodeName))
        self.publishing_timer = self.create_timer(1.0 / self.hz, self.publisher_loop)  # Change 100 to your desired frequency (Hz)    


        # message declarations
        self.odom_trans = TransformStamped()
        self.odom_trans.header.frame_id = 'map'
        self.odom_trans.child_frame_id = self.robot_name+'/base_link'
        self.joint_state = JointState()

    def integrator(self, msg):
        linear = msg.linear
        angular = msg.angular

        # Log the data using ROS 2 Python logger
        #self.get_logger().info("Linear: [x:"+str(msg.linear.x)+", y: "+str(msg.linear.y)+", z: "+str(msg.linear.z)+"], Angular: [x: "+str(msg.angular.x)+", y:"+str(msg.angular.y)+" , z: "+str(msg.angular.z)+"]")
        self.linear_x=msg.linear.x
        self.angular_z=msg.angular.z

    def publisher_loop(self):
        self.x=self.x+(cos(self.th0)*self.linear_x+0.05*sin((pi/15)*time.time()  ))*1/(self.hz)
        self.y=self.y+(sin(self.th0)*self.linear_x+0.05*cos((pi/15)*time.time()  ))*1/(self.hz)
        self.th0=self.th0+self.angular_z*1/(self.hz)
        now = self.get_clock().now()
        self.joint_state.header.stamp = now.to_msg()
        self.joint_state.name = [self.robot_name+'/wheel_right_joint', self.robot_name+'/wheel_left_joint']
        self.joint_state.position = [0.0,0.0]
        # (moving in a circle with radius=2)
        self.odom_trans.header.stamp = now.to_msg()
        self.odom_trans.transform.translation.x = self.x
        self.odom_trans.transform.translation.y = self.y
        self.odom_trans.transform.translation.z = 0.0
        self.odom_trans.transform.rotation = \
            euler_to_quaternion(0, 0, self.th0) # roll,pitch,yaw
        pose = PoseStamped()
        pose.header.stamp = now.to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = 0.0
        pose.pose.orientation = euler_to_quaternion(0, 0, self.th0)
        
        self.publish_state.publish(pose)
        
        # send the joint state and transform
        self.joint_pub.publish(self.joint_state)
        self.publish_transforms.publish(self.odom_trans)
        self.broadcaster.sendTransform(self.odom_trans)
        
        
        path=Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'
        for i in np.linspace(0, 2*np.pi, num=int(np.pi / 0.1)):
            point_msg = PoseStamped()
            point_msg.pose.position.x=self.communication_range*np.cos(i)+self.x
            point_msg.pose.position.y=self.communication_range*np.sin(i)+self.y   
            path.poses.append(point_msg)
        self.publish_comrange.publish(path)


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