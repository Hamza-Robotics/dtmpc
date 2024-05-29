#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Quaternion, Twist
import numpy as np  
class obstacle_node(Node):

    def __init__(self):
        super().__init__('obstacle_node')
        self.publisher_ = self.create_publisher(MarkerArray, 'dtmpc/obstacle_list', 10)
        timer_period = 0.1  # seconds
        self.subscription1 = self.create_subscription(Twist, 'dtmpc/obstacle/propagate_obstacle1', self.twist_obs1, 10)
        self.subscription2 = self.create_subscription(Twist, 'dtmpc/obstacle/propagate_obstacle2', self.twist_obs2, 10)
        self.subscription3 = self.create_subscription(Twist, 'dtmpc/obstacle/propagate_obstacle3', self.twist_obs3, 10)

        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.timer = self.create_timer(timer_period, self.propagate_obstacle1)

        self.i = 0
        self.hz=10
        self.obstacle1=np.array([1.0,4.0,0.5])
        self.obstacle1_xd=np.array([0,0])
        self.obstacle2=np.array([-2.0,2.0,0.5])
        self.obstacle2_xd=np.array([0,0])
        self.obstacle3=np.array([0.0,0.0,1.0])
        self.obstacle3_xd=np.array([0,0])

        
    def marker_maker(self,x,y,radius,marker_id):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = radius
        marker.scale.y = radius
        marker.scale.z = 0.5
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.id = marker_id
        return marker
        
    def timer_callback(self):
        msg = MarkerArray()
        msg.markers.append(self.marker_maker(self.obstacle1[0], self.obstacle1[1], self.obstacle1[2], 0))
        msg.markers.append(self.marker_maker(self.obstacle2[0], self.obstacle2[1], self.obstacle2[2], 1))
        msg.markers.append(self.marker_maker(self.obstacle3[0], self.obstacle3[1], self.obstacle3[2], 2))
        self.publisher_.publish(msg)

    def twist_obs1(self,msg):
        self.obstacle1_xd=np.array([msg.linear.x,msg.angular.z])
    def twist_obs2(self,msg):
        self.obstacle2_xd=np.array([msg.linear.x,msg.angular.z])
    def twist_obs3(self,msg):
        self.obstacle3_xd=np.array([msg.linear.x,msg.angular.z])


    def propagate_obstacle1(self):
        self.obstacle1[0] = self.obstacle1[0] + self.obstacle1_xd[0] * (1 / self.hz)
        self.obstacle1[1] = self.obstacle1[1] + self.obstacle1_xd[1] * (1 / self.hz)
        self.obstacle1[2] = self.obstacle1[2]

        self.obstacle2[0] = self.obstacle2[0] + self.obstacle2_xd[0] * (1 / self.hz)
        self.obstacle2[1] = self.obstacle2[1] + self.obstacle2_xd[1] * (1 / self.hz)
        self.obstacle2[2] = self.obstacle2[2]

        self.obstacle3[0] = self.obstacle3[0] + self.obstacle3_xd[0] * (1 / self.hz)
        self.obstacle3[1] = self.obstacle3[1] + self.obstacle3_xd[1] * (1 / self.hz)
        self.obstacle3[2] = self.obstacle3[2]

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = obstacle_node()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector des5roys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()