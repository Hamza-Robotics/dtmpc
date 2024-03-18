#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
from traj_gen import trajectory_genertor
import matplotlib.pyplot as plt
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class ArraySubscriber(Node):
    def __init__(self):
        super().__init__('trajectory_generator')
        self.subscription_waypoint = self.create_subscription(Float64MultiArray,'dtmpc/waypoint/robot1',self.waypoint_callback,10)
        self.subscription_state = self.create_subscription(PoseStamped,'robot1/pose',self.state_callback,10)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.waypoints = np.array([[0.001,0.001,0],[0.001,0.001,0],[0.003,0.003,0]]).reshape(-1, 3)
        self.publisher_feasbile = self.create_publisher(Path, 'dtmpc/robot1/trajectory_feasbile', 10)
        self.publisher_full = self.create_publisher(Path, 'dtmpc/robot1/trajectory_full', 10)
        self.x=np.array([[0,0,0]])
    def convert_to_path_stamped(self,traj):
        path=Path()
        for i in range(len(traj)):
            pose = PoseStamped()
            pose.pose.position.x = traj[i,0]
            pose.pose.position.y = traj[i,1]
            pose.pose.orientation.w = 1.0
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.header.frame_id="map"
            pose.header.stamp=self.get_clock().now().to_msg()
            path.poses.append(pose)

        path.header.frame_id="map"
        path.header.stamp=self.get_clock().now().to_msg()
        return path
    
    def timer_callback(self):
        if self.waypoints[0][0] != 190000091:
            gen=trajectory_genertor(max_v=1,prediction_Length=10,frequency=10,nodes=100)
            gen.waypoint=(self.waypoints)
            path,path_feasible,index=gen.trajectory(self.x)

            path_msg=self.convert_to_path_stamped(path)
            path_feasbile_msg=self.convert_to_path_stamped(path_feasible)
            self.publisher_full.publish(path_msg)
            self.publisher_feasbile.publish(path_feasbile_msg)
    def waypoint_callback(self, msg):
        data = msg.data
        self.waypoints = np.array(data).reshape(-1, 3)
        
    def state_callback(self, msg):
        self.x=np.array([[msg.pose.position.x,msg.pose.position.y,0]])

def main(args=None):
    rclpy.init(args=args)
    array_subscriber = ArraySubscriber()
    rclpy.spin(array_subscriber)
    array_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
