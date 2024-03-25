#!/usr/bin/env python3
import rclpy
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from conversion_functions import path2numpy, quaternion_to_euler, numpy2path
import yaml

import numpy as np
class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('Solution_Plotter')
        self.subscription = self.create_subscription(Float64MultiArray,'dtmpc/robot1/u_solution',self.publish_solution,10)
        self.subscription_state = self.create_subscription(PoseStamped,'robot1/pose',self.state_callback,10)
        self.x=np.array([0,0,0]).reshape(1,3)

        self.solution_pub= self.create_publisher(Path, "dtmpc/robot1/solution", 10)
        with open('src/mpc/config/mpc_params.yaml') as file:
            yamlfile = yaml.safe_load(file)
        frequency=yamlfile['Prediction_Frequency']
        self.samplingtime=1/frequency
    def markergen(self,x,y,radius,color,id,timestamp):
        marker = Marker()
        marker.type = Marker.CYLINDER
        marker.header.frame_id = 'map'
        marker.header.stamp = timestamp
        marker.id=id
        marker.ns="my_namespace"+str(id)
        marker.pose.position.x = x
        marker.pose.position.y = y
        if id<2:
            marker.pose.position.z=-0.3
        else:
            marker.pose.position.z=-0.1
        marker.scale.x = radius*2
        marker.scale.y = radius*2
        marker.scale.z = 0.1
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a=1.0
        
        return marker
    

    def path_point_gen(self,x):
        pathpoint=PoseStamped()
        pathpoint.header.stamp=self.get_clock().now().to_msg()
        pathpoint.header.frame_id='map'
        pathpoint.pose.position.x=x[0]
        pathpoint.pose.position.y=x[1]
        pathpoint.pose.position.z=0.0
        pathpoint.pose.orientation.x=0.0
        pathpoint.pose.orientation.y=0.0
        pathpoint.pose.orientation.z=0.0
        pathpoint.pose.orientation.w=1.0

        return pathpoint
    
    def prograpation(self,x,y):
        pathpoint=PoseStamped()
        pathpoint.header.stamp=self.get_clock().now().to_msg()
        pathpoint.header.frame_id='map'
        pathpoint.pose.position.x=x
        pathpoint.pose.position.y=y

        pathpoint.pose.position.z=0.0
        pathpoint.pose.orientation.x=0.0
        pathpoint.pose.orientation.y=0.0
        pathpoint.pose.orientation.z=0.0
        pathpoint.pose.orientation.w=1.0
        return pathpoint


    def state_callback(self,msg):
        rpy=quaternion_to_euler(msg.pose.orientation)
        self.x=np.array([msg.pose.position.x,msg.pose.position.y,rpy[2]]).reshape(1,3)

    def publish_solution(self, msg):
        path_solution=Path()
        
        # Set the timestamp for the paths
        timestamp = self.get_clock().now().to_msg()

        path_data = np.asarray(msg.data).reshape((len(msg.data) // 2,2))
        path_solution.header.stamp = timestamp
        path_solution.header.frame_id = 'map'

        x=self.x[0,0]
        y=self.x[0,1]
        th0=self.x[0,2]
        v=1
        thd=0
        for i in range(len(path_data)):
            v=path_data[i,0]
            thd=path_data[i,1]
            x=x+self.samplingtime*v*np.cos(th0)
            y=y+self.samplingtime*v*np.sin(th0)
            th0=th0+self.samplingtime*thd
            print(x,y)
            path_solution.poses.append(self.prograpation(x,y))



        self.solution_pub.publish(path_solution)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()