#!/usr/bin/env python3
from math import sin, cos, pi
import rclpy
import rclpy.logging
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, Twist, PoseStamped
from std_msgs.msg import String
import socket
import xml.etree.ElementTree as ET
import time


class Publisher_():
    def __init__(self,publisher,id):
        self.publisher=publisher    
        self.id=id
        
    

  
class MinimalPublisher(Node):

    def parse_xml_data(self,xml_string):
        # Parse the XML string
        root = ET.fromstring(xml_string)
        
        # Initialize a list to store the objects' data
        objects = []
        
        # Iterate over each 'object' element in the XML
        for obj in root.findall('object'):
            # Extract the attributes of the object
            obj_id = obj.get('id')
            x = float(obj.get('x'))
            y = float(obj.get('y'))
            yaw = float(obj.get('yaw'))
            time= float(obj.get('time'))
            # Create a dictionary for the object's data
            object_data = {
                'id': obj_id,
                'x': x,
                'y': y,
                'yaw': yaw,
                'time':time 
            }
            
            # Append the object data to the list
            objects.append(object_data)
        
        return objects

    def __init__(self):
        super().__init__('publish_vicon_state')
        self.publisher_1 = Publisher_(self.create_publisher(PoseStamped, 'robot1/pose', 10),"robot1")
        self.publisher_2 = Publisher_(self.create_publisher(PoseStamped, 'robot2/pose', 10),"robot2")
        self.publisher_3 = Publisher_(self.create_publisher(PoseStamped, 'robot3/pose', 10),"robot3")
        HOST = '192.168.1.68'  # IP address of the broadcaster
        PORT = 1234  # The same port number used by the broadcaster
        self.publishers_=[self.publisher_1,self.publisher_2,self.publisher_3]
        self.subscriber_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.subscriber_socket.bind((HOST, PORT))
        self.previous_data = None
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0


    def timer_callback(self):
            data, addr = self.subscriber_socket.recvfrom(1024)  # Buffer size is 1024 bytes
            
            # Check if data is different from the previous data
            if data != self.previous_data:
                self.previous_data = data  # Update previous data
                try:
                    parsed = self.parse_xml_data(data.decode())
                    #print(parsed)
                    for i in range(len(parsed)):
                        msg = PoseStamped()
                        msg.header.stamp = self.get_clock().now().to_msg()
                        msg.header.frame_id = 'map'

                        msg.pose.position.x = float(parsed[i].get('x')) / 1000
                        msg.pose.position.y = float(parsed[i].get('y')) / 1000
                        yaw = float(parsed[i].get('yaw'))
                        msg.pose.position.z = yaw
                        msg.pose.orientation = euler_to_quaternion(0, 0, yaw)
                        
                        for publisher_ in self.publishers_:
                            if publisher_.id == parsed[i].get('id'):
                                publisher_.publisher.publish(msg)
                                break
                except Exception as e:
                    # Log error
                    print(f"Error processing Vicon feedback: {e}")
            else:
                print("No new data received")

            

def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

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