#!/usr/bin/env python3

import rclpy
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
        HOST = '192.168.0.255'  # IP address of the broadcaster
        
        PORT = 12345  # The same port number used by the broadcaster
        self.publishers_=[self.publisher_1,self.publisher_2,self.publisher_3]
        self.subscriber_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.subscriber_socket.bind((HOST, PORT))

        timer_period = 0.005  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        data, addr = self.subscriber_socket.recvfrom(1024)  # Buffer size is 1024 bytes
        #print(f"Received message from {addr}: {data.decode()}")
        parsed=self.parse_xml_data(data.decode())
        k=0
        for i in range(len(parsed)):
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.pose.position.x = float(parsed[i].get('x'))/1000
            msg.pose.position.y = float(parsed[i].get('y'))/1000
            msg.pose.position.z = 0.0
            msg.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            for publisher_ in self.publishers_:
                    if publisher_.id == parsed[i].get('id'):
                        publisher_.publisher.publish(msg)
                        break
            print(parsed[0].get('id'), parsed[0].get('x'), parsed[0].get('time') - time.time())
            print("Timer callback   ")

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