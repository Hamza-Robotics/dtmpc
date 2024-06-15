#!/usr/bin/env python3
#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import Path
from std_msgs.msg import String

#from NMPC  import NMPC
from visualization_msgs.msg import MarkerArray, Marker
import sympy as sp
import time
from conversion_functions import path2numpy, quaternion_to_euler, numpy2path, path2numpy
from geometry_msgs.msg import PoseStamped, Twist, PointStamped
from std_msgs.msg import Float64MultiArray
import time
import pickle
import socket
import json

class Vizualize_Solution(Node):
    def __init__(self):
        super().__init__('vizualize_solution')
        self.publisher_path_1 = self.create_publisher(Path, 'dtmpc/solution/broadcast/'+'robot1'+'/viz', 10)
        self.publisher_path_2 = self.create_publisher(Path, 'dtmpc/solution/broadcast/'+'robot2'+'/viz', 10)
        self.publisher_path_3 = self.create_publisher(Path, 'dtmpc/solution/broadcast/'+'robot3'+'/viz', 10)

        self.timer_robot1= self.create_timer(0.1, self.timer_callback) 
        self.timer_robot2= self.create_timer(0.1, self.timer_callback2)
        self.timer_robot3= self.create_timer(0.1, self.timer_callback3)
        self.timer_pub= self.create_timer(0.1, self.publish_data)
        self.HOST =  "192.168.1.67" 
        self.PORT1 = int(str(1)*5)
        self.PORT2 = int(str(2)*5)
        self.PORT3 = int(str(3)*5)


        self.solution_robot1=np.zeros((3,1)) 
        self.solution_robot2=np.zeros((3,1))
        self.solution_robot3=np.zeros((3,1))

        self.receiver1 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.receiver1.bind((self.HOST, self.PORT1))
        #self.receiver2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        #self.receiver2.bind((self.HOST, self.PORT2))
        #self.receiver3 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        #self.receiver3.bind((self.HOST, self.PORT3))


        self.buffersize = 65536

    def timer_callback(self):
        try:
            data1, _ = self.receiver1.recvfrom(self.buffersize)  # Increased buffer size
            solution_json1 = data1.decode('utf-8')
            solution_list1 = json.loads(solution_json1)
            self.solution_robot1 = np.array(solution_list1)
            print("Received solution:", np.shape(self.solution_robot1))
        except json.JSONDecodeError as e:
            print(f"JSON decode error: {e}")
        except Exception as e:
            print(f"An error occurred: {e}")
    def timer_callback2(self):
        try:
            data2, _ = self.receiver2.recvfrom(self.buffersize)  # Increased buffer size
            solution_json2 = data2.decode('utf-8')
            solution_list2 = json.loads(solution_json2)
            self.solution_robot2 = np.array(solution_list2)
            print("Received solution:", np.shape(self.solution_robot2))
        except json.JSONDecodeError as e:
            print(f"JSON decode error: {e}")
        except Exception as e:
            print(f"An error occurred: {e}")
    def timer_callback3(self):
        try:
            data3, _ = self.receiver3.recvfrom(self.buffersize)  # Increased buffer size
            solution_json3 = data3.decode('utf-8')
            solution_list3 = json.loads(solution_json3)
            self.solution_robot3 = np.array(solution_list3)
            print("Received solution:", np.shape(self.solution_robot3))
        except json.JSONDecodeError as e:
            print(f"JSON decode error: {e}")
        except Exception as e:
            print(f"An error occurred: {e}")


    def publish_data(self):

        path1 = Path()
        path1.header.frame_id = 'map'
        path1.header.stamp = self.get_clock().now().to_msg()
        path1 = numpy2path(self,self.solution_robot1)

        self.publisher_path_1.publish(path1)

def main(args=None):
    rclpy.init (args=args)

    viz=Vizualize_Solution()

    rclpy.spin(viz)

    viz.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



