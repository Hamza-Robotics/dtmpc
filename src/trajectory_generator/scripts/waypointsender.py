#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, Bool
import numpy as np

class ArrayPublisher(Node):
    def __init__(self):
        super().__init__('dtmpc_waypoint')
        self.publisher_ = self.create_publisher(Float64MultiArray, 'dtmpc/waypoint/robot1', 10)
        self.subscription = self.create_subscription(
            Bool,
            'send_waypoints',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        if msg.data:
            self.send_waypoints()

    def send_waypoints(self):
        # Example nx3 array
        nx3_array = np.array([[1.0, 0.0, 0.0],
                              [10.0, 0.0, 0.0],
                              [20.0, 0.0, 0.0]])

        # Create Float64MultiArray message
        msg = Float64MultiArray()
        msg.layout.dim.append(MultiArrayDimension(label="rows", size=nx3_array.shape[0], stride=nx3_array.shape[0] * nx3_array.shape[1]))
        msg.layout.dim.append(MultiArrayDimension(label="columns", size=nx3_array.shape[1], stride=nx3_array.shape[1]))
        msg.data = nx3_array.flatten().tolist()

        # Publish the message
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    array_publisher = ArrayPublisher()
    rclpy.spin(array_publisher)  # Spin to keep the program alive
    rclpy.shutdown()

if __name__ == '__main__':
    main()