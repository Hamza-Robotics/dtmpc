import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
import numpy as np

class ArrayPublisher(Node):
    def __init__(self):
        super().__init__('dtmpc_waypoint')
        self.publisher_ = self.create_publisher(Float64MultiArray, 'dtmpc/waypoint/robot1', 10)

    def send_waypoints(self):
        # Example nx3 array
        nx3_array = np.array([[1.0, 1.0, 0.0],
                              [4.0, 5.0, 0.0],
                              [7.0, 8.0, 0.0]])

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
    array_publisher.send_waypoints()
    rclpy.spin_once(array_publisher)  # Spin once to publish the message
    rclpy.shutdown()

if __name__ == '__main__':
    main()
