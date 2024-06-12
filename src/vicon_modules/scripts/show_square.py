#!/usr/bin/env python3

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')
        self.publisher_ = self.create_publisher(Path, 'dtmpc/viconroom', 10)
        self.timer = self.create_timer(1.0, self.publish_path)
        self.get_logger().info('Path Publisher node has been started.')

    def publish_path(self):
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"

        # Points in meters  
        points = [
            (-2, 0.0),  # Point 1
            (-0.183, 1.380),
            (1.443, 0.072),   # Point 2
            (-0.085, -2.269), # Point 3
            (-1.837, 0.087), 
        ]

        for x, y in points:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.publisher_.publish(path_msg)
        self.get_logger().info('Publishing path with %d points' % len(points))

def main(args=None):
    rclpy.init(args=args)
    node = PathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
