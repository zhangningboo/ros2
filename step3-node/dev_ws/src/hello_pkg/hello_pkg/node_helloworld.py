import rclpy
from rclpy.node import Node
import time


def main(args=None):
    rclpy.init(args=args)
    node = Node('node_helloworld')
    while rclpy.ok():
        node.get_logger().info("Hello, world!")
        time.sleep(1)

    node.destroy_node()
    rclpy.shutdown()
