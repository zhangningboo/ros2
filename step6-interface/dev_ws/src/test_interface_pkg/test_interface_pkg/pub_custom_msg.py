import rclpy
from rclpy.node import Node
from interface_define_pkg.msg import CustomMsg


class CustomMsgPub(Node):
    
    def __init__(self, node_name):
        super().__init__(node_name)
        self.publisher = self.create_publisher(msg_type=CustomMsg, topic='custom_msg', qos_profile=10)
        self.timer = self.create_timer(timer_period_sec=1.0, callback=self.timer_callback)
        self.id = 0
        self.name = node_name
        
    def timer_callback(self):
        msg = CustomMsg()
        msg.header.stamp = self.get_clock().now().seconds_nanoseconds()[0]
        msg.header.frame_id = "custom_msg_publisher"
        msg.data = "Hello ROS2"
        msg.id = self.id
        msg.name = self.name
        self.id += 1
        self.publisher.publish(msg)
        self.get_logger().info("Publishing custom message")


def main(args=None):
    rclpy.init(args=args)
    node = CustomMsgPub("custom_msg_publisher")
    rclpy.spin(node)
    rclpy.shutdown()