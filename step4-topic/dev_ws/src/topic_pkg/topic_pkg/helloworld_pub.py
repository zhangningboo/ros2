import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class HelloWorldPublisher(Node):
    
    def __init__(self, node_name: str):
        super().__init__(node_name)
        self.pub = self.create_publisher(msg_type=String, topic='hello_world', qos_profile=10)  # 类型、话题名、队列大小
        self.timer = self.create_timer(1, self.timer_callback)  # 定时器
        
    def timer_callback(self):
        msg = String()
        msg.data = 'Hello, World!'
        self.pub.publish(msg)
        self.get_logger().info(f"Publishing: {msg.data = }")
        

def main(args=None):
    rclpy.init(args=args)
    hello_world_publisher = HelloWorldPublisher('hello_world_publisher')
    rclpy.spin(hello_world_publisher)
    hello_world_publisher.destroy_node()
    rclpy.shutdown()