import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class HelloWorldSubscriber(Node):
    
    def __init__(self, node_name: str):
        super().__init__(node_name)
        self.pub = self.create_subscription(msg_type=String, topic='hello_world', callback=self.do_something_callback, qos_profile=10)  # 类型、话题名、队列大小
        
    def do_something_callback(self, msg):
        self.get_logger().info(f"Subscriber get data: {msg.data = }")
        

def main(args=None):
    rclpy.init(args=args)
    hello_world_subscriber = HelloWorldSubscriber('hello_world_subscriber')
    rclpy.spin(hello_world_subscriber)
    hello_world_subscriber.destroy_node()
    rclpy.shutdown()