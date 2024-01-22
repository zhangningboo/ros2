import rclpy
from rclpy.node import Node
from interface_define_pkg.msg import CustomMsg


class CustomMsgSub(Node):
    
    def __init__(self, node_name):
        super().__init__(node_name)
        self.subscription = self.create_subscription(msg_type=CustomMsg, topic='custom_msg', callback=self.process_callback, qos_profile=2)
        
    def process_callback(self, msg: CustomMsg):
        self.get_logger().info(f"{msg.name = }, {msg.id = }")


def main(args=None):
    rclpy.init(args=args)
    node = CustomMsgSub("custom_msg_subscription")
    rclpy.spin(node)
    rclpy.shutdown()