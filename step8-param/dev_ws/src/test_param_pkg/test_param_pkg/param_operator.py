import rclpy
from rclpy.node import Node


class ParameterExample(Node):

    def __init__(self, node_name: str):
        super().__init__(node_name)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.declare_parameter('my_parameter', 'default_value')
        self.cnt = 0

    def timer_callback(self):
        # 获取参数
        my_parameter = self.get_parameter('my_parameter').get_parameter_value().string_value
        self.get_logger().info(f'My parameter is: {my_parameter}')
        my_parameter = rclpy.Parameter(name='my_parameter', type_=rclpy.Parameter.Type.STRING, value=f'new_value:{self.cnt}')
        # 设置参数
        self.set_parameters([my_parameter])
        self.cnt += 1


def main(args=None):
    rclpy.init(args=args)
    parameter_example = ParameterExample('parameter_example')
    rclpy.spin(parameter_example)
    parameter_example.destroy_node()
    rclpy.shutdown()