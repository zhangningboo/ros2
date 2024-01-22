import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from interface_define_pkg.action import CustomAction


class CustomActionClient(Node):
    
    def __init__(self, node_name):
        super().__init__(node_name)
        self._action_client = ActionClient(node=self, action_type=CustomAction, action_name='custom_action')
        
    def send_goal(self, enable):
        self.get_logger().info('Sending goal request...')
        goal_msg = CustomAction.Goal()
        goal_msg.enable = enable
        self.get_logger().info('Waiting for server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Sending...')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.get_response_callback)

    def feedback_callback(self, msg: CustomAction):
        feedback = msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.state = }')
    
    def get_response_callback(self, future: rclpy.task.Future):
        """请求被接受后且收到响应后执行，并不是等动作执行完毕后执行本函数"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
        
    def get_result_callback(self, future: rclpy.task.Future):
        """动作执行完毕后执行本函数"""
        result = future.result().result
        self.get_logger().info(f'Result: {result.finish = }')


def main(args=None):
    rclpy.init(args=args)
    node = CustomActionClient('custom_action_client')
    node.send_goal(True)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()