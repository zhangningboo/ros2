import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from interface_define_pkg.action import CustomAction


class CustomActionServer(Node):
    
    def __init__(self, node_name):
        super().__init__(node_name)
        self._action_server = ActionServer(
            node=self,
            action_type=CustomAction,
            action_name='custom_action',
            callback_group=ReentrantCallbackGroup,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)
    
    def goal_callback(self, goal_request):
        self.get_logger().info(f'Executing function goal_callback, {type(goal_request) = } {dir(goal_request) = }')
        return GoalResponse.ACCEPT
    
    def execute_callback(self, goal_handle: rclpy.action.server.ServerGoalHandle):
        self.get_logger().info(f'Executing function execute_callback')
        
        feedback_msg = CustomAction.Feedback()
        # 执行自定义逻辑
        result = CustomAction.Result()
        for i in range(0, 360, 30):
            feedback_msg.state = i
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info('Custom action server publish feedback')
            # 检查是否取消
            time.sleep(1)

        # 执行其他逻辑
        # ...

        # 完成动作
        self.get_logger().info('Custom action server completed')
        goal_handle.succeed()
        result.finish = True
        return result
    
    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info(f'Received cancel request, {type(goal_handle) = }')
        return CancelResponse.ACCEPT
    
def main(args=None):
    rclpy.init(args=args)
    node = CustomActionServer('custom_action_server')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()