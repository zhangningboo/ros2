#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    PositionConstraint,
    OrientationConstraint,
)
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


class MoveItDemo(Node):

    def __init__(self):
        super().__init__("python_moveit_demo")

        self.client = ActionClient(
            self,
            MoveGroup,
            "/move_action",
        )

    def build_goal(self):

        goal = MoveGroup.Goal()

        # ============================================================
        # 1. MoveIt group
        # ============================================================

        goal.request.group_name = "left_arm"

        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0

        goal.request.max_velocity_scaling_factor = 0.2
        goal.request.max_acceleration_scaling_factor = 0.2

        # ============================================================
        # 2. Start State
        #
        # 不手动设置 start_state。
        #
        # MoveIt 会使用当前 Planning Scene / current_state
        # 作为起始状态。
        # ============================================================

        # ============================================================
        # 3. 目标位置
        # ============================================================

        position_constraint = PositionConstraint()

        position_constraint.header.frame_id = "base_footprint"
        position_constraint.link_name = "left_gripper_tcp_link"

        primitive = SolidPrimitive()

        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [
            0.01,
            0.01,
            0.01,
        ]

        position_constraint.constraint_region.primitives.append(primitive)

        pose = Pose()

        pose.position.x = 0.501
        pose.position.y = 0.210
        pose.position.z = 0.93

        pose.orientation.w = 1.0

        position_constraint.constraint_region.primitive_poses.append(pose)

        position_constraint.weight = 1.0

        # ============================================================
        # 4. 姿态约束
        # ============================================================

        orientation_constraint = OrientationConstraint()

        orientation_constraint.header.frame_id = "base_footprint"

        orientation_constraint.link_name = "left_gripper_tcp_link"

        orientation_constraint.orientation.x = -0.996708289612147
        orientation_constraint.orientation.y = 0.01437037
        orientation_constraint.orientation.z = -0.0759
        orientation_constraint.orientation.w = 0.024503

        orientation_constraint.absolute_x_axis_tolerance = 0.1
        orientation_constraint.absolute_y_axis_tolerance = 0.1
        orientation_constraint.absolute_z_axis_tolerance = 0.1

        orientation_constraint.weight = 1.0

        # ============================================================
        # 5. Goal Constraints
        # ============================================================

        constraints = Constraints()

        constraints.position_constraints = [position_constraint]

        constraints.orientation_constraints = [orientation_constraint]

        goal.request.goal_constraints = [constraints]

        # ============================================================
        # 6. Planning Options
        #
        # 关键：
        #
        # plan_only = False
        #
        # => MoveIt 规划成功后直接执行。
        # ============================================================

        goal.planning_options.plan_only = False

        goal.planning_options.look_around = False

        goal.planning_options.replan = False

        return goal

    def run(self):

        self.get_logger().info("waiting for MoveIt action server...")

        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("MoveIt action server not available")
            return

        self.get_logger().info("MoveIt action server available")

        goal = self.build_goal()

        self.get_logger().info("sending MoveGroup goal:")

        self.get_logger().info("  group      : left_arm")

        self.get_logger().info("  target     : left_gripper_tcp_link")

        self.get_logger().info("  plan_only  : false")

        self.get_logger().info("  mode       : plan + execute")

        send_goal_future = self.client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback,
        )

        send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):

        feedback = feedback_msg.feedback

        self.get_logger().debug(f"MoveIt feedback: {feedback}")

    def goal_response_callback(self, future):

        goal_handle = future.result()

        if goal_handle is None:
            self.get_logger().error("MoveIt returned an invalid goal handle")

            rclpy.shutdown()
            return

        if not goal_handle.accepted:

            self.get_logger().error("MoveIt rejected the goal")

            rclpy.shutdown()
            return

        self.get_logger().info("MoveIt accepted the goal")

        self.get_logger().info("planning + execution started...")

        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):

        try:
            result = future.result()

        except Exception as e:

            self.get_logger().error(f"failed to get MoveIt result: {e}")

            rclpy.shutdown()
            return

        self.get_logger().info(f"MoveIt action status: {result.status}")

        move_result = result.result

        self.get_logger().info(f"MoveIt error code: " f"{move_result.error_code.val}")

        self.get_logger().info(
            f"planning time: " f"{move_result.planning_time:.3f} sec"
        )

        # ============================================================
        # Error code
        # ============================================================

        if move_result.error_code.val == 1:

            self.get_logger().info("MoveIt planning/execution succeeded.")

        else:

            self.get_logger().error("MoveIt planning/execution failed.")

        # ============================================================
        # Planned trajectory
        #
        # 注意：
        #
        # plan_only=False 时，MoveIt 会执行这条轨迹。
        #
        # 这里打印出来只是为了调试。
        # ============================================================

        trajectory = move_result.planned_trajectory

        joint_trajectory = trajectory.joint_trajectory

        self.get_logger().info(
            f"trajectory joint count: " f"{len(joint_trajectory.joint_names)}"
        )

        self.get_logger().info(
            f"trajectory point count: " f"{len(joint_trajectory.points)}"
        )

        if joint_trajectory.joint_names:

            self.get_logger().info("trajectory joints:")

            for name in joint_trajectory.joint_names:

                self.get_logger().info(f"  {name}")

        # ============================================================
        # 打印轨迹点
        # ============================================================

        for i, point in enumerate(joint_trajectory.points):

            self.get_logger().debug(
                f"trajectory point {i}: " f"positions={list(point.positions)}"
            )

        # ============================================================
        # 结束
        # ============================================================

        if move_result.error_code.val == 1:

            self.get_logger().info("Robot motion completed.")

        else:

            self.get_logger().error("Robot motion did not complete successfully.")

        rclpy.shutdown()


def main():
    rclpy.init()
    node = MoveItDemo()
    try:
        node.run()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user.")

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
