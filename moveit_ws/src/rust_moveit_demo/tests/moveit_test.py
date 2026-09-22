#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
    RobotState,
)
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState


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
        # 2. 使用机器人 init_pose 作为 Start State
        # ============================================================

        left_arm_joint_names = [
            "left_arm_joint1",
            "left_arm_joint2",
            "left_arm_joint3",
            "left_arm_joint4",
            "left_arm_joint5",
            "left_arm_joint6",
            "left_arm_joint7",
        ]

        left_arm_joint_positions = [
            1.76,
            -0.89,
            -0.68,
            -2.16,
            -0.52,
            -0.15,
            -0.13,
        ]

        joint_state = JointState()

        joint_state.name = left_arm_joint_names
        joint_state.position = left_arm_joint_positions

        # goal.request.start_state = RobotState()
        # goal.request.start_state.joint_state = joint_state

        # ============================================================
        # 3. 目标位置
        # ============================================================

        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = "base_footprint"
        position_constraint.link_name = "left_gripper_tcp_link"

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.01, 0.01, 0.01]

        position_constraint.constraint_region.primitives.append(primitive)

        pose = Pose()
        # pose.position.x = 0.329
        # pose.position.y = 0.339
        # pose.position.z = 0.850
        # pose.orientation.w = 1.0
        
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

        constraints.position_constraints = [
            position_constraint
        ]

        constraints.orientation_constraints = [
            orientation_constraint
        ]

        goal.request.goal_constraints = [
            constraints
        ]

        # ============================================================
        # 6. 只规划，不执行
        # ============================================================

        goal.planning_options.plan_only = True
        goal.planning_options.look_around = False
        goal.planning_options.replan = False

        return goal

    def run(self):
        self.get_logger().info(
            "waiting for MoveIt action server..."
        )

        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                "MoveIt action server not available"
            )
            return

        self.get_logger().info(
            "MoveIt action server available"
        )

        goal = self.build_goal()

        self.get_logger().info(
            "sending MoveGroup goal: "
            "left_arm -> left_arm_link7"
        )

        send_goal_future = self.client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback,
        )

        send_goal_future.add_done_callback(
            self.goal_response_callback
        )

    def feedback_callback(self, feedback_msg):

        self.get_logger().info(
            "MoveIt feedback received"
        )


    def result_callback(self, future):

        result = future.result()

        self.get_logger().info(
            f"MoveIt action status: {result.status}"
        )

        move_result = result.result

        self.get_logger().info(
            f"MoveIt error code: {move_result.error_code.val}"
        )

        self.get_logger().info(
            f"planning time: "
            f"{move_result.planning_time:.3f} sec"
        )

        trajectory = move_result.planned_trajectory

        self.get_logger().info(
            f"trajectory joint count: "
            f"{len(trajectory.joint_trajectory.joint_names)}"
        )

        self.get_logger().info(
            f"trajectory point count: "
            f"{len(trajectory.joint_trajectory.points)}"
        )

        if trajectory.joint_trajectory.joint_names:

            self.get_logger().info(
                "trajectory joints:"
            )

            for name in trajectory.joint_trajectory.joint_names:
                self.get_logger().info(
                    f"  {name}"
                )

        if trajectory.joint_trajectory.points:
            for point in trajectory.joint_trajectory.points:
                self.get_logger().info(
                    f"point positions: "
                    f"{list(point.positions)}"
                )

    def goal_response_callback(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error(
                "MoveIt rejected the goal"
            )
            rclpy.shutdown()
            return

        self.get_logger().info(
            "MoveIt accepted the goal"
        )

        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(
            self.result_callback
        )
    

def main():

    rclpy.init()

    node = MoveItDemo()
    try:
        node.run()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()