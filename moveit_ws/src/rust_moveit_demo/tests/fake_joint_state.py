#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class FakeJointStatePublisher(Node):

    def __init__(self):
        super().__init__("fake_g1_joint_state")

        self.publisher = self.create_publisher(
            JointState,
            "/joint_states",
            10,
        )

        # ============================================================
        # Galbot G1 init_pose
        #
        # leg:        5
        # head:       2
        # left arm:   7
        # left grip:  1
        # right arm:  7
        # right grip: 1
        #
        # total:     23
        # ============================================================

        self.joint_names = [

            # ------------------------
            # Leg
            # ------------------------
            "leg_joint1",
            "leg_joint2",
            "leg_joint3",
            "leg_joint4",
            "leg_joint5",

            # ------------------------
            # Head
            # ------------------------
            "head_joint1",
            "head_joint2",

            # ------------------------
            # Left arm
            # ------------------------
            "left_arm_joint1",
            "left_arm_joint2",
            "left_arm_joint3",
            "left_arm_joint4",
            "left_arm_joint5",
            "left_arm_joint6",
            "left_arm_joint7",

            # ------------------------
            # Left gripper
            # ------------------------
            "left_gripper_joint",

            # ------------------------
            # Right arm
            # ------------------------
            "right_arm_joint1",
            "right_arm_joint2",
            "right_arm_joint3",
            "right_arm_joint4",
            "right_arm_joint5",
            "right_arm_joint6",
            "right_arm_joint7",

            # ------------------------
            # Right gripper
            # ------------------------
            "right_gripper_joint",
        ]

        self.joint_positions = [

            # leg
            0.00,
            0.00,
            0.00,
            0.00,
            0.00,

            # head
            0.0,
            0.28,

            # left arm
            1.7016140222549438,
            -1.3842746019363403,
            -0.45012739300727844,
            -2.0721681118011475,
            -0.030344294384121895,
            -0.31139811873435974,
            -0.1437871903181076,

            # 0.3840063070692793, 0.9534193847259688, -0.528000074018484, -2.4794377740397784, 0.5019987550866102, 0.21518081840605127, 0.35656267591208113,
            # left gripper
            1,

            # right arm
            -1.7762774229049683,
            1.265079140663147,
            0.39507097005844116,
            2.1878888607025146,
            0.07423059642314911,
            0.27453330159187317,
            0.20706410706043243,

            # right gripper
            0.1,
        ]

        assert len(self.joint_names) == 23
        assert len(self.joint_positions) == 23

        # 100 Hz
        self.timer = self.create_timer(
            0.01,
            self.publish_joint_state,
        )

        self.get_logger().info(
            "Fake G1 joint state publisher started"
        )

    def publish_joint_state(self):
        msg = JointState()
        
        # 非常重要：
        # 使用当前 ROS 时间，而不是 0
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.name = self.joint_names
        msg.position = self.joint_positions

        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = FakeJointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()