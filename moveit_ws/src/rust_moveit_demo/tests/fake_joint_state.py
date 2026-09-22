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
            0.50,
            1.15,
            0.67,
            0.00,
            0.00,

            # head
            0.0,
            0.28,

            # left arm
            # 1.153001070022583,
            # -1.3344438076019287,
            # -0.2471630573272705,
            # -2.4798474311828613,
            # -0.4565030634403229,
            # 0.7353981633974483,
            # -0.44030117988586426,
            # 1.513796066225225, -1.112202625179496, -0.5163930455156694, -2.3345147239823874, -0.5374117851562822, 0.3003982008745887, -0.2930459298010089,
            1.715130257582618, -0.9881855262167369, -0.6666312764545133, -2.253414810208766, -0.5825612252313707, 0.05765546159932907, -0.2108731807024783,
            
            # left gripper
            1,

            # right arm
            -1.82,
            1.02,
            0.70,
            2.10,
            0.54,
            0.29,
            0.11,

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