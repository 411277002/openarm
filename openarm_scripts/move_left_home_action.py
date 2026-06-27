#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState

from rclpy.action import ActionClient


class MoveLeftHomeAction(Node):
    def __init__(self):
        super().__init__("move_left_home_action")

        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/left_joint_trajectory_controller/follow_joint_trajectory"
        )

        self.left_arm_joints = [
            "openarm_left_joint1",
            "openarm_left_joint2",
            "openarm_left_joint3",
            "openarm_left_joint4",
            "openarm_left_joint5",
            "openarm_left_joint6",
            "openarm_left_joint7",
        ]

        self.current_positions = None

        self.joint_state_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10
        )

        self.timer = self.create_timer(1.0, self.start_once)
        self.started = False

    def joint_state_callback(self, msg):
        positions = []

        for joint_name in self.left_arm_joints:
            if joint_name not in msg.name:
                self.get_logger().warn(f"Joint not found: {joint_name}")
                return

            index = msg.name.index(joint_name)
            positions.append(msg.position[index])

        self.current_positions = positions

    def start_once(self):
        if self.started:
            return

        if self.current_positions is None:
            self.get_logger().info("Waiting for /joint_states...")
            return

        self.started = True
        self.timer.cancel()

        self.get_logger().info("Current left arm positions:")
        self.get_logger().info(str(self.current_positions))

        # ROS 原點：7 軸都回到 0
        target_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.get_logger().info("Target home positions:")
        self.get_logger().info(str(target_positions))

        self.send_goal(target_positions)

    def send_goal(self, target_positions):
        self.get_logger().info("Waiting for action server...")
        self.action_client.wait_for_server()

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.left_arm_joints

        point = JointTrajectoryPoint()
        point.positions = target_positions

        # 慢慢回原點，先用 10 秒，比較安全
        point.time_from_start.sec = 10

        goal_msg.trajectory.points.append(point)

        self.get_logger().info("Sending action goal: move left arm to ROS home [0,0,0,0,0,0,0]")

        send_goal_future = self.action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by controller.")
            rclpy.shutdown()
            return

        self.get_logger().info("Goal accepted by controller.")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result

        self.get_logger().info(f"Action result error_code: {result.error_code}")
        self.get_logger().info(f"Action result error_string: {result.error_string}")

        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    node = MoveLeftHomeAction()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()


if __name__ == "__main__":
    main()
