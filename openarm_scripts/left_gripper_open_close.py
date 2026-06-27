import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory


class LeftGripperOpenClose(Node):
    def __init__(self, mode):
        super().__init__("left_gripper_open_close")

        self.mode = mode
        self.gripper_joint = "openarm_left_finger_joint1"

        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/left_gripper_controller/follow_joint_trajectory"
        )

        self.subscription = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10
        )

        self.sent = False

    def joint_state_callback(self, msg):
        if self.sent:
            return

        if self.gripper_joint not in msg.name:
            self.get_logger().error(f"Cannot find joint: {self.gripper_joint}")
            return

        index = msg.name.index(self.gripper_joint)
        current_position = msg.position[index]

        # 先用很小的幅度測試，避免夾太大或撞到極限
        step = 0.05

        if self.mode == "open":
            target_position = current_position + step
        elif self.mode == "close":
            target_position = current_position - step
        else:
            self.get_logger().error("Mode must be: open or close")
            rclpy.shutdown()
            return

        self.get_logger().info(f"Current gripper position: {current_position}")
        self.get_logger().info(f"Target gripper position: {target_position}")

        self.send_goal(target_position)
        self.sent = True

    def send_goal(self, target_position):
        self.get_logger().info("Waiting for gripper action server...")

        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Gripper action server not available.")
            rclpy.shutdown()
            return

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = [self.gripper_joint]

        # 分 3 段送，讓夾爪動作不要太突然
        current_position = None

        # 重新從最近一次 log 中不容易拿 current，所以這裡用單點也可以。
        # 夾爪動作幅度很小，先用 2 秒完成。
        point = JointTrajectoryPoint()
        point.positions = [target_position]
        point.time_from_start.sec = 2

        goal_msg.trajectory.points.append(point)

        self.get_logger().info(f"Sending gripper goal: {self.mode}")

        send_goal_future = self.action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by gripper controller.")
            rclpy.shutdown()
            return

        self.get_logger().info("Goal accepted by gripper controller.")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result

        self.get_logger().info(f"Action result error_code: {result.error_code}")
        self.get_logger().info(f"Action result error_string: {result.error_string}")

        rclpy.shutdown()


def main():
    if len(sys.argv) < 2:
        print("Usage:")
        print("  python3 left_gripper_open_close.py open")
        print("  python3 left_gripper_open_close.py close")
        return

    mode = sys.argv[1]

    rclpy.init()
    node = LeftGripperOpenClose(mode)
    rclpy.spin(node)
    node.destroy_node()


if __name__ == "__main__":
    main()
