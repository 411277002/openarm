import sys
import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory


class LeftArmMotion(Node):
    def __init__(self, motion_name):
        super().__init__("left_arm_motion")

        self.motion_name = motion_name
        self.sent = False

        self.left_arm_joints = [
            "openarm_left_joint1",
            "openarm_left_joint2",
            "openarm_left_joint3",
            "openarm_left_joint4",
            "openarm_left_joint5",
            "openarm_left_joint6",
            "openarm_left_joint7",
        ]

        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/left_joint_trajectory_controller/follow_joint_trajectory"
        )

        self.subscription = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10
        )

        # 安全設定
        self.max_delta = 0.03          # 一般相對動作，每個關節最多只允許動 0.03 rad
        self.default_steps = 5         # waypoint 數量
        self.default_total_time = 10.0 # 一般動作用 10 秒完成
        self.home_total_time = 12.0    # 回 home 慢一點

    def joint_state_callback(self, msg):
        if self.sent:
            return

        current_positions = []

        for joint_name in self.left_arm_joints:
            if joint_name not in msg.name:
                self.get_logger().error(f"Cannot find joint: {joint_name}")
                return

            index = msg.name.index(joint_name)
            current_positions.append(msg.position[index])

        self.get_logger().info("Current left arm positions:")
        self.get_logger().info(str(current_positions))

        target_positions, total_time = self.make_target(current_positions)

        if target_positions is None:
            rclpy.shutdown()
            return

        self.get_logger().info("Target left arm positions:")
        self.get_logger().info(str(target_positions))

        self.send_goal(current_positions, target_positions, total_time)
        self.sent = True

    def make_target(self, current_positions):
        target_positions = current_positions.copy()

        # motion 1：回到 ROS home
        if self.motion_name == "home":
            target_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            return target_positions, self.home_total_time

        # motion 2：joint4 +0.02、joint7 +0.01
        elif self.motion_name == "j4_j7_plus":
            deltas = {
                3: 0.02,  # joint4
                6: 0.01,  # joint7
            }
            return self.apply_relative_motion(current_positions, deltas), self.default_total_time

        # motion 3：joint4 -0.02、joint7 -0.01
        elif self.motion_name == "j4_j7_minus":
            deltas = {
                3: -0.02,  # joint4
                6: -0.01,  # joint7
            }
            return self.apply_relative_motion(current_positions, deltas), self.default_total_time

        # motion 4：joint4 小幅往正方向
        elif self.motion_name == "joint4_plus":
            deltas = {
                3: 0.02,
            }
            return self.apply_relative_motion(current_positions, deltas), self.default_total_time

        # motion 5：joint4 小幅往負方向
        elif self.motion_name == "joint4_minus":
            deltas = {
                3: -0.02,
            }
            return self.apply_relative_motion(current_positions, deltas), self.default_total_time

        # motion 6：joint7 小幅往正方向
        elif self.motion_name == "joint7_plus":
            deltas = {
                6: 0.01,
            }
            return self.apply_relative_motion(current_positions, deltas), self.default_total_time

        # motion 7：joint7 小幅往負方向
        elif self.motion_name == "joint7_minus":
            deltas = {
                6: -0.01,
            }
            return self.apply_relative_motion(current_positions, deltas), self.default_total_time
        
        # motion 8：
        elif self.motion_name == "small_wave":
            deltas = {
                3: 0.02,   # joint4
                5: 0.01,   # joint6
                6: 0.01,   # joint7
            }
            return self.apply_relative_motion(current_positions, deltas), self.default_total_time

        else:
            self.get_logger().error(f"Unknown motion name: {self.motion_name}")
            self.print_usage()
            return None, None

    def apply_relative_motion(self, current_positions, deltas):
        target_positions = current_positions.copy()

        for joint_index, delta in deltas.items():
            if abs(delta) > self.max_delta:
                self.get_logger().error(
                    f"Delta too large on joint index {joint_index}: {delta}. "
                    f"Max allowed is {self.max_delta}."
                )
                return current_positions

            target_positions[joint_index] = current_positions[joint_index] + delta

        return target_positions

    def send_goal(self, current_positions, target_positions, total_time):
        self.get_logger().info("Waiting for action server...")

        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available.")
            rclpy.shutdown()
            return

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.left_arm_joints

        # 多 waypoint，讓動作比較順
        steps = self.default_steps

        for step in range(1, steps + 1):
            ratio = step / steps

            point = JointTrajectoryPoint()
            point.positions = [
                current_positions[i] + (target_positions[i] - current_positions[i]) * ratio
                for i in range(len(current_positions))
            ]

            t = total_time * ratio
            point.time_from_start.sec = int(t)
            point.time_from_start.nanosec = int((t - int(t)) * 1e9)

            goal_msg.trajectory.points.append(point)

        self.get_logger().info(f"Sending motion: {self.motion_name}")
        self.get_logger().info(f"Waypoints: {steps}, total_time: {total_time} sec")

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

    def print_usage(self):
        self.get_logger().info("Usage:")
        self.get_logger().info("  python3 openarm_scripts/left_arm_motion.py home")
        self.get_logger().info("  python3 openarm_scripts/left_arm_motion.py j4_j7_plus")
        self.get_logger().info("  python3 openarm_scripts/left_arm_motion.py j4_j7_minus")
        self.get_logger().info("  python3 openarm_scripts/left_arm_motion.py joint4_plus")
        self.get_logger().info("  python3 openarm_scripts/left_arm_motion.py joint4_minus")
        self.get_logger().info("  python3 openarm_scripts/left_arm_motion.py joint7_plus")
        self.get_logger().info("  python3 openarm_scripts/left_arm_motion.py joint7_minus")


def main():
    if len(sys.argv) < 2:
        print("Usage:")
        print("  python3 openarm_scripts/left_arm_motion.py home")
        print("  python3 openarm_scripts/left_arm_motion.py j4_j7_plus")
        print("  python3 openarm_scripts/left_arm_motion.py j4_j7_minus")
        print("  python3 openarm_scripts/left_arm_motion.py joint4_plus")
        print("  python3 openarm_scripts/left_arm_motion.py joint4_minus")
        print("  python3 openarm_scripts/left_arm_motion.py joint7_plus")
        print("  python3 openarm_scripts/left_arm_motion.py joint7_minus")
        return

    motion_name = sys.argv[1]

    rclpy.init()
    node = LeftArmMotion(motion_name)
    rclpy.spin(node)
    node.destroy_node()


if __name__ == "__main__":
    main()
