import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory


class OpenArmManualPanel(Node):
    def __init__(self):
        super().__init__("openarm_manual_panel")

        self.left_arm_joints = [
            "openarm_left_joint1",
            "openarm_left_joint2",
            "openarm_left_joint3",
            "openarm_left_joint4",
            "openarm_left_joint5",
            "openarm_left_joint6",
            "openarm_left_joint7",
        ]

        self.left_gripper_joint = "openarm_left_finger_joint1"

        self.arm_action_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/left_joint_trajectory_controller/follow_joint_trajectory"
        )

        self.gripper_action_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/left_gripper_controller/follow_joint_trajectory"
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10
        )

        self.current_arm_positions = None
        self.current_gripper_position = None

        # ===== 安全限制 =====
        self.max_arm_delta = 0.1        # 每個手臂關節單次最多 ±0.1 rad
        self.max_joint_count = 5        # 一次最多控制 5 個關節
        self.arm_steps = 20             # 手臂 waypoint 數量
        self.arm_total_time = 1.0       # 一般手臂動作秒數
        self.home_total_time = 5.0     # 回 home 秒數

        self.gripper_step = 0.05        # 夾爪每次開合幅度
        self.gripper_min = -0.10        # 夾爪保守下限
        self.gripper_max = 0.10         # 夾爪保守上限
        self.gripper_steps = 3
        self.gripper_total_time = 2.0

    def joint_state_callback(self, msg):
        arm_positions = []

        for joint_name in self.left_arm_joints:
            if joint_name not in msg.name:
                return

            index = msg.name.index(joint_name)
            arm_positions.append(msg.position[index])

        self.current_arm_positions = arm_positions

        if self.left_gripper_joint in msg.name:
            index = msg.name.index(self.left_gripper_joint)
            self.current_gripper_position = msg.position[index]

    def wait_for_joint_states(self):
        self.get_logger().info("Waiting for /joint_states...")

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.current_arm_positions is not None:
                self.get_logger().info("Joint states received.")
                return True

        return False

    def print_help(self):
        print("")
        print("========== OpenArm Manual Panel ==========")
        print("可用指令：")
        print("")
        print("  arm 4:0.02")
        print("      joint4 +0.02 rad")
        print("")
        print("  arm 4:0.02,7:0.01")
        print("      joint4 +0.02 rad，joint7 +0.01 rad")
        print("")
        print("  arm 4:-0.02,7:-0.01")
        print("      joint4 -0.02 rad，joint7 -0.01 rad")
        print("")
        print("  gripper open")
        print("      夾爪打開一點")
        print("")
        print("  gripper close")
        print("      夾爪關閉一點")
        print("")
        print("  home")
        print("      左手臂回到 ROS home [0,0,0,0,0,0,0]")
        print("")
        print("  status")
        print("      顯示目前手臂與夾爪位置")
        print("")
        print("  help")
        print("      顯示說明")
        print("")
        print("  quit")
        print("      離開程式")
        print("")
        print("安全限制：")
        print(f"  手臂每個關節單次最多 ±{self.max_arm_delta} rad")
        print(f"  手臂一次最多控制 {self.max_joint_count} 個關節")
        print(f"  夾爪每次 step = {self.gripper_step} rad")
        print("==========================================")
        print("")

    def print_status(self):
        rclpy.spin_once(self, timeout_sec=0.1)

        print("")
        print("目前左手臂位置：")
        if self.current_arm_positions is None:
            print("  尚未收到手臂 joint_states")
        else:
            for i, pos in enumerate(self.current_arm_positions):
                print(f"  joint{i + 1}: {pos:.6f}")

        print("")
        print("目前左夾爪位置：")
        if self.current_gripper_position is None:
            print("  尚未收到夾爪 joint_states")
        else:
            print(f"  {self.left_gripper_joint}: {self.current_gripper_position:.6f}")

        print("")

    def parse_arm_command(self, text):
        try:
            parts = text.split(",")

            if len(parts) > self.max_joint_count:
                print(f"錯誤：一次最多只能控制 {self.max_joint_count} 個關節。")
                return None

            deltas = {}

            for part in parts:
                part = part.strip()

                if ":" not in part:
                    print("錯誤：格式需為 joint:delta，例如 arm 4:0.02")
                    return None

                joint_text, delta_text = part.split(":", 1)

                joint_number = int(joint_text.strip())
                delta = float(delta_text.strip())

                if joint_number < 1 or joint_number > 7:
                    print("錯誤：關節編號只能是 1~7。")
                    return None

                if abs(delta) > self.max_arm_delta:
                    print(
                        f"錯誤：joint{joint_number} 的 delta={delta} 太大，"
                        f"單次最多只能 ±{self.max_arm_delta} rad。"
                    )
                    return None

                joint_index = joint_number - 1

                if joint_index in deltas:
                    print(f"錯誤：joint{joint_number} 重複輸入。")
                    return None

                deltas[joint_index] = delta

            return deltas

        except ValueError:
            print("錯誤：請確認關節編號是整數，角度是數字。")
            return None

    def confirm(self):
        answer = input("確認執行？輸入 y 才會動：").strip().lower()
        return answer == "y"

    def handle_arm_motion(self, delta_text):
        if self.current_arm_positions is None:
            print("尚未收到手臂位置，請稍後再試。")
            return

        deltas = self.parse_arm_command(delta_text)

        if deltas is None:
            return

        current_positions = self.current_arm_positions.copy()
        target_positions = current_positions.copy()

        for joint_index, delta in deltas.items():
            target_positions[joint_index] = current_positions[joint_index] + delta

        print("")
        print("本次手臂動作：")
        for joint_index, delta in deltas.items():
            print(f"  joint{joint_index + 1}: {delta:+.4f} rad")

        print("")
        print("目前位置：")
        print([round(x, 6) for x in current_positions])

        print("目標位置：")
        print([round(x, 6) for x in target_positions])
        print("")

        if not self.confirm():
            print("取消手臂動作。")
            return

        self.send_arm_goal(current_positions, target_positions, self.arm_total_time)

    def handle_home(self):
        if self.current_arm_positions is None:
            print("尚未收到手臂位置，請稍後再試。")
            return

        current_positions = self.current_arm_positions.copy()
        target_positions = [0.0] * 7

        print("")
        print("準備回到 ROS home：")
        print("目前位置：")
        print([round(x, 6) for x in current_positions])
        print("目標位置：")
        print(target_positions)
        print("")

        if not self.confirm():
            print("取消 home 動作。")
            return

        self.send_arm_goal(current_positions, target_positions, self.home_total_time)

    def handle_gripper(self, mode):
        if self.current_gripper_position is None:
            print("尚未收到夾爪位置，請稍後再試。")
            return

        current_position = self.current_gripper_position

        if mode == "open":
            target_position = current_position + self.gripper_step
        elif mode == "close":
            target_position = current_position - self.gripper_step
        else:
            print("錯誤：gripper 指令只能是 open 或 close。")
            return

        # 夾爪保護範圍
        target_position = max(self.gripper_min, min(self.gripper_max, target_position))

        print("")
        print(f"本次夾爪動作：{mode}")
        print(f"目前位置：{current_position:.6f}")
        print(f"目標位置：{target_position:.6f}")
        print("")

        if not self.confirm():
            print("取消夾爪動作。")
            return

        self.send_gripper_goal(current_position, target_position)

    def send_arm_goal(self, current_positions, target_positions, total_time):
        self.get_logger().info("Waiting for left arm action server...")

        if not self.arm_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Left arm action server not available.")
            return

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.left_arm_joints

        for step in range(1, self.arm_steps + 1):
            raw_ratio = step / self.arm_steps
            ratio = raw_ratio * raw_ratio * (3.0 - 2.0 * raw_ratio)

            point = JointTrajectoryPoint()
            point.positions = [
                current_positions[i] + (target_positions[i] - current_positions[i]) * ratio
                for i in range(len(current_positions))
            ]

            t = total_time * ratio
            point.time_from_start.sec = int(t)
            point.time_from_start.nanosec = int((t - int(t)) * 1e9)

            goal_msg.trajectory.points.append(point)

        self.get_logger().info(
            f"Sending left arm goal, waypoints={self.arm_steps}, total_time={total_time} sec"
        )

        send_future = self.arm_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()

        if goal_handle is None:
            self.get_logger().error("Failed to send arm goal.")
            return

        if not goal_handle.accepted:
            self.get_logger().error("Arm goal rejected.")
            return

        self.get_logger().info("Arm goal accepted.")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        self.get_logger().info(f"Arm result error_code: {result.error_code}")
        self.get_logger().info(f"Arm result error_string: {result.error_string}")

    def send_gripper_goal(self, current_position, target_position):
        self.get_logger().info("Waiting for left gripper action server...")

        if not self.gripper_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Left gripper action server not available.")
            return

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = [self.left_gripper_joint]

        for step in range(1, self.gripper_steps + 1):
            ratio = step / self.gripper_steps

            point = JointTrajectoryPoint()
            point.positions = [
                current_position + (target_position - current_position) * ratio
            ]

            t = self.gripper_total_time * ratio
            point.time_from_start.sec = int(t)
            point.time_from_start.nanosec = int((t - int(t)) * 1e9)

            goal_msg.trajectory.points.append(point)

        self.get_logger().info(
            f"Sending gripper goal, waypoints={self.gripper_steps}, "
            f"total_time={self.gripper_total_time} sec"
        )

        send_future = self.gripper_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()

        if goal_handle is None:
            self.get_logger().error("Failed to send gripper goal.")
            return

        if not goal_handle.accepted:
            self.get_logger().error("Gripper goal rejected.")
            return

        self.get_logger().info("Gripper goal accepted.")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        self.get_logger().info(f"Gripper result error_code: {result.error_code}")
        self.get_logger().info(f"Gripper result error_string: {result.error_string}")

    def handle_demo_wave(self):
        print("")
        print("準備執行 demo_wave：")
        print("  1. joint4 +0.02, joint7 +0.01")
        print("  2. joint4 -0.02, joint7 -0.01")
        print("  3. joint4 +0.02, joint7 +0.01")
        print("  4. joint4 -0.02, joint7 -0.01")
        print("")

        if not self.confirm():
            print("取消 demo_wave。")
            return

        sequence = [
            {3: 0.02, 6: 0.01},
            {3: -0.02, 6: -0.01},
            {3: 0.02, 6: 0.01},
            {3: -0.02, 6: -0.01},
        ]

        for index, deltas in enumerate(sequence):
            print(f"執行 demo_wave step {index + 1}/{len(sequence)}")

            rclpy.spin_once(self, timeout_sec=0.2)

            current_positions = self.current_arm_positions.copy()
            target_positions = current_positions.copy()

            for joint_index, delta in deltas.items():
                target_positions[joint_index] = current_positions[joint_index] + delta

            self.send_arm_goal(current_positions, target_positions, self.arm_total_time)

    def handle_demo_grab(self):

        if not self.confirm():
            print("取消 demo_grab。")
            return

        # 1. home
        rclpy.spin_once(self, timeout_sec=0.2)
        current_positions = self.current_arm_positions.copy()
        target_positions = [0.0] * 7
        self.send_arm_goal(current_positions, target_positions, self.home_total_time)

        # 2. gripper open
        rclpy.spin_once(self, timeout_sec=0.2)
        current_gripper = self.current_gripper_position
        target_gripper = current_gripper + self.gripper_step
        target_gripper = max(self.gripper_min, min(self.gripper_max, target_gripper))
        self.send_gripper_goal(current_gripper, target_gripper)

        # 3. arm forward / pose change
        rclpy.spin_once(self, timeout_sec=0.2)
        current_positions = self.current_arm_positions.copy()
        target_positions = current_positions.copy()
        target_positions[3] = current_positions[3] + 0.02
        target_positions[6] = current_positions[6] + 0.01
        self.send_arm_goal(current_positions, target_positions, self.arm_total_time)

        # 4. gripper close
        rclpy.spin_once(self, timeout_sec=0.2)
        current_gripper = self.current_gripper_position
        target_gripper = current_gripper - self.gripper_step
        target_gripper = max(self.gripper_min, min(self.gripper_max, target_gripper))
        self.send_gripper_goal(current_gripper, target_gripper)

        # 5. arm back
        rclpy.spin_once(self, timeout_sec=0.2)
        current_positions = self.current_arm_positions.copy()
        target_positions = current_positions.copy()
        target_positions[3] = current_positions[3] - 0.02
        target_positions[6] = current_positions[6] - 0.01
        self.send_arm_goal(current_positions, target_positions, self.arm_total_time)

        # 6. home
        rclpy.spin_once(self, timeout_sec=0.2)
        current_positions = self.current_arm_positions.copy()
        target_positions = [0.0] * 7
        self.send_arm_goal(current_positions, target_positions, self.home_total_time)

        print("")
        print("demo_grab 執行完成。")
    
    def run_panel(self):
        if not self.wait_for_joint_states():
            return

        self.print_help()

        while rclpy.ok():
            # 更新一下目前位置
            rclpy.spin_once(self, timeout_sec=0.1)

            command = input("openarm> ").strip()

            if command == "":
                continue

            if command == "quit" or command == "exit":
                print("離開 OpenArm manual panel。")
                break

            elif command == "help":
                self.print_help()

            elif command == "status":
                self.print_status()

            elif command == "home":
                self.handle_home()

            elif command.startswith("arm "):
                delta_text = command[4:].strip()

                if delta_text == "":
                    print("錯誤：arm 後面需要輸入關節，例如 arm 4:0.02")
                    continue

                self.handle_arm_motion(delta_text)

            elif command.startswith("gripper "):
                mode = command[len("gripper "):].strip().lower()
                self.handle_gripper(mode)

            elif command.startswith("demo_wave"):
                self.handle_demo_wave()

            elif command.startswith("demo_grab"):
                self.handle_demo_grab()

            else:
                print("未知指令。輸入 help 查看可用指令。")


def main():
    rclpy.init()

    node = OpenArmManualPanel()

    try:
        node.run_panel()
    except KeyboardInterrupt:
        print("")
        print("收到 Ctrl+C，結束程式。")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()