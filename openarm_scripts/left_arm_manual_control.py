import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory


class LeftArmManualControl(Node):
    def __init__(self):
        super().__init__("left_arm_manual_control")

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

        self.current_positions = None
        self.sent = False

        # 安全限制
        self.max_delta = 0.05
        self.max_joint_count = 3
        self.steps = 5
        self.total_time = 10.0
        self.home_total_time = 12.0

    def joint_state_callback(self, msg):
        if self.current_positions is not None:
            return

        positions = []

        for joint_name in self.left_arm_joints:
            if joint_name not in msg.name:
                self.get_logger().error(f"Cannot find joint: {joint_name}")
                return

            index = msg.name.index(joint_name)
            positions.append(msg.position[index])

        self.current_positions = positions

        self.get_logger().info("Current left arm positions:")
        self.get_logger().info(str(self.current_positions))

        self.ask_user_input()

    def ask_user_input(self):
        print("")
        print("請輸入要控制的關節與角度：")
        print("格式範例：")
        print("  4:0.02")
        print("  4:0.02,7:0.01")
        print("  4:-0.02,7:-0.01")
        print("  home")
        print("")
        print("注意：")
        print("  關節編號是 1~7")
        print("  每個關節單次最多只能動 ±0.05 rad")
        print("  一次最多控制 3 個關節")
        print("")

        user_input = input("motion> ").strip()

        if user_input == "":
            print("沒有輸入，結束程式。")
            rclpy.shutdown()
            return

        if user_input.lower() == "home":
            target_positions = [0.0] * 7
            total_time = self.home_total_time

            print("")
            print("準備回到 ROS home：")
            print(target_positions)
            confirm = input("確認執行？輸入 y 才會動：").strip().lower()

            if confirm != "y":
                print("取消動作。")
                rclpy.shutdown()
                return

            self.send_goal(self.current_positions, target_positions, total_time)
            return

        deltas = self.parse_joint_deltas(user_input)

        if deltas is None:
            rclpy.shutdown()
            return

        target_positions = self.current_positions.copy()

        for joint_index, delta in deltas.items():
            target_positions[joint_index] = self.current_positions[joint_index] + delta

        print("")
        print("目前位置：")
        print(self.current_positions)
        print("")
        print("目標位置：")
        print(target_positions)
        print("")
        print("本次動作：")
        for joint_index, delta in deltas.items():
            print(f"  joint{joint_index + 1}: {delta:+.4f} rad")

        confirm = input("確認執行？輸入 y 才會動：").strip().lower()

        if confirm != "y":
            print("取消動作。")
            rclpy.shutdown()
            return

        self.send_goal(self.current_positions, target_positions, self.total_time)

    def parse_joint_deltas(self, text):
        try:
            parts = text.split(",")

            if len(parts) > self.max_joint_count:
                print(f"錯誤：一次最多只能控制 {self.max_joint_count} 個關節。")
                return None

            deltas = {}

            for part in parts:
                part = part.strip()

                if ":" not in part:
                    print("錯誤：格式需為 joint:delta，例如 4:0.02")
                    return None

                joint_text, delta_text = part.split(":", 1)

                joint_number = int(joint_text.strip())
                delta = float(delta_text.strip())

                if joint_number < 1 or joint_number > 7:
                    print("錯誤：關節編號只能是 1~7。")
                    return None

                if abs(delta) > self.max_delta:
                    print(
                        f"錯誤：joint{joint_number} 的 delta={delta} 太大。"
                        f"單次最多只能 ±{self.max_delta} rad。"
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

    def send_goal(self, current_positions, target_positions, total_time):
        self.get_logger().info("Waiting for action server...")

        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available.")
            rclpy.shutdown()
            return

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.left_arm_joints

        for step in range(1, self.steps + 1):
            ratio = step / self.steps

            point = JointTrajectoryPoint()
            point.positions = [
                current_positions[i] + (target_positions[i] - current_positions[i]) * ratio
                for i in range(len(current_positions))
            ]

            t = total_time * ratio
            point.time_from_start.sec = int(t)
            point.time_from_start.nanosec = int((t - int(t)) * 1e9)

            goal_msg.trajectory.points.append(point)

        self.get_logger().info("Sending manual motion goal...")
        self.get_logger().info(f"Waypoints: {self.steps}, total_time: {total_time} sec")

        self.sent = True

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


def main():
    rclpy.init()
    node = LeftArmManualControl()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()


if __name__ == "__main__":
    main()
