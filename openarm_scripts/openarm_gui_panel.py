import threading
import tkinter as tk
from tkinter import messagebox

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory


class OpenArmGuiNode(Node):
    def __init__(self):
        super().__init__("openarm_gui_panel")

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

        # ===== 安全設定 =====
        self.max_arm_delta = 0.5        # 每個手臂關節單次最多 ±0.5 rad
        self.max_joint_count = 5        # 一次最多控制 5 個關節
        self.arm_steps = 20             # 手臂 waypoint 數量
        self.arm_total_time = 1.0       # 一般手臂動作秒數
        self.home_total_time = 3.0     # 回 home 秒數
        self.emergency_home_time = 20.0  # emergency home 更慢

        self.gripper_step = 0.1
        self.gripper_min = -0.10
        self.gripper_max = 0.10
        self.gripper_steps = 5
        self.gripper_total_time = 3.0

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

    def smooth_ratio(self, raw_ratio):
        # smoothstep：起步慢、中間順、結尾慢
        return raw_ratio * raw_ratio * (3.0 - 2.0 * raw_ratio)

    def send_arm_goal(self, target_positions, total_time=None):
        if self.current_arm_positions is None:
            self.get_logger().error("No current arm positions yet.")
            return False

        if total_time is None:
            total_time = self.arm_total_time

        current_positions = self.current_arm_positions.copy()

        if not self.arm_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Left arm action server not available.")
            return False

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.left_arm_joints

        for step in range(1, self.arm_steps + 1):
            raw_ratio = step / self.arm_steps
            ratio = self.smooth_ratio(raw_ratio)

            point = JointTrajectoryPoint()
            point.positions = [
                current_positions[i] + (target_positions[i] - current_positions[i]) * ratio
                for i in range(len(current_positions))
            ]

            t = total_time * raw_ratio
            point.time_from_start.sec = int(t)
            point.time_from_start.nanosec = int((t - int(t)) * 1e9)

            goal_msg.trajectory.points.append(point)

        self.get_logger().info(f"Sending arm goal: {target_positions}")

        send_future = self.arm_action_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self.arm_goal_response_callback)

        return True

    def arm_goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Arm goal rejected.")
            return

        self.get_logger().info("Arm goal accepted.")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.arm_result_callback)

    def arm_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Arm result error_code: {result.error_code}")
        self.get_logger().info(f"Arm result error_string: {result.error_string}")

    def send_relative_arm_motion(self, deltas):
        if self.current_arm_positions is None:
            return False, "尚未收到手臂位置。"

        target_positions = self.current_arm_positions.copy()

        for joint_index, delta in deltas.items():
            if abs(delta) > self.max_arm_delta:
                return False, (
                    f"joint{joint_index + 1} 的 delta={delta} 太大。\n"
                    f"每個關節單次最多只能 ±{self.max_arm_delta} rad。"
                )

            target_positions[joint_index] += delta

        ok = self.send_arm_goal(target_positions, self.arm_total_time)

        if ok:
            return True, "已送出手臂動作。"

        return False, "手臂 action server 不可用。"

    def send_home(self, emergency=False):
        if self.current_arm_positions is None:
            return False, "尚未收到手臂位置。"

        target_positions = [0.0] * 7

        if emergency:
            ok = self.send_arm_goal(target_positions, self.emergency_home_time)
        else:
            ok = self.send_arm_goal(target_positions, self.home_total_time)

        if ok:
            return True, "已送出 Home 動作。"

        return False, "手臂 action server 不可用。"

    def send_gripper_goal(self, target_position):
        if self.current_gripper_position is None:
            self.get_logger().error("No current gripper position yet.")
            return False

        current_position = self.current_gripper_position

        if not self.gripper_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Left gripper action server not available.")
            return False

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = [self.left_gripper_joint]

        for step in range(1, self.gripper_steps + 1):
            raw_ratio = step / self.gripper_steps
            ratio = self.smooth_ratio(raw_ratio)

            point = JointTrajectoryPoint()
            point.positions = [
                current_position + (target_position - current_position) * ratio
            ]

            t = self.gripper_total_time * raw_ratio
            point.time_from_start.sec = int(t)
            point.time_from_start.nanosec = int((t - int(t)) * 1e9)

            goal_msg.trajectory.points.append(point)

        self.get_logger().info(f"Sending gripper goal: {target_position}")

        send_future = self.gripper_action_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self.gripper_goal_response_callback)

        return True

    def gripper_goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Gripper goal rejected.")
            return

        self.get_logger().info("Gripper goal accepted.")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.gripper_result_callback)

    def gripper_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Gripper result error_code: {result.error_code}")
        self.get_logger().info(f"Gripper result error_string: {result.error_string}")

    def send_gripper(self, mode):
        if self.current_gripper_position is None:
            return False, "尚未收到夾爪位置。"

        current_position = self.current_gripper_position

        if mode == "open":
            target_position = current_position + self.gripper_step
        elif mode == "close":
            target_position = current_position - self.gripper_step
        else:
            return False, "gripper mode 只能是 open 或 close。"

        target_position = max(self.gripper_min, min(self.gripper_max, target_position))

        ok = self.send_gripper_goal(target_position)

        if ok:
            return True, f"已送出 gripper {mode}。"

        return False, "夾爪 action server 不可用。"


class OpenArmGui:
    def __init__(self, root, node):
        self.root = root
        self.node = node

        self.root.title("OpenArm 左手臂控制介面")
        self.root.geometry("560x620")

        self.delta_entries = []

        title = tk.Label(
            root,
            text="OpenArm 左手臂 GUI 控制",
            font=("Arial", 18, "bold")
        )
        title.pack(pady=10)

        warning = tk.Label(
            root,
            text="注意：輸入的是相對角度 delta，單位 rad。空白代表該關節不動。",
            fg="red"
        )
        warning.pack(pady=5)

        limit_label = tk.Label(
            root,
            text=f"安全限制：每個關節單次最多 ±{self.node.max_arm_delta} rad，但可同時控制 1～7 個關節。"
        )
        limit_label.pack(pady=5)

        joint_frame = tk.Frame(root)
        joint_frame.pack(pady=10)

        for i in range(7):
            row = tk.Frame(joint_frame)
            row.pack(pady=4)

            label = tk.Label(row, text=f"joint{i + 1} delta:", width=14, anchor="e")
            label.pack(side=tk.LEFT)

            entry = tk.Entry(row, width=12)
            entry.pack(side=tk.LEFT, padx=6)

            self.delta_entries.append(entry)

        button_frame = tk.Frame(root)
        button_frame.pack(pady=10)

        send_button = tk.Button(
            button_frame,
            text="送出手臂動作",
            width=18,
            command=self.on_send_arm
        )
        send_button.grid(row=0, column=0, padx=6, pady=6)

        clear_button = tk.Button(
            button_frame,
            text="清空輸入",
            width=18,
            command=self.clear_entries
        )
        clear_button.grid(row=0, column=1, padx=6, pady=6)

        home_button = tk.Button(
            button_frame,
            text="Home",
            width=18,
            command=self.on_home
        )
        home_button.grid(row=1, column=0, padx=6, pady=6)

        emergency_home_button = tk.Button(
            button_frame,
            text="Emergency Home",
            width=18,
            command=self.on_emergency_home
        )
        emergency_home_button.grid(row=1, column=1, padx=6, pady=6)

        gripper_open_button = tk.Button(
            button_frame,
            text="Gripper Open",
            width=18,
            command=lambda: self.on_gripper("open")
        )
        gripper_open_button.grid(row=2, column=0, padx=6, pady=6)

        gripper_close_button = tk.Button(
            button_frame,
            text="Gripper Close",
            width=18,
            command=lambda: self.on_gripper("close")
        )
        gripper_close_button.grid(row=2, column=1, padx=6, pady=6)

        status_button = tk.Button(
            button_frame,
            text="更新目前狀態",
            width=18,
            command=self.update_status
        )
        status_button.grid(row=3, column=0, padx=6, pady=6)

        quit_button = tk.Button(
            button_frame,
            text="離開",
            width=18,
            command=self.on_quit
        )
        quit_button.grid(row=3, column=1, padx=6, pady=6)

        self.status_text = tk.Text(root, height=13, width=65)
        self.status_text.pack(pady=10)

        self.update_status_loop()

    def clear_entries(self):
        for entry in self.delta_entries:
            entry.delete(0, tk.END)

    def get_deltas_from_entries(self):
        deltas = {}

        for i, entry in enumerate(self.delta_entries):
            text = entry.get().strip()

            if text == "":
                continue

            try:
                delta = float(text)
            except ValueError:
                return None, f"joint{i + 1} 輸入不是數字：{text}"

            if abs(delta) > self.node.max_arm_delta:
                return None, (
                    f"joint{i + 1} 的 delta={delta} 太大。\n"
                    f"每個關節單次最多只能 ±{self.node.max_arm_delta} rad。"
                )

            deltas[i] = delta

        if not deltas:
            return None, "沒有輸入任何關節 delta。"

        return deltas, None

    def on_send_arm(self):
        deltas, error = self.get_deltas_from_entries()

        if error:
            messagebox.showerror("輸入錯誤", error)
            return

        msg = "確認送出以下動作？\n\n"

        for joint_index, delta in deltas.items():
            msg += f"joint{joint_index + 1}: {delta:+.4f} rad\n"

        msg += "\n輸入後手臂會實際動作。"

        if not messagebox.askyesno("確認動作", msg):
            return

        ok, result_msg = self.node.send_relative_arm_motion(deltas)

        if ok:
            messagebox.showinfo("已送出", result_msg)
        else:
            messagebox.showerror("錯誤", result_msg)

    def on_home(self):
        if not messagebox.askyesno("確認 Home", "確認讓左手臂慢慢回到 ROS home？"):
            return

        ok, result_msg = self.node.send_home(emergency=False)

        if ok:
            messagebox.showinfo("已送出", result_msg)
        else:
            messagebox.showerror("錯誤", result_msg)

    def on_emergency_home(self):
        if not messagebox.askyesno(
            "確認 Emergency Home",
            "確認執行 Emergency Home？\n手臂會用更慢速度回到 ROS home。"
        ):
            return

        ok, result_msg = self.node.send_home(emergency=True)

        if ok:
            messagebox.showinfo("已送出", result_msg)
        else:
            messagebox.showerror("錯誤", result_msg)

    def on_gripper(self, mode):
        if not messagebox.askyesno("確認夾爪動作", f"確認執行 gripper {mode}？"):
            return

        ok, result_msg = self.node.send_gripper(mode)

        if ok:
            messagebox.showinfo("已送出", result_msg)
        else:
            messagebox.showerror("錯誤", result_msg)

    def update_status(self):
        self.status_text.delete("1.0", tk.END)

        self.status_text.insert(tk.END, "目前左手臂位置：\n")

        if self.node.current_arm_positions is None:
            self.status_text.insert(tk.END, "  尚未收到 /joint_states\n")
        else:
            for i, pos in enumerate(self.node.current_arm_positions):
                self.status_text.insert(tk.END, f"  joint{i + 1}: {pos:.6f}\n")

        self.status_text.insert(tk.END, "\n目前左夾爪位置：\n")

        if self.node.current_gripper_position is None:
            self.status_text.insert(tk.END, "  尚未收到夾爪位置\n")
        else:
            self.status_text.insert(
                tk.END,
                f"  {self.node.left_gripper_joint}: {self.node.current_gripper_position:.6f}\n"
            )

        self.status_text.insert(tk.END, "\n操作提醒：\n")
        self.status_text.insert(tk.END, "  1. 空白代表不動\n")
        self.status_text.insert(tk.END, "  2. 建議先用 0.01 或 0.02\n")
        self.status_text.insert(tk.END, "  3. 不要一次輸入過大角度\n")
        self.status_text.insert(tk.END, "  4. 動 joint4 後仍建議用 candump 確認狀態是 14\n")

    def update_status_loop(self):
        self.update_status()
        self.root.after(1000, self.update_status_loop)

    def on_quit(self):
        self.root.quit()
        self.root.destroy()


def main():
    rclpy.init()

    node = OpenArmGuiNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    root = tk.Tk()
    app = OpenArmGui(root, node)

    try:
        root.mainloop()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()