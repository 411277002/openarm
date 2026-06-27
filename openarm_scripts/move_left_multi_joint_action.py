import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory


class MoveLeftMultiJointAction(Node):
    def __init__(self):
        super().__init__("move_left_multi_joint_action")

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

        self.sent = False

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

        target_positions = current_positions.copy()

        # 小角度多關節控制
        # positions[3] = joint4
        # positions[6] = joint7
        target_positions[3] = current_positions[3] + 0.5
        target_positions[6] = current_positions[6] + 0.5

        self.get_logger().info("Current left arm positions:")
        self.get_logger().info(str(current_positions))

        self.get_logger().info("Target left arm positions:")
        self.get_logger().info(str(target_positions))

        self.send_goal(current_positions, target_positions)
        self.sent = True

    def send_goal(self, current_positions, target_positions):
        self.get_logger().info("Waiting for action server...")

        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available.")
            rclpy.shutdown()
            return

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.left_arm_joints

        # 分成多段 waypoint，讓動作比較順
        steps = 5
        total_time = 10.0

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

        self.get_logger().info(
            "Sending action goal: joint4 +0.02 rad, joint7 +0.01 rad with 5 waypoints"
        )

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
    node = MoveLeftMultiJointAction()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == "__main__":
    main()

