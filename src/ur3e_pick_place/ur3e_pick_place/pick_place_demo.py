#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


UR_ACTION = "/joint_trajectory_controller/follow_joint_trajectory"
GRIPPER_ACTION = "/gripper_position_controller/follow_joint_trajectory"

UR_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

GRIPPER_JOINTS = ["finger_joint"]


class PickPlaceDemo(Node):
    def __init__(self):
        super().__init__("pick_place_demo")

        self.ur_client = ActionClient(self, FollowJointTrajectory, UR_ACTION)
        self.gripper_client = ActionClient(self, FollowJointTrajectory, GRIPPER_ACTION)

    def wait_servers(self):
        self.get_logger().info("Waiting for UR action server...")
        self.ur_client.wait_for_server()
        self.get_logger().info("Waiting for gripper action server...")
        self.gripper_client.wait_for_server()

    def send_traj(self, client, joint_names, positions, duration_sec: float):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joint_names

        pt = JointTrajectoryPoint()
        pt.positions = positions
        pt.time_from_start.sec = int(duration_sec)
        pt.time_from_start.nanosec = int((duration_sec - int(duration_sec)) * 1e9)

        goal.trajectory.points = [pt]

        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            raise RuntimeError("Trajectory goal rejected")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().result

    def move_ur(self, q, t=2.0):
        self.get_logger().info(f"UR move: {q}")
        self.send_traj(self.ur_client, UR_JOINTS, q, t)

    def set_gripper(self, finger_pos, t=1.0):
        # finger_joint range depends on your model; in your files you used ~0.695 closed.
        self.get_logger().info(f"Gripper finger_joint -> {finger_pos}")
        self.send_traj(self.gripper_client, GRIPPER_JOINTS, [finger_pos], t)


def main():
    rclpy.init()
    node = PickPlaceDemo()
    node.wait_servers()

    # -----------------------------
    # EDIT THESE VALUES (rad)
    # -----------------------------
    HOME = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]

    PRE_PICK  = [0.20, -1.30, 1.60, -1.90, -1.57, 0.20]
    PICK      = [0.20, -1.45, 1.75, -2.00, -1.57, 0.20]
    LIFT      = [0.20, -1.25, 1.60, -1.90, -1.57, 0.20]

    PRE_PLACE = [-0.90, -1.25, 1.60, -1.90, -1.57, 0.20]
    PLACE     = [-0.90, -1.45, 1.75, -2.00, -1.57, 0.20]
    RETREAT   = [-0.90, -1.25, 1.60, -1.90, -1.57, 0.20]

    GRIP_OPEN = 0.0
    GRIP_CLOSED = 0.695  # matches your xacro default closed_position usage

    try:
        node.move_ur(HOME, 3.0)
        node.set_gripper(GRIP_OPEN, 1.0)

        node.move_ur(PRE_PICK, 2.5)
        node.move_ur(PICK, 1.5)
        node.set_gripper(GRIP_CLOSED, 1.0)
        node.move_ur(LIFT, 1.5)

        node.move_ur(PRE_PLACE, 2.5)
        node.move_ur(PLACE, 1.5)
        node.set_gripper(GRIP_OPEN, 1.0)
        node.move_ur(RETREAT, 1.5)

        node.move_ur(HOME, 3.0)

    except Exception as e:
        node.get_logger().error(f"Failed: {e}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
