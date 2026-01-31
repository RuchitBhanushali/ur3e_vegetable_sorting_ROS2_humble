#!/usr/bin/env python3
# conveyor_pick_place.py  (ROBOT NODE ONLY)

import rclpy
import math
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand, FollowJointTrajectory

import tf2_ros
from geometry_msgs.msg import Pose
from std_msgs.msg import String

from ros_gz_interfaces.srv import SetEntityPose


class PegPickInsertDemoGz(Node):
    def __init__(self):
        super().__init__("conveyor_base")
        
        # Robot Speed
        self.arm_time_scale = float(
        self.declare_parameter("arm_time_scale", 0.6).value
        )

        self.world_name = self.declare_parameter("world_name", "conveyor_base").value
        self.peg_name = self.declare_parameter("peg_name", "peg").value
        self.tool_frame = self.declare_parameter("tool_frame", "tool0").value

        self.gripper_action = self.declare_parameter(
            "gripper_action", "/gripper_position_controller/gripper_cmd"
        ).value
        self.gripper_open = float(self.declare_parameter("gripper_open", 0.0).value)
        self.gripper_close = float(self.declare_parameter("gripper_close", 0.58).value)

        self.arm_action = self.declare_parameter(
            "arm_action", "/joint_trajectory_controller/follow_joint_trajectory"
        ).value

        self.tcp_offset_z = float(self.declare_parameter("tcp_offset_z", 0.244).value)

        self.get_logger().info(f"Using world: {self.world_name}")
        self.get_logger().info(f"Tool frame: {self.tool_frame}")

        self.gripper_client = ActionClient(self, GripperCommand, self.gripper_action)
        self.arm_client = ActionClient(self, FollowJointTrajectory, self.arm_action)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.set_pose_service = f"/world/{self.world_name}/set_pose"
        self.set_pose_cli = self.create_client(SetEntityPose, self.set_pose_service)
        self.set_pose_service = self._resolve_set_pose_service(self.set_pose_service)
        if self.set_pose_service != f"/world/{self.world_name}/set_pose":
            self.set_pose_cli = self.create_client(SetEntityPose, self.set_pose_service)

        # Cube trigger interface
        self.cube_ready_topic = self.declare_parameter("cube_ready_topic", "/cube_ready").value
        self.cube_done_topic = self.declare_parameter("cube_done_topic", "/cube_done").value
        self.cube_done_pub = self.create_publisher(String, self.cube_done_topic, 10)
        self.cube_sub = self.create_subscription(String, self.cube_ready_topic, self._on_cube_ready, 10)

        # State
        self.pending_cube = None
        self.sequence_running = False

        # NEW: move to P1 immediately on startup
        self.startup_done = False
        self.start_timer = self.create_timer(0.2, self._startup_move_to_p1)

        self.follow_peg = False
        self.follow_timer = self.create_timer(0.02, self.follow)

    def _list_world_set_pose_services(self):
        out = []
        for (name, types) in self.get_service_names_and_types():
            if name.startswith("/world/") and name.endswith("/set_pose"):
                out.append(name)
        return sorted(set(out))

    def _resolve_set_pose_service(self, preferred_name: str):
        for _ in range(600):
            if self.set_pose_cli.wait_for_service(timeout_sec=0.1):
                return preferred_name
            candidates = self._list_world_set_pose_services()
            if candidates:
                return candidates[0]
        raise RuntimeError(f"Service not available: {preferred_name}")

    def _wait_for_gripper_action(self):
        for _ in range(500):
            if self.gripper_client.wait_for_server(timeout_sec=0.1):
                return True
        return False

    def _wait_for_arm_action(self):
        for _ in range(500):
            if self.arm_client.wait_for_server(timeout_sec=0.1):
                return True
        return False

    def send_gripper(self, position: float, max_effort: float = 80.0):
        if not self._wait_for_gripper_action():
            self.get_logger().error(f"Gripper action server not available: {self.gripper_action}")
            return None
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(max_effort)
        return self.gripper_client.send_goal_async(goal)

    def _arm_joint_names(self):
        return [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

    # ------- Your 4 positions (keep angles, you will edit degrees) -------
    def _p1(self):
        p = JointTrajectoryPoint()
        p.positions = [math.radians(-15.00), math.radians(-17.0), math.radians(26.0),
                       math.radians(-10.0), math.radians(15.0), math.radians(0.00)]
        p.time_from_start.sec = max(1, int(round(3 * self.arm_time_scale)))
        return p

    def _p2(self):
        p = JointTrajectoryPoint()
        p.positions = [math.radians(-15.00), math.radians(-7.0), math.radians(25.0),
                       math.radians(-18.0), math.radians(-15.0), math.radians(0.00)]
        p.time_from_start.sec = max(1, int(round(3 * self.arm_time_scale)))
        return p

    def _p3(self):
        p = JointTrajectoryPoint()
        p.positions = [math.radians(-15.00), math.radians(-18.0), math.radians(12.0),
                       math.radians(6.0), math.radians(-15.0), math.radians(0.00)]
        p.time_from_start.sec = max(1, int(round(3 * self.arm_time_scale)))
        return p

    def _p4(self):
        p = JointTrajectoryPoint()
        p.positions = [math.radians(-86.00), math.radians(-6.0), math.radians(16.0),
                       math.radians(-31.0), math.radians(20.0), math.radians(0.00)]
        p.time_from_start.sec = max(1, int(round(3 * self.arm_time_scale)))
        return p

    def _send_arm_point(self, point: JointTrajectoryPoint):
        if not self._wait_for_arm_action():
            self.get_logger().error(f"Arm action server not available: {self.arm_action}")
            return None
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self._arm_joint_names()
        goal.trajectory.points = [point]
        return self.arm_client.send_goal_async(goal)

    # Startup: open gripper + go to P1 once
    def _startup_move_to_p1(self):
        if self.startup_done:
            return
        # stop timer after first successful kick
        self.start_timer.cancel()
        self.startup_done = True

        # ensure open at start
        self.send_gripper(self.gripper_open)

        fut = self._send_arm_point(self._p1())
        if fut is None:
            return
        fut.add_done_callback(self._on_start_p1_goal)

    def _on_start_p1_goal(self, future):
        try:
            gh = future.result()
            if not gh.accepted:
                self.get_logger().error("Startup P1 rejected")
                return
            gh.get_result_async().add_done_callback(lambda f: self.get_logger().info("Reached P1 (startup)"))
        except Exception as e:
            self.get_logger().error(f"Startup P1 failed: {e}")

    # Cube trigger
    def _on_cube_ready(self, msg: String):
        cube_name = msg.data.strip()
        if not cube_name:
            return
        if self.sequence_running:
            return

        self.pending_cube = cube_name
        self.sequence_running = True

        # Sequence: P2 -> close -> P3 -> P4 -> open -> publish done -> P1
        fut = self._send_arm_point(self._p2())
        if fut is None:
            self._reset_sequence()
            return
        fut.add_done_callback(self._on_p2_goal)

    def _on_p2_goal(self, future):
        try:
            gh = future.result()
            if not gh.accepted:
                self.get_logger().error("P2 rejected")
                self._reset_sequence()
                return
            gh.get_result_async().add_done_callback(self._on_p2_done)
        except Exception as e:
            self.get_logger().error(f"P2 send failed: {e}")
            self._reset_sequence()

    def _on_p2_done(self, future):
        fut = self.send_gripper(self.gripper_close)
        if fut is None:
            self._reset_sequence()
            return
        fut.add_done_callback(self._on_close_goal)

    def _on_close_goal(self, future):
        try:
            gh = future.result()
            if not gh.accepted:
                self.get_logger().error("Close rejected")
                self._reset_sequence()
                return
            gh.get_result_async().add_done_callback(self._on_close_done)
        except Exception as e:
            self.get_logger().error(f"Close send failed: {e}")
            self._reset_sequence()

    def _on_close_done(self, future):
        fut = self._send_arm_point(self._p3())
        if fut is None:
            self._reset_sequence()
            return
        fut.add_done_callback(self._on_p3_goal)

    def _on_p3_goal(self, future):
        try:
            gh = future.result()
            if not gh.accepted:
                self.get_logger().error("P3 rejected")
                self._reset_sequence()
                return
            gh.get_result_async().add_done_callback(self._on_p3_done)
        except Exception as e:
            self.get_logger().error(f"P3 send failed: {e}")
            self._reset_sequence()

    def _on_p3_done(self, future):
        fut = self._send_arm_point(self._p4())
        if fut is None:
            self._reset_sequence()
            return
        fut.add_done_callback(self._on_p4_goal)

    def _on_p4_goal(self, future):
        try:
            gh = future.result()
            if not gh.accepted:
                self.get_logger().error("P4 rejected")
                self._reset_sequence()
                return
            gh.get_result_async().add_done_callback(self._on_p4_done)
        except Exception as e:
            self.get_logger().error(f"P4 send failed: {e}")
            self._reset_sequence()

    def _on_p4_done(self, future):
        # open and immediately tell cube node to send cube on back belt
        self.send_gripper(self.gripper_open)

        if self.pending_cube:
            self.cube_done_pub.publish(String(data=self.pending_cube))

        # go back to P1 for next cube
        fut = self._send_arm_point(self._p1())
        if fut is None:
            self._reset_sequence()
            return
        fut.add_done_callback(self._on_back_p1_goal)

    def _on_back_p1_goal(self, future):
        try:
            gh = future.result()
            if not gh.accepted:
                self.get_logger().error("Back to P1 rejected")
                self._reset_sequence()
                return
            gh.get_result_async().add_done_callback(self._on_back_p1_done)
        except Exception as e:
            self.get_logger().error(f"Back to P1 failed: {e}")
            self._reset_sequence()

    def _on_back_p1_done(self, future):
        self._reset_sequence()

    def _reset_sequence(self):
        self.sequence_running = False
        self.pending_cube = None

    # (kept, unused but harmless)
    def follow(self):
        return


def main():
    rclpy.init()
    node = PegPickInsertDemoGz()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
