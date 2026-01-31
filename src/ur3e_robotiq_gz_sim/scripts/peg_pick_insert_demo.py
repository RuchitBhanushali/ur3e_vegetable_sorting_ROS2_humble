#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from rclpy.action import ActionClient
from control_msgs.action import GripperCommand, FollowJointTrajectory

import tf2_ros
from geometry_msgs.msg import Pose

# Service provided by ros_gz_sim: /world/<world>/set_pose
from ros_gz_interfaces.srv import SetEntityPose


class PegPickInsertDemoGz(Node):
    def __init__(self):
        super().__init__("peg_pick_insert_demo")

        self.world_name = self.declare_parameter("world_name", "peg_in_hole_demo").value
        self.peg_name = self.declare_parameter("peg_name", "peg").value
        self.tool_frame = self.declare_parameter("tool_frame", "tool0").value
        self.traj_topic = self.declare_parameter(
            "traj_topic", "/joint_trajectory_controller/joint_trajectory"
        ).value

        self.gripper_action = self.declare_parameter(
            "gripper_action", "/gripper_position_controller/gripper_cmd"
        ).value
        self.gripper_open = float(self.declare_parameter("gripper_open", 0.0).value)
        self.gripper_close = float(self.declare_parameter("gripper_close", 0.56).value)

        # Arm action server (required to sequence correctly)
        self.arm_action = self.declare_parameter(
            "arm_action", "/joint_trajectory_controller/follow_joint_trajectory"
        ).value

        # TCP offset along tool Z (meters) so peg sits correctly inside gripper
        self.tcp_offset_z = float(self.declare_parameter("tcp_offset_z", 0.244).value)

        self.get_logger().info(f"Using world: {self.world_name}")
        self.get_logger().info(f"Peg name: {self.peg_name}")
        self.get_logger().info(f"Tool frame: {self.tool_frame}")
        self.get_logger().info(f"Trajectory topic: {self.traj_topic}")

        # Publisher: arm (kept, but sequencing will use action)
        self.traj_pub = self.create_publisher(JointTrajectory, self.traj_topic, 10)

        # Action: gripper
        self.gripper_client = ActionClient(self, GripperCommand, self.gripper_action)

        # Action: arm (required)
        self.arm_client = ActionClient(self, FollowJointTrajectory, self.arm_action)

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- Service: set_pose (robust) ---
        # Try exact world service first; if missing, auto-discover any /world/*/set_pose
        self.set_pose_service = f"/world/{self.world_name}/set_pose"
        self.set_pose_cli = self.create_client(SetEntityPose, self.set_pose_service)

        self.set_pose_service = self._resolve_set_pose_service(self.set_pose_service)
        # Re-create client if the resolved name differs
        if self.set_pose_service != f"/world/{self.world_name}/set_pose":
            self.set_pose_cli = self.create_client(SetEntityPose, self.set_pose_service)

        self.get_logger().info(f"Using set_pose service: {self.set_pose_service}")

        # State machine timeline (kept)
        self.t0 = self.get_clock().now()
        self.phase = 0

        # Peg follow state
        self.follow_peg = False

        # Timers
        self.timer = self.create_timer(0.05, self.tick)          # main state machine
        self.follow_timer = self.create_timer(0.02, self.follow) # peg follow when "attached"

    def _list_world_set_pose_services(self):
        """Return list of services that look like /world/<name>/set_pose."""
        out = []
        for (name, types) in self.get_service_names_and_types():
            if name.startswith("/world/") and name.endswith("/set_pose"):
                # Ensure it supports the right type if possible
                if any("ros_gz_interfaces/srv/SetEntityPose" in t for t in types):
                    out.append(name)
                else:
                    # still accept it (some systems report types differently)
                    out.append(name)
        return sorted(set(out))

    def _resolve_set_pose_service(self, preferred_name: str):
        """
        Wait up to ~60s for set_pose. If preferred is not available, pick the first
        /world/*/set_pose that appears and continue.
        """
        # 600 * 0.1 = 60 seconds
        for _ in range(600):
            if self.set_pose_cli.wait_for_service(timeout_sec=0.1):
                return preferred_name

            # Try discovery while waiting
            candidates = self._list_world_set_pose_services()
            if candidates:
                chosen = candidates[0]
                # If preferred appears later, fine; but we can proceed now.
                return chosen

        # Final diagnostic
        candidates = self._list_world_set_pose_services()
        self.get_logger().error(f"Service not available: {preferred_name}")
        if candidates:
            self.get_logger().error(f"Found these set_pose services instead: {candidates}")
        else:
            self.get_logger().error("No /world/*/set_pose service found.")
            self.get_logger().error("Run: ros2 service list | grep /world/")
        raise RuntimeError(f"Service not available: {preferred_name}")

    def _wait_for_gripper_action(self):
        for _ in range(500):  # wait longer
            if self.gripper_client.wait_for_server(timeout_sec=0.1):
                return True
        return False

    def _wait_for_arm_action(self):
        for _ in range(200):
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

        fut = self.gripper_client.send_goal_async(goal)
        self.get_logger().info(f"Gripper goal sent: pos={position:.3f}")
        return fut

    def _arm_joint_names(self):
        return [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

    def _points_p1_p2(self):
        p1 = JointTrajectoryPoint()
        p1.positions = [math.radians(29.00), math.radians(-79.0), math.radians(125.0),
                        math.radians(-46.0), math.radians(29.0), math.radians(0.00)]
        p1.time_from_start.sec = 3

        p2 = JointTrajectoryPoint()
        p2.positions = [math.radians(11.00), math.radians(-73.0), math.radians(146.0),
                        math.radians(-73.0), math.radians(11.0), math.radians(0.00)]
        p2.time_from_start.sec = 6
        return [p1, p2]

    def _points_p3_p4(self):
        p3 = JointTrajectoryPoint()
        p3.positions = [math.radians(11.00), math.radians(-109.0), math.radians(112.0),
                        math.radians(-3.0), math.radians(11.0), math.radians(0.00)]
        p3.time_from_start.sec = 3

        p4 = JointTrajectoryPoint()
        p4.positions = [math.radians(4.00), math.radians(-57.0), math.radians(89.0),
                        math.radians(-32.0), math.radians(4.0), math.radians(0.00)]
        p4.time_from_start.sec = 6
        return [p3, p4]

    def _send_arm_action(self, points):
        if not self._wait_for_arm_action():
            self.get_logger().error(f"Arm action server not available: {self.arm_action}")
            return None

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self._arm_joint_names()
        goal.trajectory.points = points
        return self.arm_client.send_goal_async(goal)

    def set_peg_pose_world(self, x, y, z, qx, qy, qz, qw):
        req = SetEntityPose.Request()
        req.entity.name = self.peg_name
        req.pose = Pose()
        req.pose.position.x = float(x)
        req.pose.position.y = float(y)
        req.pose.position.z = float(z)
        req.pose.orientation.x = float(qx)
        req.pose.orientation.y = float(qy)
        req.pose.orientation.z = float(qz)
        req.pose.orientation.w = float(qw)
        self.set_pose_cli.call_async(req)

    def follow(self):
        if not self.follow_peg:
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                "world", self.tool_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05)
            )

            x = tf.transform.translation.x
            y = tf.transform.translation.y
            z = tf.transform.translation.z

            qx = tf.transform.rotation.x
            qy = tf.transform.rotation.y
            qz = tf.transform.rotation.z
            qw = tf.transform.rotation.w

            self.set_peg_pose_world(x, y, z, qx, qy, qz, qw)

        except Exception:
            pass

    # --- Sequenced flow (required change) ---
    def tick(self):
        dt = (self.get_clock().now() - self.t0).nanoseconds / 1e9

        # Phase 0: open gripper, then start p1->p2 ONLY after open is accepted
        if self.phase == 0 and dt > 2.0:
            fut = self.send_gripper(self.gripper_open)
            if fut is not None:
                fut.add_done_callback(self._on_open_sent)
                self.phase = 99  # waiting
            return

    def _on_open_sent(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"Open gripper send failed: {e}")
            self.phase = 0
            return

        if not goal_handle.accepted:
            self.get_logger().error("Open gripper goal rejected")
            self.phase = 0
            return

        # Now run p1->p2 (arm)
        fut = self._send_arm_action(self._points_p1_p2())
        if fut is None:
            self.phase = 0
            return

        fut.add_done_callback(self._on_arm12_goal)

    def _on_arm12_goal(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"Arm p1->p2 send failed: {e}")
            self.phase = 0
            return

        if not goal_handle.accepted:
            self.get_logger().error("Arm p1->p2 goal rejected")
            self.phase = 0
            return

        goal_handle.get_result_async().add_done_callback(self._on_arm12_done)

    def _on_arm12_done(self, future):
        # Close gripper AFTER p2 is actually reached
        fut = self.send_gripper(self.gripper_close)
        if fut is None:
            self.phase = 0
            return
        fut.add_done_callback(self._on_close_sent)

    def _on_close_sent(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"Close gripper send failed: {e}")
            self.phase = 0
            return

        if not goal_handle.accepted:
            self.get_logger().error("Close gripper goal rejected")
            self.phase = 0
            return

        self.follow_peg = True
        self.get_logger().info("Peg follow enabled (visual grasp)")

        # Now run p3->p4 (arm)
        fut = self._send_arm_action(self._points_p3_p4())
        if fut is None:
            self.phase = 0
            return
        fut.add_done_callback(self._on_arm34_goal)

    def _on_arm34_goal(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"Arm p3->p4 send failed: {e}")
            self.phase = 0
            return

        if not goal_handle.accepted:
            self.get_logger().error("Arm p3->p4 goal rejected")
            self.phase = 0
            return

        goal_handle.get_result_async().add_done_callback(self._on_arm34_done)

    def _on_arm34_done(self, future):
        # Open gripper AFTER insertion is done
        self.follow_peg = False
        self.send_gripper(self.gripper_open)
        self.get_logger().info("Peg follow disabled (peg left in final pose)")
        self.phase = 5


def main():
    rclpy.init()
    node = PegPickInsertDemoGz()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
