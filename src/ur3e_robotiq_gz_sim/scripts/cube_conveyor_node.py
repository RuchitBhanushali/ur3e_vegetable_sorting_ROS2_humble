#!/usr/bin/env python3
import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class CubeConveyorNode(Node):
    def __init__(self):
        super().__init__("cube_conveyor")

        self.world = self.declare_parameter("world_name", "conveyor_base").value

        self.cube_prefix = self.declare_parameter("cube_prefix", "cube").value
        self.size = float(self.declare_parameter("cube_size", 0.04).value)
        self.mass = float(self.declare_parameter("cube_mass", 0.1).value)

        # Front conveyor
        self.front_spawn_x = float(self.declare_parameter("cube_front_spawn_x", -0.45).value)
        self.front_pick_x = float(self.declare_parameter("cube_front_pick_x", 0.48).value)
        self.front_speed = float(self.declare_parameter("cube_front_speed", 1.2).value)
        self.front_y = float(self.declare_parameter("cube_front_y", 0.30).value)
        self.front_z = float(self.declare_parameter("cube_front_z", 0.77).value)

        # NEW: spacing for the queued cube behind the active
        self.front_queue_spacing = float(self.declare_parameter("cube_queue_spacing", 0.22).value)

        # Back conveyor (opposite direction)
        self.back_start_x = float(self.declare_parameter("cube_back_start_x", 0.90).value)
        self.back_end_x = float(self.declare_parameter("cube_back_end_x", -0.45).value)
        self.back_speed = float(self.declare_parameter("cube_back_speed", 1.2).value)
        self.back_y = float(self.declare_parameter("cube_back_y", 0.30).value)
        self.back_z = float(self.declare_parameter("cube_back_z", 0.77).value)

        self.dt = float(self.declare_parameter("dt", 0.015).value)

        # Robot comms
        self.cube_ready_topic = self.declare_parameter("cube_ready_topic", "/cube_ready").value
        self.cube_done_topic = self.declare_parameter("cube_done_topic", "/cube_done").value
        self.ready_pub = self.create_publisher(String, self.cube_ready_topic, 10)
        self.done_sub = self.create_subscription(String, self.cube_done_topic, self._on_robot_done, 10)

        # State
        self.seq = 0

        # active/queued cubes on FRONT
        # dict: {"name": str, "x": float, "state": "move"|"hold"}
        self.active_front = None
        self.queued_front = None

        # cubes currently on BACK: list of {"name":str,"x":float}
        self.back_cubes = []

        self._unpause_world()
        self.timer = self.create_timer(self.dt, self._tick)

    # ---------- IGN helpers ----------
    def _unpause_world(self):
        subprocess.run(
            [
                "ign", "service", "-s", f"/world/{self.world}/control",
                "--reqtype", "ignition.msgs.WorldControl",
                "--reptype", "ignition.msgs.Boolean",
                "--timeout", "1000",
                "--req", "pause: false",
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

    def _set_pose(self, name: str, x: float, y: float, z: float):
        req = f'name: "{name}" position {{x: {x} y: {y} z: {z}}}'
        subprocess.run(
            [
                "ign", "service", "-s", f"/world/{self.world}/set_pose",
                "--reqtype", "ignition.msgs.Pose",
                "--reptype", "ignition.msgs.Boolean",
                "--timeout", "1000",
                "--req", req,
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

    def _create_box(self, name: str, x: float, y: float, z: float) -> bool:
        s = self.size
        m = self.mass
        sdf = (
            f"<sdf version='1.7'>"
            f"<model name='{name}'>"
            f"<pose>{x} {y} {z} 0 0 0</pose>"
            f"<static>false</static>"
            f"<link name='base_link'>"
            f"<inertial><mass>{m}</mass></inertial>"
            f"<collision name='col'><geometry><box><size>{s} {s} {s}</size></box></geometry></collision>"
            f"<visual name='vis'><geometry><box><size>{s} {s} {s}</size></box></geometry></visual>"
            f"</link>"
            f"</model>"
            f"</sdf>"
        )
        req = f'sdf: "{sdf}" name: "{name}" allow_renaming: false'
        p = subprocess.run(
            [
                "ign", "service", "-s", f"/world/{self.world}/create",
                "--reqtype", "ignition.msgs.EntityFactory",
                "--reptype", "ignition.msgs.Boolean",
                "--timeout", "1000",
                "--req", req,
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        return p.returncode == 0

    def _remove_entity(self, name: str):
        req = f'name: "{name}"'
        subprocess.run(
            [
                "ign", "service", "-s", f"/world/{self.world}/remove",
                "--reqtype", "ignition.msgs.Entity",
                "--reptype", "ignition.msgs.Boolean",
                "--timeout", "1000",
                "--req", req,
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

    # ---------- cube lifecycle ----------
    def _new_cube_name(self) -> str:
        n = f"{self.cube_prefix}_{self.seq}"
        self.seq += 1
        return n

    def _spawn_front_cube(self, x: float) -> dict | None:
        name = self._new_cube_name()
        if not self._create_box(name, x, self.front_y, self.front_z):
            return None
        self._set_pose(name, x, self.front_y, self.front_z)
        return {"name": name, "x": x, "state": "move"}

    def _ensure_active_and_queue(self):
        # Ensure active exists
        if self.active_front is None:
            self.active_front = self._spawn_front_cube(self.front_spawn_x)

        # Ensure queue exists (only if active exists)
        if self.active_front is not None and self.queued_front is None:
            qx = self.front_spawn_x - self.front_queue_spacing
            self.queued_front = self._spawn_front_cube(qx)

    # ---------- Robot sync ----------
    def _on_robot_done(self, msg: String):
        # When robot places active cube -> send it to back conveyor
        cube = msg.data.strip()
        if not cube:
            return
        if self.active_front is None:
            return
        if self.active_front["state"] != "hold":
            return
        if cube != self.active_front["name"]:
            return

        # teleport active cube onto back conveyor
        name = self.active_front["name"]
        self.back_cubes.append({"name": name, "x": self.back_start_x})
        self._set_pose(name, self.back_start_x, self.back_y, self.back_z)

        # Promote queued cube to active immediately
        self.active_front = self.queued_front
        self.queued_front = None  # will be re-spawned by ensure()

    # ---------- Main loop ----------
    def _tick(self):
        self._ensure_active_and_queue()

        # Move active front cube
        if self.active_front is not None:
            if self.active_front["state"] == "move":
                self.active_front["x"] += self.front_speed * self.dt
                x = self.active_front["x"]

                # reached pick -> hold + trigger robot + spawn next (queue) immediately
                if x >= self.front_pick_x:
                    x = self.front_pick_x
                    self.active_front["x"] = x
                    self.active_front["state"] = "hold"
                    self._set_pose(self.active_front["name"], x, self.front_y, self.front_z)

                    self.ready_pub.publish(String(data=self.active_front["name"]))

                    # IMPORTANT: spawn queued cube immediately if missing
                    if self.queued_front is None:
                        qx = self.front_spawn_x - self.front_queue_spacing
                        self.queued_front = self._spawn_front_cube(qx)
                    return

                self._set_pose(self.active_front["name"], x, self.front_y, self.front_z)

            elif self.active_front["state"] == "hold":
                # keep fixed at pick
                self._set_pose(self.active_front["name"], self.front_pick_x, self.front_y, self.front_z)

        # Move queued front cube (it must NOT stop)
        if self.queued_front is not None:
            self.queued_front["x"] += self.front_speed * self.dt
            self._set_pose(self.queued_front["name"], self.queued_front["x"], self.front_y, self.front_z)

        # Move back conveyor cubes and remove at end
        if self.back_cubes:
            direction = -1.0 if self.back_end_x < self.back_start_x else 1.0
            for c in list(self.back_cubes):
                c["x"] += direction * self.back_speed * self.dt
                self._set_pose(c["name"], c["x"], self.back_y, self.back_z)

                done = (direction < 0 and c["x"] <= self.back_end_x) or (direction > 0 and c["x"] >= self.back_end_x)
                if done:
                    self._remove_entity(c["name"])
                    self.back_cubes.remove(c)


def main():
    rclpy.init()
    node = CubeConveyorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
