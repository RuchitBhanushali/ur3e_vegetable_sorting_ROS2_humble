#!/usr/bin/env python3
# box_spawner.py
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import SpawnEntity

BOX_SDF = """<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="{name}">
    <pose>{x} {y} {z} 0 0 0</pose>
    <static>false</static>
    <link name="link">
      <inertial><mass>0.1</mass></inertial>
      <collision name="col">
        <geometry><box><size>0.04 0.04 0.04</size></box></geometry>
      </collision>
      <visual name="vis">
        <geometry><box><size>0.04 0.04 0.04</size></box></geometry>
      </visual>
    </link>
  </model>
</sdf>
"""

class BoxSpawner(Node):
    def __init__(self):
        super().__init__("box_spawner")

        self.world = self.declare_parameter("world", "default").value
        self.spawn_period = float(self.declare_parameter("spawn_period", 1.0).value)

        self.x = float(self.declare_parameter("x", 0.0).value)
        self.y = float(self.declare_parameter("y", 0.30).value)
        self.z = float(self.declare_parameter("z", 0.82).value)

        self.prefix = self.declare_parameter("name_prefix", "box").value

        self.client = self.create_client(SpawnEntity, f"/world/{self.world}/create")
        self.counter = 0
        self.timer = self.create_timer(self.spawn_period, self.tick)

    def tick(self):
        if not self.client.service_is_ready():
            self.get_logger().warn("Spawn service not ready yet")
            return

        name = f"{self.prefix}_{self.counter}"
        self.counter += 1

        req = SpawnEntity.Request()
        req.name = name
        req.xml = BOX_SDF.format(name=name, x=self.x, y=self.y, z=self.z)

        self.client.call_async(req)

def main():
    rclpy.init()
    rclpy.spin(BoxSpawner())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
