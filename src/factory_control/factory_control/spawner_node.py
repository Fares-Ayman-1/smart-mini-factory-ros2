"""Feeds products onto the belt in Gazebo and cleans them up afterwards.

Spawns a random product (colour, shape, optional defect) at the idler end at the
configured feed rate, but only once the belt has carried the previous one at
least `min_gap_mm` away, so products never stack while the belt waits for a
pick. Publishes the ground truth of every product on /products/spawned (JSON),
which the logger uses to score the vision results.

Products are removed when they come to rest in a bin (after `bin_dwell_s`) or
fall off the belt.
"""

import json
import math
import random

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import DeleteEntity, SpawnEntity
from std_msgs.msg import Float64, String
from tf2_msgs.msg import TFMessage

from factory_control import geometry as g
from factory_control.products import product_sdf, random_spec


class Spawner(Node):

    def __init__(self):
        super().__init__('spawner')
        self.declare_parameter('world', 'factory')
        self.declare_parameter('feed_rate_per_min', 10.0)
        self.declare_parameter('defect_fraction', 0.12)
        self.declare_parameter('spawn_x', 0.085)
        self.declare_parameter('lateral_jitter_mm', 8.0)
        self.declare_parameter('min_gap_mm', 70.0)
        self.declare_parameter('max_products', 0)        # 0 = unlimited
        self.declare_parameter('seed', 0)                 # 0 = random
        self.declare_parameter('bin_dwell_s', 15.0)
        self.declare_parameter('enabled', True)

        seed = self.get_parameter('seed').value
        self.rng = random.Random(seed or None)
        world = self.get_parameter('world').value
        self.create_cli = self.create_client(SpawnEntity, f'/world/{world}/create')
        self.remove_cli = self.create_client(DeleteEntity, f'/world/{world}/remove')

        self.pub = self.create_publisher(String, '/products/spawned', 50)
        self.create_subscription(Float64, '/conveyor/speed', self.on_speed, 10)
        self.create_subscription(TFMessage, '/gz/poses', self.on_poses, 10)

        self.count = 0
        self.speed_mm_s = 0.0
        self.travel_since_spawn = 1e9
        self.since_spawn = 1e9
        self.in_bin_since = {}       # name -> first time seen resting in a bin
        self.removing = set()
        self.dt = 0.05
        self.create_timer(self.dt, self.tick)
        self.get_logger().info('spawner ready')

    # ------------------------------------------------------------------
    def on_speed(self, msg):
        self.speed_mm_s = msg.data

    def tick(self):
        self.travel_since_spawn += self.speed_mm_s * self.dt
        self.since_spawn += self.dt
        if not self.get_parameter('enabled').value:
            return
        rate = self.get_parameter('feed_rate_per_min').value
        max_n = self.get_parameter('max_products').value
        if rate <= 0 or (max_n and self.count >= max_n):
            return
        if (self.since_spawn >= 60.0 / rate
                and self.travel_since_spawn >= self.get_parameter('min_gap_mm').value
                and self.speed_mm_s > 0.0 and self.create_cli.service_is_ready()):
            self.spawn()

    def spawn(self):
        self.count += 1
        spec = random_spec(f'product_{self.count}',
                           self.get_parameter('defect_fraction').value, self.rng)
        jitter = self.get_parameter('lateral_jitter_mm').value / 1000.0
        req = SpawnEntity.Request()
        req.entity_factory.name = spec.name
        req.entity_factory.sdf = product_sdf(spec)
        p = req.entity_factory.pose
        p.position.x = self.get_parameter('spawn_x').value
        p.position.y = g.BELT_CENTRE_Y + self.rng.uniform(-jitter, jitter)
        p.position.z = g.BELT_TOP_Z + g.PRODUCT_SIZE / 2 + 0.002
        yaw = self.rng.uniform(-0.5, 0.5)
        p.orientation.z, p.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        req.entity_factory.relative_to = 'world'
        self.create_cli.call_async(req)
        self.since_spawn = 0.0
        self.travel_since_spawn = 0.0
        self.pub.publish(String(data=json.dumps({
            'name': spec.name, 'color': spec.color, 'shape': spec.shape,
            'defect': spec.defect, 'stamp': self.get_clock().now().nanoseconds * 1e-9})))
        self.get_logger().info(
            f'spawned {spec.name}: {spec.color} {spec.shape}'
            + (f' ({spec.defect})' if spec.defect else ''))

    # ------------------------------------------------------------------
    def on_poses(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        dwell = self.get_parameter('bin_dwell_s').value
        for tf in msg.transforms:
            name = tf.child_frame_id
            if not name.startswith('product_') or name in self.removing:
                continue
            t = tf.transform.translation
            in_bin = any(math.hypot(t.x - bx, t.y - by) < g.BIN_DIAMETER / 2 + 0.01
                         for bx, by in (g.bin_xy(b) for b in g.BIN_ANGLES_DEG)) \
                and t.z < g.BIN_HEIGHT + 0.01
            # Fell off the belt (end or side): below the belt surface and not in a
            # bin. Position alone is not enough: the arm carries products over
            # x > belt end on the way to B5.
            fell = not in_bin and t.z < g.BELT_TOP_Z - 0.01
            if in_bin:
                first = self.in_bin_since.setdefault(name, now)
                if now - first > dwell:
                    self.remove(name)
            elif fell:
                self.remove(name)

    def remove(self, name):
        if not self.remove_cli.service_is_ready():
            return
        self.removing.add(name)
        req = DeleteEntity.Request()
        req.entity.name = name
        req.entity.type = Entity.MODEL
        self.remove_cli.call_async(req)
        self.in_bin_since.pop(name, None)


def main():
    rclpy.init()
    node = Spawner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
