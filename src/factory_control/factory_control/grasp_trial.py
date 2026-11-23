"""Grasp bench (milestone 5): repeated picks of a product parked at the pick zone.

    ros2 run factory_control grasp_trial --ros-args -p trials:=20 -p use_sim_time:=true

Each trial spawns a random product (shape, colour, yaw within +/-yaw_deg) at the
pick zone with the belt stopped, calls /arm/pick_place for the next bin in turn,
and checks from the Gazebo poses that the product came to rest inside that bin.
Prints a per-trial line and a summary; exits 0 only if every trial passed.
"""

import math
import random
import sys
import time

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import DeleteEntity, SpawnEntity
from tf2_msgs.msg import TFMessage

from factory_control import geometry as g
from factory_control.products import COLORS, SHAPES, ProductSpec, product_sdf
from factory_interfaces.srv import PickPlace


class GraspTrial(Node):

    def __init__(self):
        super().__init__('grasp_trial')
        self.declare_parameter('trials', 20)
        self.declare_parameter('yaw_deg', 30.0)
        self.declare_parameter('xy_jitter_mm', 5.0)
        self.declare_parameter('seed', 1)
        self.declare_parameter('world', 'factory')
        self.declare_parameter('defect', '')            # '', 'chip' or 'crack' for every product
        world = self.get_parameter('world').value
        self.create_cli = self.create_client(SpawnEntity, f'/world/{world}/create')
        self.remove_cli = self.create_client(DeleteEntity, f'/world/{world}/remove')
        self.pick_cli = self.create_client(PickPlace, '/arm/pick_place')
        self.poses = {}
        self.create_subscription(TFMessage, '/gz/poses', self.on_poses, 10)

    def on_poses(self, msg):
        for tf in msg.transforms:
            t = tf.transform.translation
            self.poses[tf.child_frame_id] = (t.x, t.y, t.z)

    def call(self, client, req, timeout=300.0):
        if not client.wait_for_service(timeout_sec=20.0):
            raise RuntimeError(f'{client.srv_name} not available')
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if not fut.done():
            raise RuntimeError(f'{client.srv_name} timed out')
        return fut.result()

    def settle(self, secs):
        end = time.monotonic() + secs
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def run(self):
        rng = random.Random(self.get_parameter('seed').value)
        n = self.get_parameter('trials').value
        yaw_max = math.radians(self.get_parameter('yaw_deg').value)
        jit = self.get_parameter('xy_jitter_mm').value / 1000.0
        bins = sorted(g.BIN_ANGLES_DEG)
        passed, times = 0, []
        for i in range(n):
            name = f'trial_{i}'
            spec = ProductSpec(name, rng.choice(sorted(COLORS)), rng.choice(SHAPES),
                               self.get_parameter('defect').value)
            x = g.PICK_XY[0] + rng.uniform(-jit, jit)
            y = g.PICK_XY[1] + rng.uniform(-jit, jit)
            yaw = rng.uniform(-yaw_max, yaw_max)
            req = SpawnEntity.Request()
            req.entity_factory.name = name
            req.entity_factory.sdf = product_sdf(spec)
            req.entity_factory.pose.position.x = x
            req.entity_factory.pose.position.y = y
            req.entity_factory.pose.position.z = g.PICK_Z + 0.001
            req.entity_factory.pose.orientation.z = math.sin(yaw / 2)
            req.entity_factory.pose.orientation.w = math.cos(yaw / 2)
            self.call(self.create_cli, req)
            self.settle(1.0)

            target = bins[i % len(bins)]
            pick = PickPlace.Request(pick_x=x, pick_y=y, pick_z=g.PICK_Z,
                                     pick_yaw=yaw, bin=target)
            t0 = self.get_clock().now()
            res = self.call(self.pick_cli, pick)
            dt = (self.get_clock().now() - t0).nanoseconds * 1e-9
            self.settle(1.5)

            pos = self.poses.get(name)
            bx, by = g.bin_xy(target)
            in_bin = (pos is not None
                      and math.hypot(pos[0] - bx, pos[1] - by) < g.BIN_DIAMETER / 2
                      and pos[2] < g.BIN_HEIGHT)
            ok = res.success and in_bin
            passed += ok
            if res.success:
                times.append(dt)
            where = 'none' if pos is None else f'({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})'
            print(f'trial {i + 1:2d}: {spec.color:6s} {spec.shape:8s} yaw {math.degrees(yaw):+5.1f} '
                  f'-> {target}  {"PASS" if ok else "FAIL"}  {res.message}  rest {where}',
                  flush=True)

            rm = DeleteEntity.Request()
            rm.entity.name = name
            rm.entity.type = Entity.MODEL
            self.call(self.remove_cli, rm)
            self.settle(0.5)

        avg = sum(times) / len(times) if times else float('nan')
        print(f'\n{passed}/{n} trials passed; mean pick-and-place cycle {avg:.1f} s (sim time)',
              flush=True)
        return passed == n


def main():
    rclpy.init()
    node = GraspTrial()
    ok = False
    try:
        ok = node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(0 if ok else 1)
