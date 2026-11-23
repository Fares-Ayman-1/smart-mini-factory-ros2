"""Vision bench (milestone 6): every colour x shape, plus defective parts.

    ros2 run factory_control vision_trial --ros-args -p use_sim_time:=true

Needs the sim and vision_node running (and a calibration). Parks each product at
the pick zone with a random small offset and yaw, waits for /vision/detection,
and compares colour, shape, defect flag and position with the truth.
"""

import math
import random
import sys
import time

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import DeleteEntity, SpawnEntity

from factory_control import geometry as g
from factory_control.products import COLORS, SHAPES, ProductSpec, product_sdf
from factory_interfaces.msg import Detection


class VisionTrial(Node):

    def __init__(self):
        super().__init__('vision_trial')
        self.declare_parameter('seed', 3)
        self.declare_parameter('max_pos_err_mm', 4.0)
        self.create_cli = self.create_client(SpawnEntity, '/world/factory/create')
        self.remove_cli = self.create_client(DeleteEntity, '/world/factory/remove')
        self.det = None
        self.create_subscription(Detection, '/vision/detection', self.on_det, 10)

    def on_det(self, msg):
        self.det = (time.monotonic(), msg)

    def call(self, cli, req):
        if not cli.wait_for_service(timeout_sec=20.0):
            raise RuntimeError(f'{cli.srv_name} not available')
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=20.0)
        return fut.result()

    def detection_after(self, t, timeout=15.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.det and self.det[0] > t:
                return self.det[1]
        return None

    def run(self):
        rng = random.Random(self.get_parameter('seed').value)
        cases = [(c, s, '') for c in sorted(COLORS) for s in SHAPES]
        cases += [(rng.choice(sorted(COLORS)), rng.choice(SHAPES), d)
                  for d in ('chip', 'crack', 'chip', 'crack')]
        max_err = self.get_parameter('max_pos_err_mm').value
        ok_all, errs = 0, []
        for i, (color, shape, defect) in enumerate(cases):
            name = f'trial_v{i}'
            x = g.PICK_XY[0] + rng.uniform(-0.008, 0.008)
            y = g.PICK_XY[1] + rng.uniform(-0.006, 0.006)
            yaw = rng.uniform(-0.4, 0.4)
            req = SpawnEntity.Request()
            req.entity_factory.name = name
            req.entity_factory.sdf = product_sdf(ProductSpec(name, color, shape, defect))
            p = req.entity_factory.pose
            p.position.x, p.position.y, p.position.z = x, y, g.PICK_Z + 0.0005
            p.orientation.z, p.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            self.call(self.create_cli, req)
            time.sleep(2.5)                 # let frames of the previous product drain
            d = self.detection_after(time.monotonic())
            rm = DeleteEntity.Request()
            rm.entity.name, rm.entity.type = name, Entity.MODEL
            self.call(self.remove_cli, rm)
            time.sleep(1.0)
            if d is None:
                print(f'{i + 1:2d}: {color:6s} {shape:8s} {defect or "ok":5s} -> NO DETECTION')
                continue
            err = math.hypot(d.x - x, d.y - y) * 1000
            errs.append(err)
            good = (d.color == color and d.shape == shape and d.defective == bool(defect)
                    and err <= max_err)
            ok_all += good
            print(f'{i + 1:2d}: {color:6s} {shape:8s} {defect or "ok":5s} -> '
                  f'{d.color:6s} {d.shape:8s} {"defect" if d.defective else "ok":6s} '
                  f'pos err {err:4.1f} mm  {"PASS" if good else "FAIL"}', flush=True)
        n = len(cases)
        print(f'\n{ok_all}/{n} correct; position error mean {sum(errs) / max(len(errs), 1):.1f} mm, '
              f'max {max(errs, default=float("nan")):.1f} mm')
        return ok_all == n


def main():
    rclpy.init()
    node = VisionTrial()
    ok = False
    try:
        ok = node.run()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(0 if ok else 1)
