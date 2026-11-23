"""Pixel -> belt homography calibration (the PDF's "calibrate, then trust" step).

    ros2 run factory_control calibrate_node --ros-args -p use_sim_time:=true

1. Places a white calibration plate at belt height with five black dots at
   known belt coordinates (in the real build you lay the printed plate there).
2. Detects the dots, matches them to the known points using a prediction from
   the nominal camera mount, fits the homography on four corner dots, and
   checks it on the fifth (pass: residual < 2 mm).
3. Writes the result to calibration.yaml (homography, camera nadir, camera
   height, px/mm, residual) and removes the plate.

The sim camera has no lens distortion, so the checkerboard intrinsics step is
skipped here; on the real C920 run it first and undistort before this step.
"""

import math
import os
import sys
import time

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import DeleteEntity, SpawnEntity
from sensor_msgs.msg import CameraInfo, Image

from factory_control import geometry as g

DOTS = {                      # belt-plane metres; D5 is the check point
    'D1': (0.350, 0.330), 'D2': (0.590, 0.330), 'D3': (0.590, 0.440),
    'D4': (0.350, 0.440), 'D5': (0.500, 0.385),
}
PLATE_CENTRE = (0.470, 0.385)
PLATE_SIZE = (0.270, 0.150)
DOT_RADIUS = 0.006
DEFAULT_OUT = os.path.expanduser('~/robot_arm_ws/src/factory_bringup/config/calibration.yaml')


PLATE_TOP = 0.001             # plate surface above the belt (thin card on the belt)


def plate_sdf():
    dots = ''.join(
        f'<visual name="{n}"><pose>{x - PLATE_CENTRE[0]:.4f} {y - PLATE_CENTRE[1]:.4f} 0.0003 0 0 0</pose>'
        f'<geometry><cylinder><radius>{DOT_RADIUS}</radius><length>0.0004</length></cylinder></geometry>'
        f'<material><ambient>0 0 0 1</ambient><diffuse>0 0 0 1</diffuse></material></visual>'
        for n, (x, y) in DOTS.items())
    return f'''<sdf version="1.9"><model name="calibration_plate"><static>true</static>
<link name="plate">
  <visual name="board"><pose>0 0 -0.0004 0 0 0</pose>
    <geometry><box><size>{PLATE_SIZE[0]} {PLATE_SIZE[1]} 0.0008</size></box></geometry>
    <material><ambient>0.95 0.95 0.95 1</ambient><diffuse>0.95 0.95 0.95 1</diffuse></material></visual>
  {dots}
</link></model></sdf>'''


def nominal_pixel(info: CameraInfo, p):
    """Predict where a belt point appears, from the nominal camera mount.

    The camera looks straight down with the long image axis along the belt:
    image right = world +x, image down = world -y.
    """
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    depth = g.CAMERA_XYZ[2] - g.BELT_TOP_Z
    dx, dy = p[0] - g.CAMERA_XYZ[0], p[1] - g.CAMERA_XYZ[1]
    return np.array([cx + fx * dx / depth, cy - fy * dy / depth])


def find_dots(bgr):
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    _, dark = cv2.threshold(gray, 70, 255, cv2.THRESH_BINARY_INV)
    contours, _ = cv2.findContours(dark, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    out = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < 50:
            continue
        perim = cv2.arcLength(c, True)
        circularity = 4 * math.pi * area / (perim * perim)
        if circularity < 0.75:
            continue
        m = cv2.moments(c)
        out.append((np.array([m['m10'] / m['m00'], m['m01'] / m['m00']]), area))
    return out


class Calibrate(Node):

    def __init__(self):
        super().__init__('calibrate')
        self.declare_parameter('output', DEFAULT_OUT)
        self.declare_parameter('camera_height', g.CAMERA_XYZ[2] - g.BELT_TOP_Z)
        self.declare_parameter('max_residual_mm', 2.0)
        self.bridge = CvBridge()
        self.image = None
        self.info = None
        self.create_subscription(Image, '/camera/image_raw', self.on_image, 2)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.on_info, 2)
        self.create_cli = self.create_client(SpawnEntity, '/world/factory/create')
        self.remove_cli = self.create_client(DeleteEntity, '/world/factory/remove')

    def on_image(self, msg):
        self.image = (self.get_clock().now(), self.bridge.imgmsg_to_cv2(msg, 'bgr8'))

    def on_info(self, msg):
        self.info = msg

    def call(self, cli, req):
        if not cli.wait_for_service(timeout_sec=20.0):
            raise RuntimeError(f'{cli.srv_name} not available')
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=20.0)
        return fut.result()

    def fresh_image(self, after, timeout=30.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.image and self.info and self.image[0] > after:
                return self.image[1]
        raise RuntimeError('no camera image')

    def run(self):
        req = SpawnEntity.Request()
        req.entity_factory.name = 'calibration_plate'
        req.entity_factory.sdf = plate_sdf()
        # the request pose overrides any pose inside the SDF, so set it here
        req.entity_factory.pose.position.x = PLATE_CENTRE[0]
        req.entity_factory.pose.position.y = PLATE_CENTRE[1]
        req.entity_factory.pose.position.z = g.BELT_TOP_Z + PLATE_TOP
        self.call(self.create_cli, req)
        t_spawn = self.get_clock().now()
        try:
            self.fresh_image(t_spawn)                  # skip one frame
            img = self.fresh_image(self.get_clock().now())
        finally:
            rm = DeleteEntity.Request()
            rm.entity.name = 'calibration_plate'
            rm.entity.type = Entity.MODEL
            self.call(self.remove_cli, rm)

        found = find_dots(img)
        self.get_logger().info(f'{len(found)} dot candidates')
        matched = {}
        for name, p in DOTS.items():
            guess = nominal_pixel(self.info, p)
            best = min(found, key=lambda d: np.linalg.norm(d[0] - guess), default=None)
            if best is None or np.linalg.norm(best[0] - guess) > 60:
                raise RuntimeError(f'dot {name} not found near {guess.round(1)}')
            matched[name] = best[0]

        corners = ['D1', 'D2', 'D3', 'D4']
        src = np.array([matched[n] for n in corners], dtype=np.float64)
        dst = np.array([DOTS[n] for n in corners], dtype=np.float64)
        H, _ = cv2.findHomography(src, dst)
        check = cv2.perspectiveTransform(matched['D5'].reshape(1, 1, 2), H).reshape(2)
        residual_mm = float(np.linalg.norm(check - np.array(DOTS['D5'])) * 1000)

        cx, cy = self.info.k[2], self.info.k[5]
        nadir = cv2.perspectiveTransform(np.array([[[cx, cy]]], dtype=np.float64), H).reshape(2)
        # H maps pixels to the plate surface, PLATE_TOP above the belt. A point at
        # that height projects onto the belt plane magnified about the nadir by
        # cam_h / (cam_h - PLATE_TOP); fold that scaling into H.
        cam_h = self.get_parameter('camera_height').value
        s = cam_h / (cam_h - PLATE_TOP)
        S = np.array([[s, 0, (1 - s) * nadir[0]], [0, s, (1 - s) * nadir[1]], [0, 0, 1]])
        H = S @ H
        px_per_mm = float(np.linalg.norm(matched['D2'] - matched['D1'])
                          / (np.linalg.norm(np.subtract(DOTS['D2'], DOTS['D1'])) * 1000))

        ok = residual_mm < self.get_parameter('max_residual_mm').value
        data = {
            'homography': [[float(v) for v in row] for row in H],
            'camera_xy': [float(nadir[0]), float(nadir[1])],
            'camera_height': float(self.get_parameter('camera_height').value),
            'px_per_mm': round(px_per_mm, 3),
            'check_residual_mm': round(residual_mm, 3),
            'image_size': [int(img.shape[1]), int(img.shape[0])],
            'stamp': time.strftime('%Y-%m-%d %H:%M:%S'),
        }
        out = self.get_parameter('output').value
        if ok:
            os.makedirs(os.path.dirname(out), exist_ok=True)
            with open(out, 'w') as f:
                f.write('# Pixel -> belt-plane calibration, written by calibrate_node.\n')
                yaml.safe_dump(data, f, sort_keys=False)
        print(f'homography fitted on D1-D4; check point D5 residual {residual_mm:.2f} mm '
              f'({"PASS" if ok else "FAIL"}, limit {self.get_parameter("max_residual_mm").value} mm)')
        print(f'camera nadir on belt: ({nadir[0] * 1000:.1f}, {nadir[1] * 1000:.1f}) mm, '
              f'{px_per_mm:.2f} px/mm')
        print(f'saved to {out}' if ok else 'not saved')
        return ok


def main():
    rclpy.init()
    node = Calibrate()
    ok = False
    try:
        ok = node.run()
    except RuntimeError as e:
        print(f'calibration failed: {e}')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(0 if ok else 1)
