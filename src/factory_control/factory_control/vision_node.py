"""Vision node: camera frames -> product detections at the pick zone.

  sub /camera/image_raw         sensor_msgs/Image
  pub /vision/detection         factory_interfaces/Detection  product nearest the pick zone
  pub /vision/debug             sensor_msgs/Image             annotated, half size

Loads the homography from calibration.yaml (run calibrate_node first).
"""

import os

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

from factory_control import geometry as g
from factory_control.calibrate_node import DEFAULT_OUT
from factory_control.vision import VisionConfig, annotate, detect
from factory_interfaces.msg import Detection


def load_config(path):
    with open(path) as f:
        data = yaml.safe_load(f)
    return VisionConfig(homography=np.array(data['homography'], dtype=np.float64),
                        camera_xy=tuple(data['camera_xy']),
                        camera_height=float(data['camera_height']),
                        product_height=g.PRODUCT_SIZE, product_size=g.PRODUCT_SIZE)


class VisionNode(Node):

    def __init__(self):
        super().__init__('vision')
        self.declare_parameter('calibration', DEFAULT_OUT)
        self.declare_parameter('pick_radius', 0.060)   # only report products this close to the pick zone
        self.declare_parameter('debug_rate', 3.0)      # Hz
        path = self.get_parameter('calibration').value
        if not os.path.exists(path):
            raise SystemExit(f'calibration file {path} not found: run calibrate_node first')
        self.cfg = load_config(path)
        self.bridge = CvBridge()
        self.det_pub = self.create_publisher(Detection, '/vision/detection', 10)
        self.dbg_pub = self.create_publisher(Image, '/vision/debug', 2)
        self.create_subscription(Image, '/camera/image_raw', self.on_image, 2)
        self.last_debug = self.get_clock().now()
        self.count = 0
        self.get_logger().info(f'vision ready (calibration {path})')

    def on_image(self, msg):
        bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        blobs = detect(bgr, self.cfg)
        pick = np.array(g.PICK_XY)
        near = [b for b in blobs
                if np.hypot(b.x - pick[0], b.y - pick[1]) < self.get_parameter('pick_radius').value]
        if near:
            b = min(near, key=lambda b: np.hypot(b.x - pick[0], b.y - pick[1]))
            self.count += 1
            d = Detection()
            d.header = msg.header
            d.id = self.count
            d.color, d.shape = b.color, b.shape
            d.defective, d.defect_reason = b.defective, b.defect_reason
            d.x, d.y, d.yaw, d.width, d.confidence = b.x, b.y, b.yaw, b.width, b.confidence
            self.det_pub.publish(d)

        now = self.get_clock().now()
        rate = self.get_parameter('debug_rate').value
        if self.dbg_pub.get_subscription_count() and (now - self.last_debug).nanoseconds > 1e9 / rate:
            self.last_debug = now
            img = annotate(bgr, blobs, self.cfg, g.PICK_XY)
            img = cv2.resize(img, (img.shape[1] // 2, img.shape[0] // 2))
            out = self.bridge.cv2_to_imgmsg(img, 'bgr8')
            out.header = msg.header
            self.dbg_pub.publish(out)


def main():
    rclpy.init()
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
