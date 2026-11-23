"""Relay Gazebo's dynamic model poses to ROS, keeping the entity names.

ros_gz_bridge's Pose_V -> TFMessage conversion leaves frame ids empty for
/world/<world>/dynamic_pose/info, so this node reads the topic with the gz
transport Python bindings and republishes model poses on /gz/poses
(tf2_msgs/TFMessage, child_frame_id = model name, poses in the world frame).
Simulation only: it is how the spawner and logger know where products are.
"""

import threading

import rclpy
from geometry_msgs.msg import TransformStamped
from gz.msgs.pose_v_pb2 import Pose_V
from gz.transport import Node as GzNode
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


class PoseRelay(Node):

    def __init__(self):
        super().__init__('pose_relay')
        self.declare_parameter('world', 'factory')
        self.declare_parameter('prefixes', ['product_', 'trial_'])
        self.declare_parameter('rate', 20.0)
        self.prefixes = tuple(self.get_parameter('prefixes').value)
        self.pub = self.create_publisher(TFMessage, '/gz/poses', 10)
        self.lock = threading.Lock()
        self.latest = None
        self.gz = GzNode()
        topic = f'/world/{self.get_parameter("world").value}/dynamic_pose/info'
        if not self.gz.subscribe(Pose_V, topic, self.on_gz):
            raise RuntimeError(f'cannot subscribe to {topic}')
        self.create_timer(1.0 / self.get_parameter('rate').value, self.publish)
        self.get_logger().info(f'relaying {topic} -> /gz/poses')

    def on_gz(self, msg):
        with self.lock:
            self.latest = msg

    def publish(self):
        with self.lock:
            msg, self.latest = self.latest, None
        if msg is None:
            return
        out = TFMessage()
        stamp = self.get_clock().now().to_msg()
        for p in msg.pose:
            if not p.name.startswith(self.prefixes):
                continue
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = 'world'
            t.child_frame_id = p.name
            t.transform.translation.x = p.position.x
            t.transform.translation.y = p.position.y
            t.transform.translation.z = p.position.z
            t.transform.rotation.x = p.orientation.x
            t.transform.rotation.y = p.orientation.y
            t.transform.rotation.z = p.orientation.z
            t.transform.rotation.w = p.orientation.w
            out.transforms.append(t)
        self.pub.publish(out)


def main():
    rclpy.init()
    node = PoseRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
