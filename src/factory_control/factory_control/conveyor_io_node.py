"""Hardware boundary for the belt and the cell I/O (simulation backend).

This node plays the part of the microcontroller firmware. In Phase 2 a serial
backend with the same ROS interface replaces it, so nothing above it changes.

ROS interface (same for sim and real):
  sub  /conveyor/cmd     std_msgs/Float64  requested belt speed, mm/s
  sub  /relay/cmd        std_msgs/Bool     supervisor asks for motor power
  sub  /heartbeat        std_msgs/Empty    supervisor heartbeat (watchdog)
  srv  /estop/set        std_srvs/SetBool  press (true) / release (false) the E-stop
  pub  /conveyor/speed   std_msgs/Float64  actual belt speed, mm/s
  pub  /ir1, /ir2        std_msgs/Bool     beam blocked (debounced)
  pub  /estop            std_msgs/Bool     E-stop pressed
  pub  /relay/state      std_msgs/Bool     motor power on
  pub  /io/watchdog_ok   std_msgs/Bool     heartbeat received within the window

Like the real relay circuit, motor power needs BOTH the E-stop chain intact and
the supervisor asking for it; software cannot override the E-stop. Losing power
stops the belt at once (no ramp). With power on, speed changes follow the
200 mm/s^2 ramp from the design.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Empty, Float64
from std_srvs.srv import SetBool

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class ConveyorIO(Node):

    def __init__(self):
        super().__init__('conveyor_io')
        self.declare_parameter('accel_mm_s2', 200.0)
        self.declare_parameter('max_speed_mm_s', 60.0)
        self.declare_parameter('watchdog_s', 2.0)
        self.declare_parameter('debounce_samples', 2)

        self.cmd_mm_s = 0.0
        self.speed_mm_s = 0.0
        self.relay_requested = False
        self.estop = False
        self.last_heartbeat = None
        self.beam = {'ir1': [False, 0], 'ir2': [False, 0]}   # state, consecutive opposite samples

        self.belt_pub = self.create_publisher(Float64, '/conveyor/belt_cmd_vel', 10)
        self.speed_pub = self.create_publisher(Float64, '/conveyor/speed', 10)
        self.estop_pub = self.create_publisher(Bool, '/estop', LATCHED)
        self.relay_pub = self.create_publisher(Bool, '/relay/state', LATCHED)
        self.wd_pub = self.create_publisher(Bool, '/io/watchdog_ok', LATCHED)
        self.ir_pub = {k: self.create_publisher(Bool, f'/{k}', LATCHED) for k in self.beam}

        self.create_subscription(Float64, '/conveyor/cmd', self.on_cmd, 10)
        self.create_subscription(Bool, '/relay/cmd', self.on_relay_cmd, 10)
        self.create_subscription(Empty, '/heartbeat', self.on_heartbeat, 10)
        for k in self.beam:
            self.create_subscription(LaserScan, f'/{k}/scan',
                                     lambda m, k=k: self.on_scan(k, m), 10)
        self.create_service(SetBool, '/estop/set', self.on_estop)

        self.dt = 0.01
        self.create_timer(self.dt, self.tick)
        self.create_timer(0.5, self.publish_states)
        self.publish_states()
        for k in self.beam:
            self.ir_pub[k].publish(Bool(data=False))
        self.get_logger().info('conveyor I/O (sim backend) ready')

    # --- inputs ----------------------------------------------------------
    def on_cmd(self, msg):
        vmax = self.get_parameter('max_speed_mm_s').value
        self.cmd_mm_s = max(0.0, min(vmax, msg.data))

    def on_relay_cmd(self, msg):
        self.relay_requested = msg.data

    def on_heartbeat(self, _msg):
        self.last_heartbeat = self.get_clock().now()

    def on_estop(self, req, resp):
        self.estop = req.data
        self.get_logger().warning('E-STOP PRESSED' if self.estop else 'E-stop released')
        self.publish_states()
        resp.success = True
        resp.message = 'pressed' if self.estop else 'released'
        return resp

    def on_scan(self, key, msg):
        r = msg.ranges[0] if msg.ranges else math.inf
        blocked = math.isfinite(r) and r < msg.range_max
        state = self.beam[key]
        if blocked != state[0]:
            state[1] += 1
            if state[1] >= self.get_parameter('debounce_samples').value:
                state[0], state[1] = blocked, 0
                self.ir_pub[key].publish(Bool(data=blocked))
        else:
            state[1] = 0

    # --- outputs ---------------------------------------------------------
    def watchdog_ok(self):
        if self.last_heartbeat is None:
            return False
        age = (self.get_clock().now() - self.last_heartbeat).nanoseconds * 1e-9
        return age <= self.get_parameter('watchdog_s').value

    def powered(self):
        return (not self.estop) and self.relay_requested and self.watchdog_ok()

    def tick(self):
        if self.powered():
            step = self.get_parameter('accel_mm_s2').value * self.dt
            diff = self.cmd_mm_s - self.speed_mm_s
            self.speed_mm_s += max(-step, min(step, diff))
        else:
            self.speed_mm_s = 0.0
        self.belt_pub.publish(Float64(data=self.speed_mm_s / 1000.0))
        self.speed_pub.publish(Float64(data=self.speed_mm_s))

    def publish_states(self):
        self.estop_pub.publish(Bool(data=self.estop))
        self.relay_pub.publish(Bool(data=self.powered()))
        self.wd_pub.publish(Bool(data=self.watchdog_ok()))


def main():
    rclpy.init()
    node = ConveyorIO()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
