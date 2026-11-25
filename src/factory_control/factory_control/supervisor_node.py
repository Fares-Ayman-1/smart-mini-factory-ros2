"""Cell supervisor: the state machine that runs the line (runs on the laptop / PC).

States: IDLE -> RUNNING -> STOPPING -> PICKING -> RUNNING, plus FAULT and ESTOP.

  RUNNING   relay on, belt at speed. IR1 counts entries. When IR2 trips, the
            belt runs `stop_delay_s` longer so the product stops centred in the
            pick zone, then stops.
  PICKING   once the belt is still, the latest vision detection picks the bin
            from the sorting table (defect or unknown -> reject bin) and the arm
            runs pick-and-place. A failed pick is retried once.
  FAULT     amber: product jam (IR2 blocked > jam_timeout_s), watchdog lost.
            Belt stops. Clear with /cell/reset.
  ESTOP     red: E-stop pressed. Arm halted, relay off. After the E-stop is
            released, acknowledge with /cell/reset.

Services: /cell/start, /cell/stop, /cell/reset (std_srvs/Trigger),
          /cell/pc_crash (std_srvs/SetBool: stop sending heartbeats, to test the
          microcontroller watchdog).
Publishes /cell/status (CellStatus, 5 Hz) and /cell/events (JSON per item).
"""

import json
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, Float64, String
from std_srvs.srv import SetBool, Trigger

from factory_control import geometry as g
from factory_interfaces.msg import CellStatus, Detection
from factory_interfaces.srv import PickPlace

TOWER = {'IDLE': 'off', 'RUNNING': 'green', 'STOPPING': 'green', 'PICKING': 'green',
         'FAULT': 'amber', 'ESTOP': 'red'}


def parse_table(entries):
    """['red/*=B1', 'green/cube=B2', ...] -> {(color, shape): bin}"""
    table = {}
    for e in entries:
        key, bin_ = e.split('=')
        color, shape = key.split('/')
        table[(color.strip(), shape.strip())] = bin_.strip()
    return table


def choose_bin(table, det: Detection, reject=g.REJECT_BIN):
    if det.defective or det.color == 'unknown' or det.shape == 'unknown':
        return reject
    for key in ((det.color, det.shape), (det.color, '*'), ('*', det.shape)):
        if key in table:
            return table[key]
    return reject


class Supervisor(Node):

    def __init__(self):
        super().__init__('supervisor')
        self.declare_parameter('auto_start', True)
        self.declare_parameter('belt_speed_mm_s', 30.0)
        self.declare_parameter('stop_delay_s', 1.1)
        self.declare_parameter('settle_s', 0.5)
        self.declare_parameter('detection_timeout_s', 3.0)
        self.declare_parameter('jam_timeout_s', 5.0)
        self.declare_parameter('pick_retries', 1)
        self.declare_parameter('sorting_table', ['red/*=B1', 'green/*=B2', 'blue/*=B3',
                                                 'yellow/*=B4'])
        self.table = parse_table(self.get_parameter('sorting_table').value)

        # Everything runs on one thread: no callback blocks (service calls are
        # async and polled from tick), and a single thread keeps the state
        # machine free of races between the sensor callbacks and tick.
        self.belt_pub = self.create_publisher(Float64, '/conveyor/cmd', 10)
        self.relay_pub = self.create_publisher(Bool, '/relay/cmd', 10)
        self.hb_pub = self.create_publisher(Empty, '/heartbeat', 10)
        self.status_pub = self.create_publisher(CellStatus, '/cell/status', 10)
        self.event_pub = self.create_publisher(String, '/cell/events', 50)
        self.create_subscription(Bool, '/ir1', self.on_ir1, 10)
        self.create_subscription(Bool, '/ir2', self.on_ir2, 10)
        self.create_subscription(Bool, '/estop', self.on_estop, 10)
        self.create_subscription(Bool, '/io/watchdog_ok', self.on_watchdog, 10)
        self.create_subscription(Float64, '/conveyor/speed', self.on_speed, 10)
        self.create_subscription(String, '/io/fault', self.on_io_fault, 10)
        self.reset_cli = self.create_client(Trigger, '/io/reset')
        self.create_subscription(Detection, '/vision/detection', self.on_detection, 10)
        self.pick_cli = self.create_client(PickPlace, '/arm/pick_place')
        self.halt_cli = self.create_client(Trigger, '/arm/halt')
        self.home_cli = self.create_client(Trigger, '/arm/home')
        self.create_service(Trigger, '/cell/start', self.srv_start)
        self.create_service(Trigger, '/cell/stop', self.srv_stop)
        self.create_service(Trigger, '/cell/reset', self.srv_reset)
        self.create_service(SetBool, '/cell/pc_crash', self.srv_pc_crash)

        self.state = 'IDLE'
        self.fault = ''
        self.t_state = self.now()
        self.ir1 = self.ir2 = False
        self.ir2_since = None
        self.estop = False
        self.watchdog_ok = False
        self.speed = 0.0
        self.io_fault = ''        # fault reported by the microcontroller, '' = none
        self.reset_t = 0.0
        self.last_det = None
        self.stop_at = None
        self.pick_future = None
        self.pick_det = None
        self.pick_bin = None
        self.pick_t0 = None
        self.attempt = 0
        self.heartbeat = True
        self.entered = self.sorted = self.rejected = 0
        self.bin_counts = {b: 0 for b in sorted(g.BIN_ANGLES_DEG)}
        self.done_times = deque()

        self.create_timer(0.02, self.tick)
        self.create_timer(0.2, self.publish_status)
        self.create_timer(0.2, self.send_heartbeat)
        if self.get_parameter('auto_start').value:
            self.create_timer(3.0, self.auto_start_once)
        self.get_logger().info(f'supervisor ready, sorting table {self.table}')

    # --- helpers ---------------------------------------------------------
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def set_state(self, s, fault=''):
        if s != self.state or fault != self.fault:
            msg = f'{self.state} -> {s}' + (f' ({fault})' if fault else '')
            # rclpy forbids one call site logging at different severities
            if s in ('FAULT', 'ESTOP'):
                self.get_logger().warning(msg)
            else:
                self.get_logger().info(msg)
        self.state, self.fault, self.t_state = s, fault, self.now()

    def belt(self, mm_s):
        self.belt_pub.publish(Float64(data=float(mm_s)))

    def relay(self, on):
        self.relay_pub.publish(Bool(data=on))

    def emit(self, **event):
        event['t'] = self.now()
        self.event_pub.publish(String(data=json.dumps(event)))

    # --- inputs ----------------------------------------------------------
    def on_ir1(self, msg):
        if msg.data and not self.ir1 and self.state in ('RUNNING', 'STOPPING'):
            self.entered += 1
        self.ir1 = msg.data

    def on_ir2(self, msg):
        if msg.data and not self.ir2:
            self.ir2_since = self.now()
            if self.state == 'RUNNING':
                self.stop_at = self.now() + self.get_parameter('stop_delay_s').value
                self.set_state('STOPPING')
        if not msg.data:
            self.ir2_since = None
        self.ir2 = msg.data

    def on_estop(self, msg):
        if msg.data and not self.estop:
            self.estop = True
            self.enter_estop()
        elif not msg.data:
            self.estop = False

    def on_watchdog(self, msg):
        self.watchdog_ok = msg.data

    def on_io_fault(self, msg):
        self.io_fault = msg.data
        if msg.data and self.state in ('RUNNING', 'STOPPING', 'PICKING'):
            self.enter_fault('microcontroller: ' + msg.data)

    def on_speed(self, msg):
        self.speed = msg.data

    def on_detection(self, msg):
        self.last_det = (self.now(), msg)

    # --- services --------------------------------------------------------
    def auto_start_once(self):
        if self.state == 'IDLE' and not getattr(self, '_auto_done', False):
            self._auto_done = True
            self.start()

    def start(self):
        if self.estop:
            return False, 'E-stop is pressed'
        if self.state in ('FAULT', 'ESTOP'):
            return False, f'in {self.state}: call /cell/reset first'
        self.relay(True)
        self.belt(self.get_parameter('belt_speed_mm_s').value)
        self.set_state('RUNNING')
        return True, 'running'

    def srv_start(self, _req, resp):
        resp.success, resp.message = self.start()
        return resp

    def srv_stop(self, _req, resp):
        self.belt(0.0)
        if self.state != 'PICKING':
            self.set_state('IDLE')
        resp.success, resp.message = True, 'stopping (an active pick finishes first)'
        self._stop_requested = True
        return resp

    def srv_reset(self, _req, resp):
        if self.estop:
            resp.success, resp.message = False, 'release the E-stop first'
            return resp
        if self.state not in ('FAULT', 'ESTOP'):
            resp.success, resp.message = True, f'nothing to reset ({self.state})'
            return resp
        self.ir2_since = self.now() if self.ir2 else None
        self.set_state('IDLE')
        self.reset_t = self.now()
        if self.reset_cli.service_is_ready():     # the microcontroller clears its own fault too
            self.reset_cli.call_async(Trigger.Request())
        self.relay(True)
        if self.home_cli.service_is_ready():
            self.home_cli.call_async(Trigger.Request())
        resp.success, resp.message = self.start()
        return resp

    def srv_pc_crash(self, req, resp):
        self.heartbeat = not req.data
        resp.success = True
        resp.message = 'heartbeat stopped' if req.data else 'heartbeat resumed'
        self.get_logger().warning(resp.message)
        return resp

    def send_heartbeat(self):
        if self.heartbeat:
            self.hb_pub.publish(Empty())

    # --- state machine ---------------------------------------------------
    def enter_estop(self):
        self.belt(0.0)
        self.relay(False)
        if self.halt_cli.service_is_ready():
            self.halt_cli.call_async(Trigger.Request())
        if self.pick_future is not None:
            self.emit(type='aborted', reason='estop', bin=self.pick_bin,
                      detection=self.det_dict(self.pick_det))
            self.pick_future = None
        self.set_state('ESTOP', 'E-stop pressed')

    def enter_fault(self, why):
        self.belt(0.0)
        self.set_state('FAULT', why)

    def tick(self):
        t = self.now()
        s = self.state
        if s in ('ESTOP',):
            self.relay(False)
            return
        if s == 'FAULT' and self.pick_future is not None:
            self.collect_pick(t)          # a pick that was running when the fault hit
        if s in ('RUNNING', 'STOPPING', 'PICKING'):
            self.relay(True)
            # A fault that is still there 2 s after a reset (the on-change message was missed).
            if self.io_fault and t - self.reset_t > 2.0:
                self.enter_fault('microcontroller: ' + self.io_fault)
                return
            # Jam = IR2 blocked while the belt should be moving products past it.
            # While PICKING the open fingers reach across the IR2 beam, so it is
            # expected to be blocked then.
            if (s == 'RUNNING' and self.ir2_since is not None
                    and t - max(self.ir2_since, self.t_state) > self.get_parameter('jam_timeout_s').value):
                self.enter_fault('product jam at IR2')
                return
            if not self.watchdog_ok and t - self.t_state > 3.0 and s != 'PICKING':
                self.enter_fault('microcontroller watchdog: no heartbeat')
                return

        if s == 'RUNNING':
            self.belt(self.get_parameter('belt_speed_mm_s').value)
        elif s == 'STOPPING':
            if t >= self.stop_at:
                self.belt(0.0)
            if t >= self.stop_at and self.speed <= 0.01:
                self.set_state('PICKING')
                self.attempt = 0
                self.pick_future = None
        elif s == 'PICKING':
            self.belt(0.0)
            self.step_picking(t)

    def step_picking(self, t):
        if self.pick_future is None:
            settle = self.get_parameter('settle_s').value
            if t - self.t_state < settle:
                return
            fresh = self.last_det is not None and self.last_det[0] >= self.t_state + settle
            if not fresh:
                if t - self.t_state > settle + self.get_parameter('detection_timeout_s').value:
                    self.emit(type='undetected')
                    self.get_logger().warning('no detection at the pick zone, restarting belt')
                    self.set_state('RUNNING')
                return
            det = self.last_det[1]
            self.pick_det = det
            self.pick_bin = choose_bin(self.table, det)
            self.pick_t0 = t
            # a cylinder's measured yaw is arbitrary: keep the fingers square to the belt
            yaw = det.yaw if det.shape == 'cube' else 0.0
            req = PickPlace.Request(pick_x=det.x, pick_y=det.y, pick_z=g.PICK_Z,
                                    pick_yaw=yaw, bin=self.pick_bin)
            self.emit(type='pick_start', det_id=det.id, detection=self.det_dict(det))
            self.get_logger().info(
                f'pick {det.color} {det.shape}{" (defect)" if det.defective else ""} '
                f'at ({det.x * 1000:.0f}, {det.y * 1000:.0f}) mm -> {self.pick_bin}')
            self.pick_future = self.pick_cli.call_async(req)
            return

        self.collect_pick(t)

    def collect_pick(self, t):
        """Handle the result of the running pick, if it has finished."""
        if not self.pick_future.done():
            return
        res = self.pick_future.result()
        self.pick_future = None
        cycle = t - self.pick_t0
        can_retry = self.state == 'PICKING'
        if res.success:
            if self.pick_bin == g.REJECT_BIN:
                self.rejected += 1
            else:
                self.sorted += 1
            self.bin_counts[self.pick_bin] += 1
            self.done_times.append(t)
            self.emit(type='placed', bin=self.pick_bin, cycle_s=round(cycle, 2),
                      det_id=self.pick_det.id, detection=self.det_dict(self.pick_det))
            self.finish_pick()
        elif can_retry and res.message == 'arm busy':
            self.t_state = t            # arm still finishing a move: ask again, no attempt used
        elif (can_retry and self.attempt < self.get_parameter('pick_retries').value
              and res.message != 'halted'):
            self.attempt += 1
            self.get_logger().warning(f'pick failed ({res.message}), retrying')
            self.t_state = t            # wait for a fresh detection
        else:
            self.emit(type='missed', bin=self.pick_bin, reason=res.message,
                      det_id=self.pick_det.id, detection=self.det_dict(self.pick_det))
            self.get_logger().error(f'pick failed: {res.message}; letting the product pass')
            self.finish_pick()

    def finish_pick(self):
        if getattr(self, '_stop_requested', False):
            self._stop_requested = False
            self.set_state('IDLE')
        elif self.state == 'PICKING':
            self.set_state('RUNNING')

    @staticmethod
    def det_dict(d):
        if d is None:
            return None
        return {'color': d.color, 'shape': d.shape, 'defective': d.defective,
                'reason': d.defect_reason, 'x': round(d.x, 4), 'y': round(d.y, 4),
                'yaw': round(d.yaw, 3)}

    def publish_status(self):
        t = self.now()
        while self.done_times and t - self.done_times[0] > 60.0:
            self.done_times.popleft()
        m = CellStatus()
        m.header.stamp = self.get_clock().now().to_msg()
        m.state, m.tower, m.fault = self.state, TOWER[self.state], self.fault
        m.entered, m.sorted, m.rejected = self.entered, self.sorted, self.rejected
        m.bin_counts = [self.bin_counts[b] for b in sorted(self.bin_counts)]
        m.items_per_min = float(len(self.done_times))
        total = self.sorted + self.rejected
        m.reject_rate = self.rejected / total if total else 0.0
        self.status_pub.publish(m)


def main():
    rclpy.init()
    node = Supervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
