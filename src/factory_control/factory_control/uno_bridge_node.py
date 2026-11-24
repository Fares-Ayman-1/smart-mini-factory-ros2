"""Serial bridge between ROS and the Uno firmware (replaces conveyor_io_node on hardware).

Publishes the same topics as the simulated belt and I/O node, so the supervisor,
vision and arm code do not change:

  sub  /conveyor/cmd     Float64        belt speed, mm/s            -> BELT
  sub  /relay/cmd        Bool           ask for motor power         -> RELAY
  sub  /heartbeat        Empty          supervisor heartbeat        -> HB (forwarded 1:1)
  sub  /cell/status      CellStatus     tower colour                -> TWR
  sub  /uno/servos_us    UInt16MultiArray  six pulse widths         -> SRV
  sub  /uno/hood_led     UInt8          0..255                      -> LED
  srv  /io/reset         Trigger        clear a fault               -> RST
  pub  /conveyor/speed   Float64        actual belt speed, mm/s
  pub  /ir1 /ir2         Bool           beam blocked
  pub  /estop            Bool           E-stop open (pressed or wire broken)
  pub  /relay/state      Bool           motor relay closed
  pub  /io/watchdog_ok   Bool           Uno has fresh heartbeats AND the serial link is up
  pub  /io/fault         String         "" or the Uno's fault, or "serial link lost"
  pub  /uno/state        String         IDLE | RUN | PARK | FAULT
  pub  /uno/current_ma, /uno/supply_mv  Float64
  pub  /panel/start, /panel/stop        Bool

The bridge sends the desired state again whenever it differs from what the Uno
reports, so a dropped line or an Uno reset heals itself. It never clears a fault
on its own: only /io/reset sends RST (repeated for up to 3 s until the Uno confirms).
"""

import time

import rclpy
import serial
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Empty, Float64, String, UInt8, UInt16MultiArray
from std_srvs.srv import Trigger

from factory_control.uno_protocol import Parser, encode
from factory_interfaces.msg import CellStatus

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
TOWER_BITS = {'green': 1, 'amber': 2, 'red': 4, 'off': 0}


class UnoBridge(Node):

    def __init__(self):
        super().__init__('uno_bridge')
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('link_timeout_s', 1.0)
        # The firmware refuses to close the relay until it has been given a servo
        # pose ("home pulses first"). Set send_home:=true to send home_us after
        # connecting. The default is a placeholder: measure your own rest pose first.
        self.declare_parameter('send_home', False)
        self.declare_parameter('home_us', [1352, 1500, 1500, 1500, 1500, 2000])

        self.pubs = {
            'conveyor/speed': self.create_publisher(Float64, '/conveyor/speed', 10),
            'ir1': self.create_publisher(Bool, '/ir1', LATCHED),
            'ir2': self.create_publisher(Bool, '/ir2', LATCHED),
            'estop': self.create_publisher(Bool, '/estop', LATCHED),
            'relay/state': self.create_publisher(Bool, '/relay/state', LATCHED),
            'io/watchdog_ok': self.create_publisher(Bool, '/io/watchdog_ok', LATCHED),
            'io/fault': self.create_publisher(String, '/io/fault', LATCHED),
            'uno/state': self.create_publisher(String, '/uno/state', LATCHED),
            'uno/current_ma': self.create_publisher(Float64, '/uno/current_ma', 10),
            'uno/supply_mv': self.create_publisher(Float64, '/uno/supply_mv', 10),
            'panel/start': self.create_publisher(Bool, '/panel/start', LATCHED),
            'panel/stop': self.create_publisher(Bool, '/panel/stop', LATCHED),
        }
        self.msgtype = {'conveyor/speed': Float64, 'io/fault': String, 'uno/state': String,
                        'uno/current_ma': Float64, 'uno/supply_mv': Float64}
        self.shown = {}

        self.create_subscription(Float64, '/conveyor/cmd', self.on_belt, 10)
        self.create_subscription(Bool, '/relay/cmd', self.on_relay, 10)
        self.create_subscription(Empty, '/heartbeat', self.on_heartbeat, 10)
        self.create_subscription(CellStatus, '/cell/status', self.on_status, 10)
        self.create_subscription(UInt16MultiArray, '/uno/servos_us', self.on_servos, 10)
        self.create_subscription(UInt8, '/uno/hood_led', self.on_led, 10)
        self.create_service(Trigger, '/io/reset', self.on_reset)

        self.parser = Parser()
        self.ser = None
        self.seq = 0
        self.pending = {}                          # seq -> (cmd, args, time)
        self.t_open = 0.0
        self.last_rx = 0.0
        self.link_up = False
        self.tel = None
        self.home_sent = False
        self.rst_until = 0.0                       # keep sending RST until the Uno leaves FAULT

        self.want = {'relay': False, 'belt': 0, 'srv': None, 'led': 0, 'twr': 0}
        self.sent_t = {}
        self.acked = {'belt': None, 'srv': None, 'led': None, 'twr': None}

        self.publish('io/watchdog_ok', False)
        self.publish('io/fault', 'serial link lost')
        self.create_timer(0.005, self.on_rx)
        self.create_timer(0.05, self.on_tx)
        self.get_logger().info(f'uno bridge ready, port {self.get_parameter("port").value}')

    # --- publishing ------------------------------------------------------
    def publish(self, name, value):
        if name in self.shown and self.shown[name] == value and name not in ('uno/current_ma', 'uno/supply_mv', 'conveyor/speed'):
            return
        self.shown[name] = value
        msg = self.msgtype.get(name, Bool)()
        msg.data = value
        self.pubs[name].publish(msg)

    # --- ROS -> desired state --------------------------------------------
    def on_belt(self, m):
        self.want['belt'] = max(0, min(600, int(round(m.data * 10))))

    def on_relay(self, m):
        self.want['relay'] = bool(m.data)

    def on_heartbeat(self, _m):
        self.send('HB')

    def on_status(self, m):
        self.want['twr'] = TOWER_BITS.get(m.tower, 0)

    def on_servos(self, m):
        if len(m.data) == 6:
            self.want['srv'] = [int(x) for x in m.data]
        else:
            self.get_logger().warning('/uno/servos_us needs exactly 6 values')

    def on_led(self, m):
        self.want['led'] = int(m.data)

    def on_reset(self, _req, resp):
        resp.success = self.send('RST')
        # a line can be lost (link coming back, serial noise): repeat it until the Uno confirms
        self.rst_until = time.monotonic() + 3.0
        resp.message = 'RST sent' if resp.success else 'no serial link to the Uno (will keep trying for 3 s)'
        return resp

    # --- serial ----------------------------------------------------------
    def close(self, why):
        if self.ser:
            self.get_logger().warning(f'serial port closed: {why}')
            try:
                self.ser.close()
            except OSError:
                pass
        self.ser = None
        self.t_open = time.monotonic()

    def send(self, cmd, *args):
        if self.ser is None:
            return False
        self.seq = self.seq % 65535 + 1
        try:
            self.ser.write(encode(self.seq, cmd, *args))
        except (serial.SerialException, OSError) as e:
            self.close(str(e))
            return False
        now = time.monotonic()
        self.pending[self.seq] = (cmd, args, now)
        for s in [s for s, v in self.pending.items() if now - v[2] > 2.0]:
            del self.pending[s]
        return True

    def on_rx(self):
        now = time.monotonic()
        if self.ser is None:
            if now - self.t_open >= 2.0:
                self.t_open = now
                try:
                    self.ser = serial.Serial(self.get_parameter('port').value,
                                             self.get_parameter('baud').value,
                                             timeout=0, write_timeout=0.2)
                    self.parser = Parser()
                    self.get_logger().info('serial port open')
                except (serial.SerialException, OSError) as e:
                    self.get_logger().warning(f'cannot open serial port: {e}', throttle_duration_sec=10.0)
        else:
            try:
                data = self.ser.read(self.ser.in_waiting or 1)
            except (serial.SerialException, OSError) as e:
                self.close(str(e))
                data = b''
            for msg in self.parser.feed(data):
                self.last_rx = now
                self.on_message(msg)
        up = self.ser is not None and now - self.last_rx < self.get_parameter('link_timeout_s').value
        if up != self.link_up:
            self.link_up = up
            if not up:
                self.tel = None
                self.acked = {k: None for k in self.acked}
                self.home_sent = False
                self.publish('io/watchdog_ok', False)
                self.publish('relay/state', False)
                self.publish('io/fault', 'serial link lost')
                self.get_logger().warning('lost the Uno')
            else:
                self.get_logger().info('Uno connected')

    def on_message(self, msg):
        kind = msg[0]
        if kind == 'telemetry':
            self.on_telemetry(msg[1])
        elif kind == 'event':
            name = {'IR1': 'ir1', 'IR2': 'ir2'}.get(msg[1])
            if name:
                self.publish(name, bool(msg[2]))
        elif kind == 'reply':
            _, seq, ok, why = msg
            cmd, args, _t = self.pending.pop(seq, (None, (), 0))
            if seq == 0:
                self.get_logger().warning('the Uno rejected a line (checksum or length)', throttle_duration_sec=5.0)
            elif ok and cmd in ('BELT', 'LED', 'TWR'):
                self.acked[cmd.lower()] = int(args[0])
            elif ok and cmd == 'SRV':
                self.acked['srv'] = list(args)
            elif not ok:
                if cmd == 'BELT':
                    self.acked['belt'] = None
                self.get_logger().warning(f'the Uno refused {cmd}: {why}', throttle_duration_sec=5.0)

    def on_telemetry(self, t):
        self.tel = t
        self.publish('conveyor/speed', t['belt_dmm'] / 10.0)
        self.publish('ir1', t['ir1'])
        self.publish('ir2', t['ir2'])
        self.publish('estop', t['estop_open'])
        self.publish('relay/state', t['relay'])
        self.publish('panel/start', t['start'])
        self.publish('panel/stop', t['stop'])
        self.publish('uno/state', t['state'])
        self.publish('uno/current_ma', float(t['mA']))
        self.publish('uno/supply_mv', float(t['mV']))
        self.publish('io/watchdog_ok', t['hb_fresh'])
        # E-stop has its own topic and state in the supervisor; everything else is a fault.
        in_fault = t['state'] in ('FAULT', 'PARK') and t['fault_code'] != 1
        self.publish('io/fault', t['fault'] if in_fault else '')
        if self.get_parameter('send_home').value and not self.home_sent:
            self.want['srv'] = [int(x) for x in self.get_parameter('home_us').value]
            self.home_sent = True

    # --- desired state -> serial ----------------------------------------
    def due(self, key, now, wanted, acked, fast=0.4, slow=1.0):
        last = self.sent_t.get(key, 0.0)
        return now - last >= (fast if wanted != acked else slow)

    def on_tx(self):
        if not self.link_up or self.tel is None:
            return
        now = time.monotonic()
        w, t = self.want, self.tel
        if now < self.rst_until:
            if t['state'] != 'FAULT':
                self.rst_until = 0.0
            elif now - self.sent_t.get('rst', 0.0) >= 0.4:
                self.sent_t['rst'] = now
                self.send('RST')
        if w['relay'] != t['relay'] and now - self.sent_t.get('relay', 0.0) >= 0.4:
            self.sent_t['relay'] = now
            self.send('RELAY', int(w['relay']))
        if self.due('belt', now, w['belt'], self.acked['belt'], fast=0.1):
            self.sent_t['belt'] = now
            self.send('BELT', w['belt'])
        if w['srv'] and self.due('srv', now, w['srv'], self.acked['srv']):
            self.sent_t['srv'] = now
            self.send('SRV', *w['srv'])
        for key, cmd in (('led', 'LED'), ('twr', 'TWR')):
            if self.due(key, now, w[key], self.acked[key]):
                self.sent_t[key] = now
                self.send(cmd, w[key])


def main():
    rclpy.init()
    node = UnoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
