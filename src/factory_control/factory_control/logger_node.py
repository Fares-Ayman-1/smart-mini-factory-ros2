"""Item logger: one SQLite row per product the cell handled.

Listens to /cell/events from the supervisor. In simulation it also scores the
result against ground truth: it matches each event to the spawned product
nearest the pick point (from /gz/poses and /products/spawned), and a few
seconds after a placement checks where that product actually came to rest.

    ros2 run factory_control report        # print the summary from the database
"""

import json
import math
import os
import sqlite3
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from factory_control import geometry as g

DEFAULT_DB = os.path.expanduser('~/robot_arm_ws/data/factory.db')

SCHEMA = '''
CREATE TABLE IF NOT EXISTS items (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  run TEXT, t REAL, event TEXT, reason TEXT,
  color TEXT, shape TEXT, defective INTEGER, bin TEXT, cycle_s REAL,
  pick_x REAL, pick_y REAL,
  product TEXT, true_color TEXT, true_shape TEXT, true_defect TEXT,
  rest_x REAL, rest_y REAL, rest_z REAL, landed_bin TEXT
)'''


def bin_at(x, y, z):
    for name in g.BIN_ANGLES_DEG:
        bx, by = g.bin_xy(name)
        if math.hypot(x - bx, y - by) < g.BIN_DIAMETER / 2 and z < g.BIN_HEIGHT:
            return name
    return ''


class Logger(Node):

    def __init__(self):
        super().__init__('item_logger')
        self.declare_parameter('db', DEFAULT_DB)
        self.declare_parameter('run', '')
        self.declare_parameter('rest_check_s', 4.0)
        path = self.get_parameter('db').value
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self.db = sqlite3.connect(path)
        self.db.execute(SCHEMA)
        self.run = self.get_parameter('run').value or f'run_{int(self.now())}'
        self.truth = {}
        self.poses = {}
        self.pending = []           # (check_time, row_id, product)
        self.picking = {}           # detection id -> product under the gripper at pick start
        self.create_subscription(String, '/cell/events', self.on_event, 50)
        self.create_subscription(String, '/products/spawned', self.on_spawned, 50)
        self.create_subscription(TFMessage, '/gz/poses', self.on_poses, 10)
        self.create_timer(0.5, self.check_rest)
        self.get_logger().info(f'logging to {path} as {self.run}')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_spawned(self, msg):
        d = json.loads(msg.data)
        self.truth[d['name']] = d

    def on_poses(self, msg):
        for tf in msg.transforms:
            if tf.child_frame_id.startswith('product_'):
                t = tf.transform.translation
                self.poses[tf.child_frame_id] = (t.x, t.y, t.z)

    def nearest_product(self, x, y, max_d=0.03):
        best, bd = None, max_d
        for name, (px, py, pz) in self.poses.items():
            d = math.hypot(px - x, py - y)
            if d < bd and pz > g.BELT_TOP_Z - 0.005:
                best, bd = name, d
        return best

    def on_event(self, msg):
        e = json.loads(msg.data)
        det = e.get('detection') or {}
        if e['type'] == 'pick_start':
            # match now, while the product is still at the pick point
            self.picking[e.get('det_id')] = self.nearest_product(det['x'], det['y'])
            return
        product = self.picking.pop(e.get('det_id'), None)
        if product is None and det:
            product = self.nearest_product(det['x'], det['y'])
        truth = self.truth.get(product, {})
        cur = self.db.execute(
            'INSERT INTO items (run, t, event, reason, color, shape, defective, bin, cycle_s, '
            'pick_x, pick_y, product, true_color, true_shape, true_defect) '
            'VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)',
            (self.run, e['t'], e['type'], e.get('reason', ''), det.get('color'), det.get('shape'),
             int(bool(det.get('defective'))) if det else None, e.get('bin'), e.get('cycle_s'),
             det.get('x'), det.get('y'), product, truth.get('color'), truth.get('shape'),
             truth.get('defect')))
        self.db.commit()
        if product and e['type'] == 'placed':
            self.pending.append((self.now() + self.get_parameter('rest_check_s').value,
                                 cur.lastrowid, product))

    def check_rest(self):
        t = self.now()
        keep = []
        for when, row, product in self.pending:
            if t < when:
                keep.append((when, row, product))
                continue
            pos = self.poses.get(product)
            if pos:
                self.db.execute('UPDATE items SET rest_x=?, rest_y=?, rest_z=?, landed_bin=? '
                                'WHERE id=?', (*pos, bin_at(*pos), row))
                self.db.commit()
        self.pending = keep


def summarize(db_path, run=None):
    db = sqlite3.connect(db_path)
    if run is None:
        row = db.execute('SELECT run FROM items ORDER BY id DESC LIMIT 1').fetchone()
        if not row:
            return 'no items logged'
        run = row[0]
    rows = db.execute('SELECT event, color, shape, defective, bin, cycle_s, true_color, '
                      'true_shape, true_defect, landed_bin FROM items WHERE run=?',
                      (run,)).fetchall()
    placed = [r for r in rows if r[0] == 'placed']
    missed = [r for r in rows if r[0] == 'missed']
    undetected = [r for r in rows if r[0] == 'undetected']
    scored = [r for r in placed if r[6] is not None]
    cls_ok = sum(1 for r in scored if r[1] == r[6] and r[2] == r[7])
    defect_true = [r for r in scored if r[8]]
    defect_found = sum(1 for r in defect_true if r[3])
    false_alarm = sum(1 for r in scored if r[3] and not r[8])
    landed = [r for r in placed if r[9] is not None]
    landed_ok = sum(1 for r in landed if r[9] == r[4])
    cycles = [r[5] for r in placed if r[5]]
    attempts = len(placed) + len(missed)

    def pct(a, b):
        return f'{a}/{b} = {100.0 * a / b:.1f} %' if b else 'n/a'

    lines = [
        f'run {run}: {len(rows)} events',
        f'  picks succeeded        {pct(len(placed), attempts)}',
        f'  landed in chosen bin   {pct(landed_ok, len(landed))}',
        f'  classification correct {pct(cls_ok, len(scored))}  (colour and shape vs ground truth)',
        f'  defects detected       {pct(defect_found, len(defect_true))}',
        f'  false defect alarms    {pct(false_alarm, len(scored))}',
        f'  undetected at pick     {len(undetected)}',
        f'  mean cycle             {sum(cycles) / len(cycles):.1f} s (sim time)' if cycles else '',
    ]
    return '\n'.join(line for line in lines if line)


def main():
    rclpy.init()
    node = Logger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def report_main():
    path = sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith('-') else DEFAULT_DB
    print(summarize(path))
