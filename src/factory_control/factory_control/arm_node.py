"""Pick-and-place sequencer for the arm.

Offers /arm/pick_place (factory_interfaces/PickPlace). Each request runs the
motion phases from the design:

    open -> approach -> descend -> grip -> lift -> swing -> lower -> release
         -> raise -> home

Every pose is solved with the closed-form IK *before* the arm moves, so an
unreachable target is rejected without any motion. Joint moves go to
arm_controller / gripper_controller as FollowJointTrajectory goals, timed from
the largest joint change and the configured joint speed.

Also offers /arm/home and /arm/halt (std_srvs/Trigger). Halt cancels the
active goals and makes the running sequence fail; use it for the E-stop.
"""

import math
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

from factory_control import geometry as g
from factory_control.ik import JOINT_NAMES, UnreachableError, ik, wrist_roll_for
from factory_interfaces.srv import PickPlace

FINGERS = ('finger_left_joint', 'finger_right_joint')


class Halted(Exception):
    pass


class ArmNode(Node):

    def __init__(self):
        super().__init__('arm')
        self.declare_parameter('home', [math.radians(-20), math.pi / 2, math.pi / 2,
                                        math.pi / 2, 0.0])
        self.declare_parameter('joint_speed', 1.5)          # rad/s for the largest joint move
        self.declare_parameter('slow_speed', 0.8)           # rad/s near the product / bin
        self.declare_parameter('approach_height', 0.050)    # above the pick / place point
        self.declare_parameter('gripper_open', 0.027)       # per finger, m (limit 0.0275)
        self.declare_parameter('gripper_close', 0.013)      # target per finger: 2 mm squeeze on a 30 mm part
        self.declare_parameter('grip_settle_s', 0.5)
        self.declare_parameter('min_grip', 0.010)           # fingers closer than this = nothing held
        # Simulation only: ask the GraspAttach Gazebo plugin to lock the product
        # between the closed fingers (friction grasps of small parts slip in
        # Gazebo). Set false on real hardware.
        self.declare_parameter('grasp_assist', True)
        # With the pads open (35.5 mm to the outer face, 22 mm wide) a roll of
        # theta reaches 35.5*sin + 11*cos sideways from the product centre. The
        # rails are 30 mm from the lane centreline and a product can sit up to
        # ~8 mm off it, so keep that under ~21 mm: 15 deg. The 54 mm opening
        # still clears a 30 mm cube's 42 mm diagonal at any yaw.
        self.declare_parameter('max_wrist_roll_deg', 15.0)

        cb = ReentrantCallbackGroup()
        self.attach_pub = self.create_publisher(Empty, '/factory/grasp/attach', 10)
        self.detach_pub = self.create_publisher(Empty, '/factory/grasp/detach', 10)
        self.create_subscription(String, '/factory/grasp/state', self.on_grasp_state, 10,
                                 callback_group=cb)
        self.grasp_state = None
        self.arm_ac = ActionClient(self, FollowJointTrajectory,
                                   '/arm_controller/follow_joint_trajectory', callback_group=cb)
        self.grip_ac = ActionClient(self, FollowJointTrajectory,
                                    '/gripper_controller/follow_joint_trajectory',
                                    callback_group=cb)
        self.create_subscription(JointState, '/joint_states', self.on_joint_states, 10,
                                 callback_group=cb)
        self.create_service(PickPlace, '/arm/pick_place', self.on_pick_place, callback_group=cb)
        self.create_service(Trigger, '/arm/home', self.on_home, callback_group=cb)
        self.create_service(Trigger, '/arm/halt', self.on_halt, callback_group=cb)

        self.joints = {}
        self.busy = threading.Lock()
        self.halted = False
        self.active_goals = []
        self.get_logger().info('arm sequencer ready')

    # --- state -----------------------------------------------------------
    def on_joint_states(self, msg):
        self.joints.update(zip(msg.name, msg.position))

    def current_arm(self):
        return [self.joints.get(n, 0.0) for n in JOINT_NAMES]

    def on_grasp_state(self, msg):
        self.grasp_state = msg.data

    def assist_attach(self):
        """Ask the sim grasp assist to lock the product; True if it did."""
        if not self.get_parameter('grasp_assist').value:
            return True
        self.grasp_state = None
        self.attach_pub.publish(Empty())
        clock = self.get_clock()
        end = clock.now().nanoseconds * 1e-9 + 1.0
        end_wall = time.monotonic() + 10.0
        while self.grasp_state is None:
            if clock.now().nanoseconds * 1e-9 > end or time.monotonic() > end_wall:
                return False
            time.sleep(0.005)
        return self.grasp_state != 'none'

    def assist_detach(self):
        if self.get_parameter('grasp_assist').value:
            self.detach_pub.publish(Empty())

    # --- low-level moves ---------------------------------------------------
    def _send(self, client, names, positions, duration):
        if self.halted:
            raise Halted('arm halted')
        if not client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError(f'{client._action_name} not available')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(names)
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in positions]
        pt.velocities = [0.0] * len(positions)
        secs = max(duration, 0.2)
        pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        goal.trajectory.points = [pt]

        self.get_logger().debug(f'{client._action_name}: {pt.positions} in {secs:.2f} s')
        handle = self._wait(client.send_goal_async(goal), 5.0)
        if handle is None:
            raise RuntimeError(f'{client._action_name}: no response to goal')
        if not handle.accepted:
            raise RuntimeError(f'{client._action_name}: goal rejected')
        self.active_goals.append(handle)
        try:
            result = self._wait(handle.get_result_async(), secs + 5.0)
        finally:
            self.active_goals.remove(handle)
        if self.halted:
            raise Halted('arm halted')
        if result is None:
            raise RuntimeError(f'{client._action_name}: no result within {secs + 5.0:.1f} s')
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f'trajectory failed: status {result.status}, '
                               f'error {result.result.error_code} {result.result.error_string}')

    def _wait(self, future, timeout):
        """Wait for a future, with the timeout counted in node (sim) time.

        The simulation can run slower than real time, so a wall-clock timeout
        would fail moves that are on schedule. A wall-clock cap of 10x still
        catches a stalled simulator.
        """
        clock = self.get_clock()
        end_sim = clock.now().nanoseconds * 1e-9 + timeout
        end_wall = time.monotonic() + 10.0 * timeout + 5.0
        while not future.done():
            if clock.now().nanoseconds * 1e-9 > end_sim or time.monotonic() > end_wall:
                return None
            time.sleep(0.005)
        return future.result()

    def sleep_sim(self, secs):
        """Sleep for `secs` of node (sim) time."""
        clock = self.get_clock()
        end = clock.now().nanoseconds * 1e-9 + secs
        end_wall = time.monotonic() + 10.0 * secs + 2.0
        while clock.now().nanoseconds * 1e-9 < end and time.monotonic() < end_wall:
            time.sleep(0.005)

    def move_joints(self, q, speed=None):
        speed = speed or self.get_parameter('joint_speed').value
        delta = max(abs(a - b) for a, b in zip(q, self.current_arm())) if self.joints else 1.0
        self._send(self.arm_ac, JOINT_NAMES, q, delta / speed + 0.15)

    def move_to(self, x, y, z, roll, speed=None):
        self.move_joints(ik(x, y, z, wrist_roll=roll).as_list(), speed)

    def gripper(self, opening, duration=0.35):
        self._send(self.grip_ac, FINGERS, [opening, opening], duration)

    def finger_gap(self):
        return sum(self.joints.get(f, 0.0) for f in FINGERS)

    # --- services --------------------------------------------------------
    def on_pick_place(self, req, resp):
        if not self.busy.acquire(blocking=False):
            resp.success, resp.message = False, 'arm busy'
            return resp
        try:
            self.halted = False
            resp.success, resp.message = self.pick_place(req)
        finally:
            self.busy.release()
        if resp.success:
            self.get_logger().info(f'pick_place -> {req.bin}: {resp.message}')
        else:
            self.get_logger().error(f'pick_place -> {req.bin}: {resp.message}')
        return resp

    def pick_place(self, req):
        if req.bin not in g.BIN_ANGLES_DEG:
            return False, f'unknown bin {req.bin!r}'
        up = self.get_parameter('approach_height').value
        slow = self.get_parameter('slow_speed').value
        px, py = req.pick_x, req.pick_y
        pz = g.grasp_z(req.pick_z)              # fingers stop just above the belt
        # Line the finger pads up with the product's faces (any roll suits a
        # cylinder, and the cube rule keeps the roll within +/-45 deg).
        lim = math.radians(self.get_parameter('max_wrist_roll_deg').value)
        roll = max(-lim, min(lim, wrist_roll_for(px, py, req.pick_yaw, 'cube')))
        bx, by = g.bin_xy(req.bin)
        bz = g.grasp_z(g.PLACE_Z)
        # Solve every pose first: reject unreachable targets before moving.
        try:
            for p in ((px, py, pz + up), (px, py, pz), (bx, by, bz + up), (bx, by, bz)):
                ik(*p, wrist_roll=roll)
        except UnreachableError as e:
            return False, f'unreachable: {e}'

        t0 = self.get_clock().now()
        try:
            self.gripper(self.get_parameter('gripper_open').value)
            self.move_to(px, py, pz + up, roll)
            self.move_to(px, py, pz, roll, slow)
            self.gripper(self.get_parameter('gripper_close').value, 0.8)   # close gently
            self.sleep_sim(self.get_parameter('grip_settle_s').value)
            if (self.finger_gap() < self.get_parameter('min_grip').value
                    or not self.assist_attach()):
                self.gripper(self.get_parameter('gripper_open').value)
                self.move_to(px, py, pz + up, roll)
                self.home()
                return False, 'missed: no product between the fingers'
            self.move_to(px, py, pz + up, roll, slow)
            self.move_to(bx, by, bz + up, roll)
            self.move_to(bx, by, bz, roll, slow)
            self.assist_detach()
            self.gripper(self.get_parameter('gripper_open').value)
            self.sleep_sim(0.2)
            self.move_to(bx, by, bz + up, roll, slow)
            self.home()
        except Halted:
            self.assist_detach()
            return False, 'halted'
        except (RuntimeError, UnreachableError) as e:
            self.assist_detach()
            return False, str(e)
        secs = (self.get_clock().now() - t0).nanoseconds * 1e-9
        return True, f'done in {secs:.1f} s'

    def home(self):
        self.move_joints(list(self.get_parameter('home').value))

    def on_home(self, _req, resp):
        if not self.busy.acquire(blocking=False):
            resp.success, resp.message = False, 'arm busy'
            return resp
        try:
            self.halted = False
            self.home()
            resp.success, resp.message = True, 'home'
        except (Halted, RuntimeError) as e:
            resp.success, resp.message = False, str(e)
        finally:
            self.busy.release()
        return resp

    def on_halt(self, _req, resp):
        self.halted = True
        for h in list(self.active_goals):
            h.cancel_goal_async()
        # Hold the current pose so the controller stops tracking the old goal.
        if self.joints and self.arm_ac.server_is_ready():
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = list(JOINT_NAMES)
            pt = JointTrajectoryPoint(positions=self.current_arm(),
                                      time_from_start=Duration(sec=0, nanosec=100_000_000))
            goal.trajectory.points = [pt]
            self.arm_ac.send_goal_async(goal)
        resp.success, resp.message = True, 'halted'
        self.get_logger().warning('arm halted')
        return resp


def main():
    rclpy.init()
    node = ArmNode()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
