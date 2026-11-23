"""Closed-form inverse kinematics for the Smart Mini Factory arm.

The arm is base yaw + a planar shoulder/elbow chain + a wrist pitch that keeps
the gripper pointing straight down, plus an optional wrist roll. Ported from the
``ik(r, z)`` function in "Smart Mini Factory.html" (elbow-up solution).

Joint conventions (match the URDF):

* ``base_yaw``: world heading of the arm minus ``ArmGeometry.base_yaw_zero``.
* ``shoulder``: elevation of link 1 above horizontal (0 = forward, pi/2 = up).
* ``elbow``: bend of link 2 relative to link 1, positive downward (0 = straight).
* ``wrist_pitch``: bend of the gripper relative to link 2, positive downward;
  ``wrist_pitch = pi/2 + shoulder - elbow`` keeps the gripper vertical.
* ``wrist_roll``: rotation of the gripper about its own (vertical) axis.
"""

import math
from dataclasses import astuple, dataclass

from factory_control.geometry import ArmGeometry, DEFAULT_ARM

JOINT_NAMES = ('base_yaw', 'shoulder', 'elbow', 'wrist_pitch', 'wrist_roll')


class UnreachableError(ValueError):
    """The target cannot be reached within the arm's envelope or joint limits."""


@dataclass(frozen=True)
class JointAngles:
    base_yaw: float
    shoulder: float
    elbow: float
    wrist_pitch: float
    wrist_roll: float = 0.0

    def as_list(self) -> list:
        return list(astuple(self))

    def as_dict(self) -> dict:
        return dict(zip(JOINT_NAMES, self.as_list()))


def wrap_angle(a: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    a = math.fmod(a + math.pi, 2.0 * math.pi)
    if a <= 0.0:
        a += 2.0 * math.pi
    return a - math.pi


def ik(x: float, y: float, z: float, wrist_roll: float = 0.0,
       arm: ArmGeometry = DEFAULT_ARM) -> JointAngles:
    """Joint angles that put the fingertip centre at world (x, y, z), gripper vertical.

    Raises UnreachableError if the target is outside the reachable annulus or
    any joint would leave its limits.
    """
    dx, dy = x - arm.base_xy[0], y - arm.base_xy[1]
    r = math.hypot(dx, dy)
    # The wrist sits L3 straight above the fingertip.
    dz = z + arm.l3 - arm.shoulder_z
    d = math.hypot(r, dz)
    if d > arm.max_reach:
        raise UnreachableError(
            f'target ({x:.3f}, {y:.3f}, {z:.3f}) needs {d * 1000:.1f} mm of reach, '
            f'limit is {arm.max_reach * 1000:.1f} mm')
    if d < arm.min_reach or r < 1e-6:
        raise UnreachableError(
            f'target ({x:.3f}, {y:.3f}, {z:.3f}) is too close to the arm base')

    c = (d * d - arm.l1 ** 2 - arm.l2 ** 2) / (2.0 * arm.l1 * arm.l2)
    elbow = math.acos(max(-1.0, min(1.0, c)))
    beta = math.atan2(arm.l2 * math.sin(elbow), arm.l1 + arm.l2 * math.cos(elbow))
    shoulder = math.atan2(dz, r) + beta
    wrist_pitch = math.pi / 2 + shoulder - elbow
    base_yaw = wrap_angle(math.atan2(dy, dx) - arm.base_yaw_zero)

    joints = JointAngles(base_yaw, shoulder, elbow, wrist_pitch, wrist_roll)
    check_limits(joints, arm)
    return joints


def check_limits(joints: JointAngles, arm: ArmGeometry = DEFAULT_ARM) -> None:
    """Raise UnreachableError if any joint is outside its limits."""
    eps = 1e-9
    for name, value in joints.as_dict().items():
        lo, hi = arm.limits[name]
        if not lo - eps <= value <= hi + eps:
            raise UnreachableError(
                f'{name} = {math.degrees(value):.1f} deg is outside '
                f'[{math.degrees(lo):.0f}, {math.degrees(hi):.0f}] deg')


def is_reachable(x: float, y: float, z: float, arm: ArmGeometry = DEFAULT_ARM) -> bool:
    try:
        ik(x, y, z, arm=arm)
    except UnreachableError:
        return False
    return True


def fk(joints: JointAngles, arm: ArmGeometry = DEFAULT_ARM) -> tuple:
    """World (x, y, z) of the fingertip centre for the given joint angles."""
    heading = joints.base_yaw + arm.base_yaw_zero
    a1 = joints.shoulder
    a2 = a1 - joints.elbow                    # absolute angle of link 2
    a3 = a2 - joints.wrist_pitch              # absolute angle of the gripper
    r = arm.l1 * math.cos(a1) + arm.l2 * math.cos(a2) + arm.l3 * math.cos(a3)
    h = arm.l1 * math.sin(a1) + arm.l2 * math.sin(a2) + arm.l3 * math.sin(a3)
    return (arm.base_xy[0] + r * math.cos(heading),
            arm.base_xy[1] + r * math.sin(heading),
            arm.shoulder_z + h)


def heading(x: float, y: float, arm: ArmGeometry = DEFAULT_ARM) -> float:
    """World heading of the arm when reaching (x, y), rad."""
    return math.atan2(y - arm.base_xy[1], x - arm.base_xy[0])


def wrist_roll_for(x: float, y: float, product_yaw: float, shape: str,
                   arm: ArmGeometry = DEFAULT_ARM) -> float:
    """Wrist roll that lines the finger pads up with a product's faces.

    With the gripper pointing down, the fingers close along the world direction
    ``heading + pi/2 - wrist_roll``. A cube is symmetric every 90 deg, so pick
    the smallest roll that aligns with any face; cylinders need no roll.
    """
    if shape != 'cube':
        return 0.0
    base = heading(x, y, arm) + math.pi / 2 - product_yaw
    roll = math.remainder(base, math.pi / 2)       # in [-pi/4, pi/4]
    return roll


def gripper_pitch(joints: JointAngles) -> float:
    """Absolute gripper angle from horizontal (-pi/2 means pointing straight down)."""
    return joints.shoulder - joints.elbow - joints.wrist_pitch
