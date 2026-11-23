"""Check that the URDF kinematic chain agrees with ik.fk / ik.ik.

Expands factory_description/urdf/arm.urdf.xacro, walks the joints from `world`
to `tcp` with numpy, and compares the TCP position and gripper direction with
the closed-form solution. Keeps the model and the solver from drifting apart.
"""

import math
import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

from factory_control import geometry as g
from factory_control.ik import ik, wrist_roll_for

XACRO = Path(__file__).resolve().parents[2] / 'factory_description' / 'urdf' / 'arm.urdf.xacro'

pytestmark = pytest.mark.skipif(
    shutil.which('xacro') is None or not XACRO.exists(),
    reason='xacro or the description package is not available')


def _rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _axis_angle(axis, a):
    x, y, z = axis / np.linalg.norm(axis)
    c, s, t = math.cos(a), math.sin(a), 1 - math.cos(a)
    return np.array([[t * x * x + c, t * x * y - s * z, t * x * z + s * y],
                     [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
                     [t * x * z - s * y, t * y * z + s * x, t * z * z + c]])


@pytest.fixture(scope='module')
def joints_by_child():
    urdf = subprocess.run(['xacro', str(XACRO), 'use_sim:=false'], check=True,
                          capture_output=True, text=True).stdout
    out = {}
    for j in ET.fromstring(urdf).findall('joint'):
        origin = j.find('origin')
        xyz = [float(v) for v in (origin.get('xyz', '0 0 0') if origin is not None
                                  else '0 0 0').split()]
        rpy = [float(v) for v in (origin.get('rpy', '0 0 0') if origin is not None
                                  else '0 0 0').split()]
        axis_el = j.find('axis')
        axis = np.array([float(v) for v in (axis_el.get('xyz') if axis_el is not None
                                            else '1 0 0').split()])
        out[j.find('child').get('link')] = dict(
            name=j.get('name'), type=j.get('type'), parent=j.find('parent').get('link'),
            xyz=np.array(xyz), rot=_rpy_matrix(*rpy), axis=axis)
    return out


def _tcp_pose(joints_by_child, q):
    chain, link = [], 'tcp'
    while link in joints_by_child:
        chain.append(joints_by_child[link])
        link = joints_by_child[link]['parent']
    R, p = np.eye(3), np.zeros(3)
    for j in reversed(chain):
        p = p + R @ j['xyz']
        R = R @ j['rot']
        if j['type'] == 'revolute':
            R = R @ _axis_angle(j['axis'], q.get(j['name'], 0.0))
    return R, p


@pytest.mark.parametrize('target', [
    (*g.PICK_XY, g.PICK_Z),
    (*g.PICK_XY, g.PICK_Z + 0.05),
    (*g.bin_xy('B1'), g.PLACE_Z),
    (*g.bin_xy('B3'), g.PLACE_Z),
    (*g.bin_xy('B5'), g.PLACE_Z),
    (0.50, 0.40, 0.10),
])
def test_urdf_tcp_matches_ik(joints_by_child, target):
    q = ik(*target).as_dict()
    R, p = _tcp_pose(joints_by_child, q)
    assert p == pytest.approx(target, abs=1e-6)
    # The gripper's long axis (local +x) must point straight down.
    assert R[:, 0] == pytest.approx([0.0, 0.0, -1.0], abs=1e-6)


@pytest.mark.parametrize('yaw_deg', [-40, -10, 0, 25, 44, 70])
def test_wrist_roll_aligns_fingers_with_cube_faces(joints_by_child, yaw_deg):
    x, y = g.PICK_XY
    yaw = math.radians(yaw_deg)
    roll = wrist_roll_for(x, y, yaw, 'cube')
    q = ik(x, y, g.PICK_Z, wrist_roll=roll).as_dict()
    R, _ = _tcp_pose(joints_by_child, q)
    finger_dir = math.atan2(R[1, 1], R[0, 1])        # world direction the fingers close along
    # Must be parallel to one of the cube's face normals (every 90 deg).
    assert math.remainder(finger_dir - yaw, math.pi / 2) == pytest.approx(0.0, abs=1e-6)
    assert abs(roll) <= math.pi / 4 + 1e-9
