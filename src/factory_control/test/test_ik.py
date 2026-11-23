"""Unit tests for the closed-form IK (milestone 1 checks from the plan)."""

import math

import pytest

from factory_control import geometry as g
from factory_control.ik import (JointAngles, UnreachableError, fk, gripper_pitch,
                                ik, is_reachable, wrap_angle)

ARM = g.DEFAULT_ARM


def test_pick_pose_matches_hand_calculation():
    j = ik(*g.PICK_XY, g.PICK_Z)
    # Pick is 140 mm out, straight across the belt (world heading 90 deg).
    assert math.degrees(j.base_yaw) == pytest.approx(90.0 - 200.0, abs=1e-6)
    # Values worked by hand from the law of cosines (r=140, dz=55 mm).
    assert math.degrees(j.elbow) == pytest.approx(102.4, abs=0.1)
    assert math.degrees(j.shoulder) == pytest.approx(72.6, abs=0.1)
    assert math.degrees(j.wrist_pitch) == pytest.approx(60.2, abs=0.1)


@pytest.mark.parametrize('name', sorted(g.BIN_ANGLES_DEG))
def test_every_bin_is_reachable(name):
    x, y = g.bin_xy(name)
    j = ik(x, y, g.PLACE_Z)
    assert fk(j) == pytest.approx((x, y, g.PLACE_Z), abs=1e-6)


def test_bin_centres_match_the_design_table():
    expected = {'B1': (0.396, 0.145), 'B2': (0.465, 0.076), 'B3': (0.560, 0.050),
                'B4': (0.655, 0.076), 'B5': (0.725, 0.145)}
    for name, xy in expected.items():
        assert g.bin_xy(name) == pytest.approx(xy, abs=0.001)


def test_round_trip_over_a_grid():
    checked = 0
    for r_mm in range(60, 235, 5):
        for deg in range(70, 335, 15):
            for z_mm in (40, 65, 85, 120, 160):
                a = math.radians(deg)
                p = (ARM.base_xy[0] + r_mm / 1000 * math.cos(a),
                     ARM.base_xy[1] + r_mm / 1000 * math.sin(a),
                     z_mm / 1000)
                try:
                    j = ik(*p)
                except UnreachableError:
                    continue
                assert fk(j) == pytest.approx(p, abs=1e-4)          # 0.1 mm
                assert gripper_pitch(j) == pytest.approx(-math.pi / 2, abs=1e-9)
                checked += 1
    assert checked > 500


def test_beyond_reach_at_pick_height_is_rejected():
    # Geometric limit at pick height is sqrt(240^2 - 55^2) = 233.6 mm.
    x = ARM.base_xy[0]
    assert not is_reachable(x, ARM.base_xy[1] + 0.234, g.PICK_Z)
    assert not is_reachable(x, ARM.base_xy[1] + 0.300, g.PICK_Z)
    assert is_reachable(x, ARM.base_xy[1] + 0.200, g.PICK_Z)


def test_too_close_is_rejected():
    with pytest.raises(UnreachableError):
        ik(ARM.base_xy[0], ARM.base_xy[1], 0.15)


def test_base_yaw_range_covers_belt_and_bins_but_not_behind():
    # 65..335 deg world heading is allowed; 0 deg (toward +x) is outside it.
    assert is_reachable(ARM.base_xy[0] + 0.15, ARM.base_xy[1], 0.1) is False
    assert is_reachable(*g.PICK_XY, g.PICK_Z)


def test_wrist_roll_out_of_range_is_rejected():
    with pytest.raises(UnreachableError):
        ik(*g.PICK_XY, g.PICK_Z, wrist_roll=math.radians(100))


def test_wrap_angle():
    assert wrap_angle(math.radians(190)) == pytest.approx(math.radians(-170))
    assert wrap_angle(math.pi) == pytest.approx(math.pi)
    assert wrap_angle(-math.pi) == pytest.approx(math.pi)


def test_joint_angles_helpers():
    j = JointAngles(0.1, 0.2, 0.3, 0.4, 0.5)
    assert j.as_list() == [0.1, 0.2, 0.3, 0.4, 0.5]
    assert j.as_dict()['elbow'] == 0.3
