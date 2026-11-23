"""Unit tests for the sorting table and the colour / defect logic."""

import cv2
import numpy as np
import pytest

from factory_control.supervisor_node import choose_bin, parse_table
from factory_control.vision import VisionConfig, classify_hue, detect
from factory_interfaces.msg import Detection

TABLE = parse_table(['red/*=B1', 'green/*=B2', 'blue/cube=B3', '*/cylinder=B4'])


def det(color, shape, defective=False):
    return Detection(color=color, shape=shape, defective=defective)


@pytest.mark.parametrize('d, expected', [
    (det('red', 'cube'), 'B1'),
    (det('green', 'cylinder'), 'B2'),
    (det('blue', 'cube'), 'B3'),
    (det('blue', 'cylinder'), 'B4'),
    (det('yellow', 'cube'), 'B5'),            # no rule -> reject
    (det('red', 'cube', defective=True), 'B5'),
    (det('unknown', 'cube'), 'B5'),
])
def test_choose_bin(d, expected):
    assert choose_bin(TABLE, d) == expected


@pytest.mark.parametrize('hue, name', [(2, 'red'), (175, 'red'), (28, 'yellow'),
                                       (60, 'green'), (115, 'blue'), (150, 'unknown')])
def test_classify_hue(hue, name):
    assert classify_hue(np.full(100, hue))[0] == name


def _synthetic_frame(marks=False):
    """Top-down frame at 4 px/mm with the nadir at the image centre and a flat
    (zero-height) square, so parallax plays no part."""
    img = np.zeros((400, 600, 3), np.uint8)
    cv2.rectangle(img, (240, 140), (360, 260), (30, 30, 220), -1)         # red 30 mm square
    if marks:
        cv2.rectangle(img, (280, 180), (320, 220), (10, 10, 10), -1)       # 10 mm dark chip
    # pixel -> metres: x = 0.44 + (u-300)/4000, y = 0.38 + (v-200)/4000
    H = np.array([[1 / 4000, 0, 0.44 - 300 / 4000], [0, 1 / 4000, 0.38 - 200 / 4000], [0, 0, 1]])
    cfg = VisionConfig(homography=H, camera_xy=(0.44, 0.38), camera_height=1e6,
                       product_height=0.0, lane_y=(0.2, 0.6), lane_x=(0.2, 0.7))
    return img, cfg


def test_detect_flat_square_position_and_shape():
    img, cfg = _synthetic_frame()
    blobs = detect(img, cfg)
    assert len(blobs) == 1
    b = blobs[0]
    assert (b.color, b.shape, b.defective) == ('red', 'cube', False)
    assert b.x == pytest.approx(0.44, abs=0.001)
    assert b.y == pytest.approx(0.38, abs=0.001)


def test_detect_dark_mark_is_a_defect():
    img, cfg = _synthetic_frame(marks=True)
    (b,) = detect(img, cfg)
    assert b.defective
    assert 'dark mark' in b.defect_reason
