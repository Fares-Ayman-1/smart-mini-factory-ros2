"""Cell and arm geometry for the Smart Mini Factory.

All values come from "Smart Mini Factory - Hardware Architecture" and are in SI
units (metres, radians). The world frame is the baseplate's front-left corner:
x along the belt, y toward the back, z up.
"""

import math
from dataclasses import dataclass, field

# --- Conveyor -------------------------------------------------------------
BELT_TOP_Z = 0.050          # belt surface height
BELT_CENTRE_Y = 0.380       # lane centreline
BELT_X_START = 0.060        # idler roller axis
BELT_X_END = 0.660          # drive roller axis
IR1_X = 0.140               # entry beam
IR2_X = 0.540               # pick-stop beam
PICK_XY = (0.560, 0.380)    # pick-zone centre

# --- Products -------------------------------------------------------------
PRODUCT_SIZE = 0.030        # 30 mm cubes / cylinders
PICK_Z = BELT_TOP_Z + PRODUCT_SIZE / 2      # product centre on the belt

# --- Gripper --------------------------------------------------------------
FINGER_LENGTH = 0.040       # finger pads, centred on the fingertip centre (TCP)
FINGER_CLEARANCE = 0.003    # finger tips stop this far above the belt


def grasp_z(product_centre_z: float, product_height: float = PRODUCT_SIZE) -> float:
    """TCP height for a grasp: as low as possible without the fingers touching
    the surface the product stands on."""
    bottom = product_centre_z - product_height / 2
    return max(product_centre_z, bottom + FINGER_CLEARANCE + FINGER_LENGTH / 2)

# --- Camera ---------------------------------------------------------------
# DESIGN FINDING: the PDF mounts the camera at x = 440 mm. Its 283 mm view then
# ends at x = 581 mm, but at the 560 mm pick zone a 30 mm product's top face
# (magnified ~1.18x by parallax at 200 mm) reaches ~599 mm and is cut off. The
# sim mounts it at x = 470 mm (view 329..611 mm), with the long image axis
# along the belt.
CAMERA_XYZ = (0.470, 0.380, 0.250)
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720
CAMERA_HFOV = math.radians(70.0)

# --- Bins -----------------------------------------------------------------
BIN_RADIUS = 0.190
BIN_DIAMETER = 0.080
BIN_HEIGHT = 0.040
BIN_ANGLES_DEG = {'B1': 210.0, 'B2': 240.0, 'B3': 270.0, 'B4': 300.0, 'B5': 330.0}
REJECT_BIN = 'B5'
# Release with the product centre this high above the baseplate: 20 mm over
# the rim plus half a product.
PLACE_Z = BIN_HEIGHT + 0.020 + PRODUCT_SIZE / 2


@dataclass(frozen=True)
class ArmGeometry:
    """Fixed dimensions and joint limits of the 4-DOF arm (+ wrist roll)."""

    base_xy: tuple = (0.560, 0.240)
    shoulder_z: float = 0.100     # shoulder axis above the baseplate
    l1: float = 0.120             # shoulder -> elbow
    l2: float = 0.120             # elbow -> wrist
    l3: float = 0.090             # wrist -> fingertip centre
    # The 270 deg base servo is zeroed so its +/-135 deg range covers
    # 65..335 deg in the world frame; joint 0 therefore points at 200 deg.
    base_yaw_zero: float = math.radians(200.0)
    # Keep clear of full extension, where the shoulder carries most torque.
    reach_margin: float = 0.02
    limits: dict = field(default_factory=lambda: {
        'base_yaw': (math.radians(-135.0), math.radians(135.0)),
        'shoulder': (0.0, math.pi),
        'elbow': (0.0, math.pi),
        'wrist_pitch': (0.0, math.pi),
        'wrist_roll': (-math.pi / 2, math.pi / 2),
    })

    @property
    def max_reach(self) -> float:
        """Largest allowed shoulder-to-wrist distance."""
        return (self.l1 + self.l2) * (1.0 - self.reach_margin)

    @property
    def min_reach(self) -> float:
        """Smallest allowed shoulder-to-wrist distance."""
        return abs(self.l1 - self.l2) + 0.03


DEFAULT_ARM = ArmGeometry()


def bin_xy(name: str, arm: ArmGeometry = DEFAULT_ARM) -> tuple:
    """World (x, y) of a bin centre."""
    a = math.radians(BIN_ANGLES_DEG[name])
    return (arm.base_xy[0] + BIN_RADIUS * math.cos(a),
            arm.base_xy[1] + BIN_RADIUS * math.sin(a))
