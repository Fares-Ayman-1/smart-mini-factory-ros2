"""Classical vision pipeline for the factory camera (pure functions, no ROS).

Steps, as in the design:
  1. HSV threshold: products are saturated; belt, baseplate and hood are not.
  2. Keep blobs inside the belt lane (the colour reference tile sits outside it).
  3. Colour from the median hue (red wraps around 0/180, so it has two ranges).
  4. Position: pixels -> belt plane through the calibration homography, then a
     parallax correction (below).
  5. Shape from how well the top face fills its bounding rectangle; defects
     from holes (dark marks) inside the product's colour mask.

Parallax: the camera is only ~200 mm above the belt, so the top of a 30 mm part
is seen displaced away from the point under the camera (the nadir), and its
side facing the camera is visible too. The top face is lit from above and
renders brighter than the sides, so it is segmented by brightness; its
centroid, mapped to the belt plane, sits magnified by k = H / (H - h) away from
the nadir, so scaling it back by 1/k gives the product's true centre. The top
face also gives the shape (square vs circle) and the cube's yaw.
"""

import math
from dataclasses import dataclass, field

import cv2
import numpy as np

# Hue ranges on OpenCV's 0..179 scale. Red wraps, so it gets two ranges.
HUE_RANGES = {
    'red': [(0, 10), (165, 180)],
    'yellow': [(18, 38)],
    'green': [(40, 88)],
    'blue': [(95, 135)],
}


@dataclass
class VisionConfig:
    homography: np.ndarray                 # 3x3, image pixel -> belt-plane metres
    camera_xy: tuple                       # nadir on the belt plane, metres
    camera_height: float = 0.200           # lens above the belt top, metres
    product_height: float = 0.030
    product_size: float = 0.030
    lane_y: tuple = (0.330, 0.430)         # belt-plane y range searched for products
    lane_x: tuple = (0.250, 0.650)
    sat_min: int = 90
    val_min: int = 50
    min_area_mm2: float = 300.0            # smaller blobs are noise
    hole_area_mm2: float = 25.0            # dark marks bigger than this = defect
    cube_fill: float = 0.90                # top-face fill ratio above this = cube (circle = 0.785)
    top_light_frac: float = 0.05           # top face = pixels within 5 % of the lightest (Lab L); the top is uniform, sides are a gradient


@dataclass
class Blob:
    color: str
    shape: str
    defective: bool
    defect_reason: str
    x: float                                # true centre on the belt, metres
    y: float
    yaw: float                              # rad, cube face orientation in [-pi/4, pi/4)
    width: float                            # metres
    confidence: float
    contour_px: np.ndarray = field(repr=False, default=None)


def to_plane(H, pts_px):
    """Map Nx2 pixel points to the belt plane (metres)."""
    pts = np.asarray(pts_px, dtype=np.float64).reshape(-1, 1, 2)
    return cv2.perspectiveTransform(pts, H).reshape(-1, 2)


def to_pixels(H, pts_m):
    """Map Nx2 belt-plane points (metres) to pixels."""
    pts = np.asarray(pts_m, dtype=np.float64).reshape(-1, 1, 2)
    return cv2.perspectiveTransform(pts, np.linalg.inv(H)).reshape(-1, 2)


def classify_hue(hues):
    """Colour name from an array of hue values (0..179), or 'unknown'."""
    if len(hues) == 0:
        return 'unknown', 0.0
    votes = {}
    for name, ranges in HUE_RANGES.items():
        votes[name] = sum(int(np.count_nonzero((hues >= lo) & (hues < hi))) for lo, hi in ranges)
    best = max(votes, key=votes.get)
    conf = votes[best] / len(hues)
    return (best if conf > 0.5 else 'unknown'), conf


def detect(bgr, cfg: VisionConfig):
    """Find products in a BGR frame. Returns a list of Blob."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    lightness = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)[..., 0]
    sat = cv2.inRange(hsv, (0, cfg.sat_min, cfg.val_min), (180, 255, 255))
    sat = cv2.morphologyEx(sat, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

    # Pixel area of 1 mm^2 near the image centre, for area thresholds.
    c_px = to_pixels(cfg.homography, [cfg.camera_xy, (cfg.camera_xy[0] + 0.001,
                                                      cfg.camera_xy[1] + 0.001)])
    px_per_mm = float(np.linalg.norm(c_px[1] - c_px[0]) / math.sqrt(2))
    mm2 = px_per_mm ** 2

    contours, hierarchy = cv2.findContours(sat, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
    blobs = []
    if hierarchy is None:
        return blobs
    hierarchy = hierarchy[0]
    k = cfg.camera_height / (cfg.camera_height - cfg.product_height)
    c = np.array(cfg.camera_xy)
    for i, cnt in enumerate(contours):
        if hierarchy[i][3] != -1:            # inner contour (a hole), handled with its parent
            continue
        area_px = cv2.contourArea(cnt)
        if area_px < cfg.min_area_mm2 * mm2:
            continue
        pts = to_plane(cfg.homography, cnt.reshape(-1, 2))
        centroid = pts.mean(axis=0)
        if not (cfg.lane_y[0] <= centroid[1] <= cfg.lane_y[1]
                and cfg.lane_x[0] <= centroid[0] <= cfg.lane_x[1]):
            continue

        # --- colour ---------------------------------------------------------
        mask = np.zeros(sat.shape, np.uint8)
        cv2.drawContours(mask, contours, i, 255, thickness=cv2.FILLED)
        solid = cv2.bitwise_and(mask, sat)
        color, conf = classify_hue(hsv[..., 0][solid > 0])

        # --- defects: holes inside the colour mask ---------------------------
        hole_mm2 = 0.0
        j = hierarchy[i][2]
        while j != -1:
            hole_mm2 = max(hole_mm2, cv2.contourArea(contours[j]) / mm2)
            j = hierarchy[j][0]
        defective = hole_mm2 > cfg.hole_area_mm2
        reason = f'dark mark {hole_mm2:.0f} mm^2' if defective else ''

        # --- top face, shape and position ----------------------------------
        top = top_face(lightness, solid, cfg.top_light_frac)
        if top is not None:
            top_pts = to_plane(cfg.homography, top.reshape(-1, 2))
            m = cv2.moments(top)
            top_centre = to_plane(cfg.homography, [[m['m10'] / m['m00'], m['m01'] / m['m00']]])[0]
        else:                                    # fall back to the outer band of the blob
            top_pts, top_centre = outer_band(pts, c, cfg.product_size * k)
        (_, _), (rw, rh), ang = cv2.minAreaRect(top_pts.astype(np.float32))
        yaw = math.remainder(math.radians(ang), math.pi / 2)
        fill = cv2.contourArea(top_pts.astype(np.float32)) / max(rw * rh, 1e-12)
        shape = 'cube' if fill > cfg.cube_fill else 'cylinder'
        # the top face is magnified away from the nadir by k: scale it back
        true_centre = c + (np.asarray(top_centre) - c) / k

        blobs.append(Blob(color, shape, defective, reason, float(true_centre[0]),
                          float(true_centre[1]), float(yaw), float(max(rw, rh) / k),
                          float(conf), cnt))
    return blobs


def top_face(lightness, solid, frac):
    """Contour (pixels) of the top face inside a product's colour mask.

    The top face is lit straight from above and renders lighter than the side
    faces the camera sees at an angle. Lab lightness separates them well even
    for bright colours whose HSV value saturates on both (yellow), so keep the
    pixels within `frac` of the blob's lightest. Returns None if too little is left.
    """
    vals = lightness[solid > 0]
    if vals.size < 50:
        return None
    lmax = np.percentile(vals, 95)
    bright = ((lightness >= lmax * (1.0 - frac)) & (solid > 0)).astype(np.uint8) * 255
    bright = cv2.morphologyEx(bright, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    bright = cv2.morphologyEx(bright, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    cs, _ = cv2.findContours(bright, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not cs:
        return None
    best = max(cs, key=cv2.contourArea)
    if cv2.contourArea(best) < 0.3 * vals.size:
        return None
    return best


def outer_band(pts, c, depth):
    """Fallback top-face estimate: the blob's outer band along the radial direction."""
    centroid = pts.mean(axis=0)
    radial = centroid - c
    dist = np.linalg.norm(radial)
    u = radial / dist if dist > 1e-4 else np.array([1.0, 0.0])
    s = (pts - c) @ u
    band = pts[s > s.max() - depth * 0.9]
    return band, band.mean(axis=0)


def annotate(bgr, blobs, cfg: VisionConfig, pick_xy=None):
    """Debug image: contours, labels and the true centres."""
    out = bgr.copy()
    if pick_xy is not None:
        p = to_pixels(cfg.homography, [pick_xy])[0].astype(int)
        cv2.drawMarker(out, tuple(int(v) for v in p), (255, 255, 255), cv2.MARKER_CROSS, 40, 2)
    for b in blobs:
        col = (0, 0, 255) if b.defective else (0, 255, 0)
        cv2.drawContours(out, [b.contour_px], -1, col, 2)
        p = to_pixels(cfg.homography, [(b.x, b.y)])[0].astype(int)
        cv2.circle(out, tuple(int(v) for v in p), 6, col, -1)
        label = f'{b.color} {b.shape}{" DEFECT" if b.defective else ""}'
        pos = f'({b.x * 1000:.0f}, {b.y * 1000:.0f}) mm'
        x0, y0 = int(p[0]) + 12, int(p[1]) - 10
        for k, text in enumerate((label, pos)):
            cv2.putText(out, text, (x0, y0 + 26 * k), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4)
            cv2.putText(out, text, (x0, y0 + 26 * k), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)
    return out
