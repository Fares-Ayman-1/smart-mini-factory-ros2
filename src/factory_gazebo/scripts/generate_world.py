#!/usr/bin/env python3
"""Generate worlds/factory.sdf from the shared cell geometry.

Every position comes from factory_control/geometry.py (the same numbers the IK
and the nodes use), so the world cannot drift from the code. Re-run after
changing the geometry:

    python3 src/factory_gazebo/scripts/generate_world.py
"""

import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve()
sys.path.insert(0, str(HERE.parents[2] / 'factory_control'))
from factory_control import geometry as g  # noqa: E402

OUT = HERE.parents[1] / 'worlds' / 'factory.sdf'

# --- extra layout values from the hardware PDF ------------------------------
BASEPLATE = (1.000, 0.500, 0.018)
BELT_WIDTH = 0.100
BELT_THICK = 0.006
ROLLER_R = 0.015
RAIL_H = 0.040
RAIL_T = 0.003
LANE_WIDE = 0.100           # lane at the idler end
LANE_NARROW = 0.060         # lane after the funnel
FUNNEL_END_X = 0.160
RAIL_END_X = 0.600
IR_HEIGHT = 0.012           # beam height above the belt
HOOD_X = (0.300, 0.600)
HOOD_Y = (0.300, 0.460)
HOOD_TOP = 0.300
HOOD_WALL = 0.004
# DESIGN FINDING: the PDF puts a wall on the arm side of the hood (y = 300 mm)
# with a 140 x 140 mm cut-out at the pick zone. That wall is only 60 mm from the
# arm's base axis (y = 240 mm). Moving between the bins (210..330 deg) and the
# belt (90 deg), the base must swing through ~150..165 deg, and at working reach
# plus half the gripper width (~225 mm) the arm sweeps straight through that
# wall. The 65..335 deg base range rules out swinging the other way, and the
# hood top is too high to clear. In the first headless test the arm stalled
# against it. So the arm side of the hood is left open here (back and top only).
ARM_SIDE_WALL = False

MAT = {
    'black': ('0.04 0.04 0.045 1', '0.06 0.06 0.065 1', '0.02 0.02 0.02 1'),
    'belt': ('0.03 0.03 0.03 1', '0.05 0.05 0.05 1', '0.01 0.01 0.01 1'),
    'alu': ('0.55 0.56 0.58 1', '0.70 0.71 0.73 1', '0.3 0.3 0.3 1'),
    'white': ('0.85 0.85 0.85 1', '0.92 0.92 0.92 1', '0.05 0.05 0.05 1'),
    'acrylic': ('0.7 0.8 0.9 0.25', '0.7 0.8 0.9 0.25', '0.5 0.5 0.5 0.25'),
    'tray': ('0.30 0.32 0.36 1', '0.42 0.44 0.48 1', '0.1 0.1 0.1 1'),
    'reject': ('0.45 0.12 0.10 1', '0.60 0.16 0.14 1', '0.1 0.1 0.1 1'),
    'red': ('0.75 0.08 0.08 1', '0.85 0.10 0.10 1', '0.1 0.1 0.1 1'),
    'green': ('0.08 0.55 0.18 1', '0.10 0.65 0.20 1', '0.1 0.1 0.1 1'),
    'blue': ('0.08 0.20 0.70 1', '0.10 0.25 0.80 1', '0.1 0.1 0.1 1'),
    'yellow': ('0.80 0.70 0.08 1', '0.90 0.80 0.10 1', '0.1 0.1 0.1 1'),
    'grey': ('0.45 0.45 0.45 1', '0.55 0.55 0.55 1', '0.1 0.1 0.1 1'),
}


def material(name):
    a, d, s = MAT[name]
    return f'<material><ambient>{a}</ambient><diffuse>{d}</diffuse><specular>{s}</specular></material>'


def pose(x=0, y=0, z=0, r=0, p=0, yw=0):
    return f'<pose>{x:.5f} {y:.5f} {z:.5f} {r:.5f} {p:.5f} {yw:.5f}</pose>'


def box(name, size, at, mat, collide=True, surface=''):
    sx, sy, sz = size
    geom = f'<geometry><box><size>{sx:.5f} {sy:.5f} {sz:.5f}</size></box></geometry>'
    out = f'<visual name="{name}_v">{pose(*at)}{geom}{material(mat)}</visual>'
    if collide:
        out += f'<collision name="{name}_c">{pose(*at)}{geom}{surface}</collision>'
    return out


def cylinder(name, r, length, at, mat, collide=True):
    geom = f'<geometry><cylinder><radius>{r:.5f}</radius><length>{length:.5f}</length></cylinder></geometry>'
    out = f'<visual name="{name}_v">{pose(*at)}{geom}{material(mat)}</visual>'
    if collide:
        out += f'<collision name="{name}_c">{pose(*at)}{geom}</collision>'
    return out


def static_model(name, body, at=(0, 0, 0)):
    return (f'<model name="{name}"><static>true</static>{pose(*at)}'
            f'<link name="link">{body}</link></model>')


def baseplate():
    sx, sy, sz = BASEPLATE
    return static_model('baseplate', box('plate', BASEPLATE, (sx / 2, sy / 2, -sz / 2), 'black'))


def conveyor():
    # Same friction recipe as gz-sim's conveyor.sdf example: low mu across the
    # belt, very high mu along it, so TrackController's surface speed carries
    # the products.
    surface = ('<surface><friction><ode><mu>0.7</mu><mu2>150</mu2>'
               '<fdir1>0 1 0</fdir1></ode></friction></surface>')
    length = g.BELT_X_END - g.BELT_X_START
    cx = (g.BELT_X_START + g.BELT_X_END) / 2
    zc = g.BELT_TOP_Z - BELT_THICK / 2
    belt = box('belt', (length, BELT_WIDTH, BELT_THICK), (0, 0, 0), 'belt', surface=surface)
    for i, x in enumerate((g.BELT_X_START, g.BELT_X_END)):
        belt += cylinder(f'roller{i}', ROLLER_R, BELT_WIDTH + 0.004,
                         (x - cx, 0, -ROLLER_R + BELT_THICK / 2, math.pi / 2, 0, 0), 'belt')
    # The track controller must act on a non-static model with a single link; the
    # link is welded to the world with a fixed joint.
    return f'''
<model name="conveyor">
  {pose(cx, g.BELT_CENTRE_Y, zc)}
  <link name="belt">
    <inertial><mass>2.0</mass><inertia><ixx>0.002</ixx><iyy>0.06</iyy><izz>0.06</izz>
      <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
    {belt}
  </link>
  <joint name="weld" type="fixed"><parent>world</parent><child>belt</child></joint>
  <plugin filename="gz-sim-track-controller-system" name="gz::sim::systems::TrackController">
    <link>belt</link>
    <odometry_publish_frequency>1</odometry_publish_frequency>
  </plugin>
</model>'''


def frame_and_rails():
    body = ''
    # 2020 extrusion side frames under the belt
    length = g.BELT_X_END - g.BELT_X_START + 0.06
    cx = (g.BELT_X_START + g.BELT_X_END) / 2
    for i, dy in enumerate((-0.065, 0.065)):
        body += box(f'frame{i}', (length, 0.020, 0.020),
                    (cx, g.BELT_CENTRE_Y + dy, 0.030), 'alu')
        for j, x in enumerate((g.BELT_X_START, g.BELT_X_END)):
            body += box(f'leg{i}{j}', (0.020, 0.020, 0.020),
                        (x, g.BELT_CENTRE_Y + dy, 0.010), 'alu')
    # acrylic side rails: funnel 100 -> 60 mm over x = 60..160, straight to x = 600
    zc = g.BELT_TOP_Z + RAIL_H / 2 + 0.001
    for i, side in enumerate((-1, 1)):
        y0 = g.BELT_CENTRE_Y + side * (LANE_WIDE / 2 + RAIL_T / 2)
        y1 = g.BELT_CENTRE_Y + side * (LANE_NARROW / 2 + RAIL_T / 2)
        dx, dy = FUNNEL_END_X - g.BELT_X_START, y1 - y0
        flen = math.hypot(dx, dy)
        body += box(f'funnel{i}', (flen, RAIL_T, RAIL_H),
                    ((g.BELT_X_START + FUNNEL_END_X) / 2, (y0 + y1) / 2, zc, 0, 0,
                     math.atan2(dy, dx)), 'acrylic')
        slen = RAIL_END_X - FUNNEL_END_X
        body += box(f'rail{i}', (slen, RAIL_T, RAIL_H),
                    ((FUNNEL_END_X + RAIL_END_X) / 2, y1, zc), 'acrylic')
    return static_model('frame_and_rails', body)


def hood():
    x0, x1 = HOOD_X
    y0, y1 = HOOD_Y
    zb, zt = g.BELT_TOP_Z, HOOD_TOP
    w = HOOD_WALL
    body = ''
    # top plate
    body += box('top', (x1 - x0, y1 - y0, w), ((x0 + x1) / 2, (y0 + y1) / 2, zt + w / 2), 'white')
    # back wall (y1), full
    body += box('back', (x1 - x0, w, zt - zb), ((x0 + x1) / 2, y1 + w / 2, (zb + zt) / 2), 'white')
    # Arm side left open (see ARM_SIDE_WALL above). One full-height post at the
    # upstream corner carries the top plate; it is 267 mm from the arm axis,
    # outside the swept circle.
    yw = y0 - w / 2
    body += box('front_post', (0.015, 0.015, zt - zb), (x0, yw, (zb + zt) / 2), 'alu')
    # wall supports down to the baseplate (the hood is mounted to the baseplate,
    # not the belt frame, per the vibration note)
    for i, (x, y) in enumerate(((x0, y0), (x0, y1), (x1, y0), (x1, y1))):
        body += box(f'post{i}', (0.015, 0.015, zb), (x, y, zb / 2), 'alu')
    # colour reference tile: five 12 mm squares on a bracket in the view corner,
    # outside the belt lane so vision ignores it for products
    for i, c in enumerate(('red', 'green', 'blue', 'yellow', 'grey')):
        body += box(f'ref_{c}', (0.012, 0.012, 0.002),
                    (x0 + 0.020 + i * 0.014, y0 + 0.012, g.BELT_TOP_Z + 0.001), c, collide=False)
    body += box('ref_bracket', (0.080, 0.022, g.BELT_TOP_Z),
                (x0 + 0.048, y0 + 0.012, g.BELT_TOP_Z / 2 - 0.0005), 'black')
    return static_model('hood', body)


def camera():
    x, y, z = g.CAMERA_XYZ
    return f'''
<model name="camera">
  <static>true</static>
  {pose(x, y, z, 0, math.pi / 2, math.pi / 2)}
  <link name="link">
    {box('body', (0.03, 0.09, 0.03), (-0.015, 0, 0), 'black', collide=False)}
    <sensor name="camera" type="camera">
      <always_on>true</always_on>
      <update_rate>10</update_rate>
      <topic>camera/image</topic>
      <camera>
        <horizontal_fov>{g.CAMERA_HFOV:.6f}</horizontal_fov>
        <image><width>{g.CAMERA_WIDTH}</width><height>{g.CAMERA_HEIGHT}</height><format>R8G8B8</format></image>
        <clip><near>0.05</near><far>1.0</far></clip>
        <camera_info_topic>camera/camera_info</camera_info_topic>
      </camera>
    </sensor>
  </link>
</model>'''


def ir_beams():
    """Through-beam IR pairs, modelled as single-ray lidars across the lane."""
    out = ''
    y_emit = g.BELT_CENTRE_Y - LANE_NARROW / 2 + 0.002   # just inside the near rail
    max_range = LANE_NARROW - 0.006                       # stops short of the far rail
    for name, x in (('ir1', g.IR1_X), ('ir2', g.IR2_X)):
        if name == 'ir1':
            # IR1 sits in the funnel, where the lane is wider than 60 mm
            frac = (x - g.BELT_X_START) / (FUNNEL_END_X - g.BELT_X_START)
            half = (LANE_WIDE + (LANE_NARROW - LANE_WIDE) * frac) / 2
            y = g.BELT_CENTRE_Y - half + 0.002
            rng = 2 * half - 0.006
        else:
            y, rng = y_emit, max_range
        out += f'''
<model name="{name}">
  <static>true</static>
  {pose(x, y, g.BELT_TOP_Z + IR_HEIGHT, 0, 0, math.pi / 2)}
  <link name="link">
    {box('emitter', (0.006, 0.004, 0.006), (-0.003, 0, 0), 'grey', collide=False)}
    <sensor name="{name}" type="gpu_lidar">
      <always_on>true</always_on>
      <update_rate>30</update_rate>
      <topic>{name}/scan</topic>
      <lidar>
        <scan><horizontal><samples>1</samples><resolution>1</resolution>
          <min_angle>0</min_angle><max_angle>0</max_angle></horizontal></scan>
        <range><min>0.002</min><max>{rng:.4f}</max><resolution>0.001</resolution></range>
      </lidar>
    </sensor>
  </link>
</model>'''
    return out


def bins():
    out = ''
    n_seg = 16
    r = g.BIN_DIAMETER / 2
    seg_len = 2 * r * math.tan(math.pi / n_seg) + 0.001
    for name in sorted(g.BIN_ANGLES_DEG):
        x, y = g.bin_xy(name)
        mat = 'reject' if name == g.REJECT_BIN else 'tray'
        body = cylinder('floor', r, 0.004, (0, 0, 0.002), mat)
        for k in range(n_seg):
            a = 2 * math.pi * k / n_seg
            body += box(f'wall{k}', (seg_len, 0.003, g.BIN_HEIGHT),
                        (r * math.cos(a), r * math.sin(a), g.BIN_HEIGHT / 2, 0, 0, a + math.pi / 2), mat)
        out += static_model(f'bin_{name}', body, (x, y, 0))
    return out


def lights():
    out = '''
<light type="directional" name="room">
  <cast_shadows>false</cast_shadows>
  <pose>0 0 3 0 0 0</pose>
  <diffuse>0.35 0.35 0.35 1</diffuse>
  <specular>0.1 0.1 0.1 1</specular>
  <direction>-0.3 0.2 -0.9</direction>
</light>'''
    # two LED strips along both long hood edges, behind a diffuser
    x0, x1 = HOOD_X
    for i, y in enumerate((HOOD_Y[0] + 0.012, HOOD_Y[1] - 0.012)):
        for j, x in enumerate((x0 + 0.06, (x0 + x1) / 2, x1 - 0.06)):
            out += f'''
<light type="point" name="hood_led_{i}{j}">
  <cast_shadows>false</cast_shadows>
  {pose(x, y, HOOD_TOP - 0.02)}
  <diffuse>0.55 0.55 0.53 1</diffuse>
  <specular>0.05 0.05 0.05 1</specular>
  <attenuation><range>0.6</range><constant>0.4</constant><linear>1.5</linear><quadratic>2.0</quadratic></attenuation>
</light>'''
    return out


def world():
    return f'''<?xml version="1.0" ?>
<!-- GENERATED by scripts/generate_world.py from factory_control/geometry.py. Do not edit by hand. -->
<sdf version="1.9">
<world name="factory">
  <physics name="2ms" type="ignored">
    <max_step_size>0.002</max_step_size>
    <real_time_factor>1.0</real_time_factor>
  </physics>
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
  <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
  <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
  <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
    <render_engine>ogre2</render_engine>
  </plugin>
  <scene>
    <ambient>0.45 0.45 0.45 1</ambient>
    <background>0.75 0.77 0.80 1</background>
    <grid>false</grid>
  </scene>
  <gui fullscreen="0">
    <camera name="user_camera"><pose>1.25 -0.55 0.85 0 0.55 2.45</pose></camera>
  </gui>
  {lights()}
  <model name="ground"><static>true</static>
    <link name="link">
      <collision name="c"><pose>0 0 -0.0185 0 0 0</pose><geometry><plane><normal>0 0 1</normal><size>4 4</size></plane></geometry></collision>
      <visual name="v"><pose>0 0 -0.0185 0 0 0</pose><geometry><plane><normal>0 0 1</normal><size>4 4</size></plane></geometry>
        <material><ambient>0.5 0.5 0.5 1</ambient><diffuse>0.6 0.6 0.6 1</diffuse></material></visual>
    </link>
  </model>
  {baseplate()}
  {conveyor()}
  {frame_and_rails()}
  {hood()}
  {camera()}
  {ir_beams()}
  {bins()}
</world>
</sdf>
'''


if __name__ == '__main__':
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(world())
    print(f'wrote {OUT}')
