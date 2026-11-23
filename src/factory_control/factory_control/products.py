"""SDF for the test products: 30 mm cubes and cylinders in four colours.

Defective parts carry a dark mark on the top face (what the downward camera
sees): a 10 mm "chip" or a thin 22 mm "crack".
"""

import random
from dataclasses import dataclass

from factory_control.geometry import PRODUCT_SIZE

COLORS = {
    # diffuse RGB, matched to the colour reference tile in the world
    'red': (0.85, 0.10, 0.10),
    'green': (0.10, 0.65, 0.20),
    'blue': (0.10, 0.25, 0.80),
    'yellow': (0.90, 0.80, 0.10),
}
SHAPES = ('cube', 'cylinder')
DEFECTS = ('chip', 'crack')
MASS = 0.030


@dataclass(frozen=True)
class ProductSpec:
    name: str
    color: str
    shape: str
    defect: str = ''          # '' | 'chip' | 'crack'

    @property
    def defective(self) -> bool:
        return bool(self.defect)


def random_spec(name: str, defect_fraction: float, rng: random.Random) -> ProductSpec:
    defect = rng.choice(DEFECTS) if rng.random() < defect_fraction else ''
    return ProductSpec(name, rng.choice(sorted(COLORS)), rng.choice(SHAPES), defect)


def _material(rgb, dim=1.0):
    r, g, b = (c * dim for c in rgb)
    return (f'<material><ambient>{r:.3f} {g:.3f} {b:.3f} 1</ambient>'
            f'<diffuse>{r:.3f} {g:.3f} {b:.3f} 1</diffuse>'
            f'<specular>0.05 0.05 0.05 1</specular></material>')


def product_sdf(spec: ProductSpec) -> str:
    s = PRODUCT_SIZE
    if spec.shape == 'cube':
        geom = f'<box><size>{s} {s} {s}</size></box>'
        i = MASS * (s * s + s * s) / 12
        inertia = f'<ixx>{i:.3e}</ixx><iyy>{i:.3e}</iyy><izz>{i:.3e}</izz>'
    else:
        r = s / 2
        geom = f'<cylinder><radius>{r}</radius><length>{s}</length></cylinder>'
        ixx = MASS * (3 * r * r + s * s) / 12
        izz = MASS * r * r / 2
        inertia = f'<ixx>{ixx:.3e}</ixx><iyy>{ixx:.3e}</iyy><izz>{izz:.3e}</izz>'

    mark = ''
    top = s / 2 + 0.0003
    if spec.defect == 'chip':
        mark = (f'<visual name="chip"><pose>0.005 0.004 {top} 0 0 0.4</pose>'
                f'<geometry><box><size>0.010 0.010 0.0006</size></box></geometry>'
                f'{_material((0.05, 0.05, 0.05))}</visual>')
    elif spec.defect == 'crack':
        mark = (f'<visual name="crack"><pose>0 0 {top} 0 0 0.7</pose>'
                f'<geometry><box><size>0.022 0.0025 0.0006</size></box></geometry>'
                f'{_material((0.05, 0.05, 0.05))}</visual>')

    surface = ('<surface><friction><ode><mu>1.5</mu><mu2>1.5</mu2></ode></friction>'
               '<contact><ode><kp>100000</kp><kd>1</kd><min_depth>0.0005</min_depth>'
               '<max_vel>0.01</max_vel></ode></contact></surface>')
    return f'''<sdf version="1.9">
<model name="{spec.name}">
  <link name="body">
    <inertial><mass>{MASS}</mass><inertia>{inertia}<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
    <collision name="c"><geometry>{geom}</geometry>{surface}</collision>
    <visual name="v"><geometry>{geom}</geometry>{_material(COLORS[spec.color])}</visual>
    {mark}
  </link>
</model>
</sdf>'''
