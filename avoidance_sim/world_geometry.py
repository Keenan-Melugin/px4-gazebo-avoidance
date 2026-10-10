"""Static world geometry from a Gazebo world file, for the nodes and the tests.

Every obstacle in the bundled worlds is an axis-aligned box: a static model
with a <pose> and a <box><size>. The parser reads exactly that and nothing
more, which is why it is a regular expression rather than an SDF library:
the measurement scripts must run on a machine with the workspace sourced
and nothing else installed.

Two questions are answered here, because they are the two the measurement
scripts ask. Which box faces lie ahead of a point along a cardinal
direction, and where the first one is. The walls world's numbers come out
of these functions (box1's west face at east +4.5 for the standoff test,
box2's south face at north +4.5 for the planning test); the scripts used to
carry them as constants, and a different world silently broke them.

Frames. Gazebo's world frame is ENU and shares PX4's local origin, measured
in world_markers.py, so world x is east and world y is north, and the
numbers here compare directly with PX4's local position (north, east, down).

    python3 -m avoidance_sim.world_geometry walls
    python3 -m avoidance_sim.world_geometry pillars --from 0 0 --alt 7 --dir east
"""

import argparse
import math
import os
import re
from collections import namedtuple


PX4_ROOT = os.path.expanduser('~/PX4-Autopilot')
PX4_WORLDS = os.path.join(PX4_ROOT, 'Tools', 'simulation', 'gz', 'worlds')
# The repository's own worlds, next to this package in a source checkout and
# in share/avoidance_sim/worlds once installed. install.sh links them into
# PX4's worlds directory too, because PX4 builds the world path itself from
# PX4_GZ_WORLDS, which its generated gz_env.sh overwrites (see docs/extend.md, "Where PX4 looks").
HERE = os.path.dirname(os.path.abspath(__file__))
REPO_WORLDS = (
    os.path.normpath(os.path.join(HERE, '..', 'worlds')),
    os.path.normpath(os.path.join(HERE, '..', '..', '..', 'share',
                                  'avoidance_sim', 'worlds')),
)

DIRECTIONS = ('east', 'west', 'north', 'south')


class Box(namedtuple('Box', 'name cx cy cz sx sy sz yaw')):
    """An axis-aligned box: centre, size and the yaw the file gave it."""
    __slots__ = ()

    @property
    def x_min(self): return self.cx - self.sx / 2.0
    @property
    def x_max(self): return self.cx + self.sx / 2.0
    @property
    def y_min(self): return self.cy - self.sy / 2.0
    @property
    def y_max(self): return self.cy + self.sy / 2.0
    @property
    def z_min(self): return self.cz - self.sz / 2.0
    @property
    def z_max(self): return self.cz + self.sz / 2.0

    @property
    def rotated(self):
        """True if the box is yawed, which the face queries cannot handle."""
        return abs(self.yaw) > 1e-6

    def describe(self):
        s = (f'{self.name}: east {self.x_min:+.1f}..{self.x_max:+.1f}, '
             f'north {self.y_min:+.1f}..{self.y_max:+.1f}, '
             f'up {self.z_min:.1f}..{self.z_max:.1f} m')
        if self.rotated:
            s += f' (yawed {math.degrees(self.yaw):.0f} deg: drawn, not queried)'
        return s


def resolve_world(spec, px4_worlds=PX4_WORLDS, extra_dirs=REPO_WORLDS):
    """A world name or a path -> the path of its .sdf, or FileNotFoundError.

    A name is looked up where PX4 looks, so that `PX4_GZ_WORLD=name` for PX4
    and `name` here always mean the same file, and then in this repository's
    own worlds directory for a world not linked in yet.
    """
    spec = os.path.expanduser(str(spec))
    if os.sep in spec or spec.endswith('.sdf'):
        if os.path.isfile(spec):
            return os.path.abspath(spec)
        raise FileNotFoundError(f'world file not found: {spec}')
    tried = []
    for d in (px4_worlds,) + tuple(extra_dirs):
        p = os.path.join(d, spec + '.sdf')
        tried.append(p)
        if os.path.isfile(p):
            return p
    raise FileNotFoundError(
        f'no world named {spec!r}. Looked for:\n  ' + '\n  '.join(tried)
        + '\nA world in this repository is linked into the PX4 directory by '
          'scripts/link_assets.sh.')


def world_name(path):
    """The <world name="..."> inside the file, which is what PX4 calls it."""
    m = re.search(r'<world\s+name=[\'"]([^\'"]+)[\'"]', open(path).read())
    return m.group(1) if m else os.path.splitext(os.path.basename(path))[0]


def load_boxes(path, skip=('ground_plane',)):
    """Every static box model in the file, in file order."""
    text = open(path).read()
    boxes = []
    for m in re.finditer(r'<model name=[\'"]([^\'"]+)[\'"]>(.*?)</model>',
                         text, re.S):
        name, body = m.group(1), m.group(2)
        if name in skip:
            continue
        # The model's own pose is the one before its first <link>; a pose
        # inside the link would offset the geometry within the model, which
        # none of the bundled worlds do. Fall back to the first pose at all.
        head = body.split('<link', 1)[0]
        pose = re.search(r'<pose[^>]*>([^<]+)</pose>', head) or \
            re.search(r'<pose[^>]*>([^<]+)</pose>', body)
        box = re.search(r'<box>\s*<size>([^<]+)</size>', body, re.S)
        if not (pose and box):
            continue
        p = [float(v) for v in pose.group(1).split()]
        px, py, pz = p[:3]
        yaw = p[5] if len(p) >= 6 else 0.0
        sx, sy, sz = [float(v) for v in box.group(1).split()[:3]]
        boxes.append(Box(name, px, py, pz, sx, sy, sz, yaw))
    return boxes


def faces_ahead(boxes, east, north, alt, direction, margin=0.0):
    """Box faces ahead of (east, north, alt) along a cardinal direction.

    Returns [(face_coordinate, box), ...] nearest first. A box counts if
    its span across the direction of travel contains the point's cross
    coordinate (widened by `margin`), its height contains `alt`, and the
    point is not already inside it. Yawed boxes are skipped.
    """
    if direction not in DIRECTIONS:
        raise ValueError(f'direction must be one of {DIRECTIONS}')
    out = []
    for b in boxes:
        if b.rotated or not (b.z_min <= alt <= b.z_max):
            continue
        if direction in ('east', 'west'):
            if not (b.y_min - margin <= north <= b.y_max + margin):
                continue
            if direction == 'east' and b.x_min > east:
                out.append((b.x_min, b))
            elif direction == 'west' and b.x_max < east:
                out.append((b.x_max, b))
        else:
            if not (b.x_min - margin <= east <= b.x_max + margin):
                continue
            if direction == 'north' and b.y_min > north:
                out.append((b.y_min, b))
            elif direction == 'south' and b.y_max < north:
                out.append((b.y_max, b))
    along = east if direction in ('east', 'west') else north
    out.sort(key=lambda fb: abs(fb[0] - along))
    return out


def first_face_ahead(boxes, east, north, alt, direction, margin=0.0):
    """The nearest (face_coordinate, box) ahead, or None."""
    faces = faces_ahead(boxes, east, north, alt, direction, margin)
    return faces[0] if faces else None


def clearance(boxes, east, north):
    """Horizontal distance from (east, north) to the nearest box, metres."""
    best = float('inf')
    for b in boxes:
        dx = max(b.x_min - east, 0.0, east - b.x_max)
        dy = max(b.y_min - north, 0.0, north - b.y_max)
        best = min(best, (dx * dx + dy * dy) ** 0.5)
    return best


def clear_point(boxes, min_clear, near=(0.0, 0.0), extent=40.0, step=2.0):
    """The point nearest `near` with no box within `min_clear` metres.

    For tests that need open air, such as flying a known direction without
    collision prevention deflecting the aircraft. None if nothing qualifies.
    """
    best, best_d = None, float('inf')
    n = int(extent / step)
    for i in range(-n, n + 1):
        for j in range(-n, n + 1):
            e, no = near[0] + i * step, near[1] + j * step
            if clearance(boxes, e, no) >= min_clear:
                d = (e - near[0]) ** 2 + (no - near[1]) ** 2
                if d < best_d:
                    best, best_d = (e, no), d
    return best


def main(argv=None):
    ap = argparse.ArgumentParser(
        description='Print what a world looks like to the measurement scripts.')
    ap.add_argument('world', help='world name (as PX4_GZ_WORLD) or path to .sdf')
    ap.add_argument('--from', dest='origin', nargs=2, type=float,
                    metavar=('EAST', 'NORTH'), default=None,
                    help='where the aircraft is, to list the faces ahead of it')
    ap.add_argument('--alt', type=float, default=7.0, help='altitude, m')
    ap.add_argument('--dir', default='east', choices=DIRECTIONS)
    a = ap.parse_args(argv)
    path = resolve_world(a.world)
    boxes = load_boxes(path)
    print(f'{path}\n  world name: {world_name(path)}\n  {len(boxes)} boxes')
    for b in boxes:
        print('  ' + b.describe())
    if a.origin:
        east, north = a.origin
        faces = faces_ahead(boxes, east, north, a.alt, a.dir)
        print(f'  faces {a.dir} of east {east:+.1f} north {north:+.1f} '
              f'at {a.alt:.1f} m:')
        if not faces:
            print('    none')
        for coord, b in faces:
            along = east if a.dir in ('east', 'west') else north
            print(f'    {b.name} face at {coord:+.1f}, {abs(coord - along):.1f} m away')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
