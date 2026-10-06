#!/usr/bin/env python3
"""Exercise the obstacle node with synthetic sensors. No simulator needed.

    python3 test/histogram_selftest.py

The only script in this directory that runs without PX4 and Gazebo: it
builds the node, hands it point clouds and laser scans it makes up, and
checks the histogram bin by bin. It is the test to run after touching
obstacle_distance.py, before the gate, because it says which bin went wrong
rather than that the aircraft hit a wall. Exit code is the number of
failures.

What it pins down:
  1. The single camera, as the measured stack runs it: 15 observed bins, a
     wall 3 m ahead lands in bin 0 at the camera's mount offset.
  2. Two sensors merge by nearest range, and the observed arc is their union.
  3. A sensor that goes stale leaves the union; a sensor sending all-NaN
     is dead and leaves it at once.
  4. A yawed sensor's readings land where the yaw says, a scan's angles
     map counter-clockwise to clockwise bearings, and a sensor behind the
     aircraft fills the rear bins.
"""
import math
import os
import sys

import numpy as np
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Header

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from avoidance_sim.obstacle_distance import (BINS, UINT16_MAX,     # noqa: E402
                                             ObstacleDistancePublisher)

FAILS = 0


def check(label, cond, detail=''):
    global FAILS
    print('  %-62s %s %s' % (label, 'ok' if cond else 'FAIL', detail))
    if not cond:
        FAILS += 1


def cloud(points_flu):
    """A PointCloud2 of (x, y, z) in FLU, NaN allowed."""
    pts = np.asarray(points_flu, dtype=np.float32).reshape(-1, 3)
    m = PointCloud2()
    m.header = Header(frame_id='camera_link')
    m.height, m.width = 1, pts.shape[0]
    m.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                for i, n in enumerate('xyz')]
    m.is_bigendian, m.point_step, m.row_step = False, 12, 12 * pts.shape[0]
    m.is_dense = False
    m.data = pts.tobytes()
    return m


def scan(ranges, amin=-math.pi, amax=math.pi, rmin=0.3, rmax=30.0):
    m = LaserScan()
    m.header = Header(frame_id='lidar_link')
    n = len(ranges)
    m.angle_min, m.angle_max = float(amin), float(amax)
    m.angle_increment = float((amax - amin) / (n - 1)) if n > 1 else 0.0
    m.range_min, m.range_max = float(rmin), float(rmax)
    m.ranges = [float(r) for r in ranges]
    return m


def node(**params):
    params.setdefault('diagnostics', False)      # the extents log is noise here
    overrides = [Parameter(k, value=v) for k, v in params.items()]
    return ObstacleDistancePublisher(parameter_overrides=overrides)


def bins_at(hist, deg):
    """The bin index for a clockwise bearing in degrees."""
    return int(round((deg % 360.0) / 5.0)) % BINS


def main():
    rclpy.init()

    print('1. single camera, legacy parameters, the stack as measured')
    n = node()
    t = n.now_s()
    check('no sensor yet -> nothing to publish', n.histogram(t) is None)
    n.on_cloud(n.sources[0], cloud([[3.0, 0.0, 0.0], [3.0, 0.05, 0.0]]))
    out, mn, mx, alive = n.histogram(n.now_s())
    obs = int((out != UINT16_MAX).sum())
    check('15 observed bins (73 deg / 5)', obs == 15, f'got {obs}')
    # The camera sits 0.12 m ahead of base_link, so a wall 3.00 m from the
    # camera is 3.12 m from the body, which is what PX4 is told.
    check('wall 3 m ahead -> bin 0 at 312 cm (mount offset included)',
          310 <= out[0] <= 314, f'got {out[0]}')
    check('bin 35 (behind-left edge) UNKNOWN', out[36] == UINT16_MAX)
    check('bin 5 (25 deg right) observed-clear = max+1', out[5] == mx + 1, f'got {out[5]}, max {mx}')
    check('min/max are the camera\'s', (mn, mx) == (20, 1910), f'got {(mn, mx)}')
    n.on_cloud(n.sources[0], cloud([[float('nan')] * 3] * 50))
    check('all-NaN cloud -> dead -> nothing published', n.histogram(n.now_s()) is None)
    n.destroy_node()

    print('2. camera plus a 360 degree lidar on the tail')
    n = node(**{'sources': 'camera lidar', 'lidar.type': 'scan', 'lidar.topic': '/lidar',
                'lidar.mount_xyz_frd': [-0.10, 0.0, -0.30],
                'lidar.min_distance_cm': 30, 'lidar.max_distance_cm': 3000})
    cam, lid = n.sources
    # 360 rays, one per degree, CCW from -180: index i is angle -180 + i.
    r = [float('inf')] * 360
    r[180 + 90] = 4.0      # +90 deg CCW = sensor's LEFT  -> bearing -90 = bin 54
    r[180 + 0] = 5.0       # straight ahead of the lidar  -> bin 0, 5.0 - 0.10 = 4.90 m
    r[0] = 6.0             # -180 = behind                -> bin 36, 6.0 + 0.10 = 6.10 m
    n.on_scan(lid, scan(r))
    n.on_cloud(cam, cloud([[3.0, 0.0, 0.0]]))
    out, mn, mx, alive = n.histogram(n.now_s())
    check('both alive', len(alive) == 2)
    check('observed arc is the union: all 72 bins', int((out != UINT16_MAX).sum()) == 72)
    check('bin 0 takes the nearer of 312 (camera) and 490 (lidar)', 310 <= out[0] <= 314, f'got {out[0]}')
    check('left wall from the scan -> bin 54 at 400 cm', 398 <= out[54] <= 402, f'got {out[54]}')
    check('rear wall from the scan -> bin 36 at 610 cm', 608 <= out[36] <= 612, f'got {out[36]}')
    check('max is the longer sensor, clear bins = 3001', out[18] == 3001 and mx == 3000, f'got {out[18]}, max {mx}')
    check('min is the shorter sensor', mn == 20, f'got {mn}')
    # Camera stale: the lidar alone. Bin 0 becomes the lidar's 490.
    cam.last_t -= 5.0
    out, mn, mx, alive = n.histogram(n.now_s())
    check('camera stale -> lidar only, bin 0 = 490', len(alive) == 1 and 488 <= out[0] <= 492, f'got {out[0]}')
    # Lidar dead too: nothing.
    n.on_scan(lid, scan([float('nan')] * 360))
    check('lidar all-NaN as well -> nothing published', n.histogram(n.now_s()) is None)
    n.destroy_node()

    print('3. a second camera facing backwards (yaw 180)')
    n = node(**{'sources': 'camera rear', 'rear.type': 'cloud', 'rear.topic': '/rear/points',
                'rear.yaw_deg': 180.0, 'rear.mount_xyz_frd': [-0.12, 0.0, -0.242]})
    cam, rear = n.sources
    check('rear camera observes bins around 180', rear.observed[36] and rear.observed[29]
          and rear.observed[43] and not rear.observed[0])
    # A wall 2 m in front of the REAR camera is 2.12 m behind the body.
    n.on_cloud(rear, cloud([[2.0, 0.0, 0.0]]))
    n.on_cloud(cam, cloud([[float('inf')] * 3]))        # front camera: valid, sees nothing
    out, mn, mx, alive = n.histogram(n.now_s())
    check('rear wall -> bin 36 at 212 cm', 210 <= out[36] <= 214, f'got {out[36]}')
    check('front bin 0 observed-clear', out[0] == mx + 1)
    check('30 observed bins (two 73 deg arcs)', int((out != UINT16_MAX).sum()) == 30,
          f'got {int((out != UINT16_MAX).sum())}')
    # A wall to the rear camera's LEFT (its +y in FLU) is on the body's RIGHT.
    # Far enough out that the camera's 0.12 m mount offset does not shift the
    # bearing: at 2 m it would be 93 degrees and bin 19, which is correct and
    # was the first version of this check's mistake.
    n.on_cloud(rear, cloud([[0.0, 10.0, 0.0]]))
    out, *_ = n.histogram(n.now_s())
    check('rear camera\'s left is the body\'s right: bin 18 (90 deg clockwise)',
          998 <= out[18] <= 1002, f'got {out[18]}')
    n.destroy_node()

    print('4. a 270 degree scan yawed +90 (facing right)')
    n = node(**{'sources': 'side', 'side.type': 'scan', 'side.topic': '/side',
                'side.yaw_deg': 90.0, 'side.min_distance_cm': 30, 'side.max_distance_cm': 3000})
    side = n.sources[0]
    r = [float('inf')] * 271                     # one ray per degree, -135..+135
    r[135] = 3.0                                 # straight ahead of the sensor = body right
    n.on_scan(side, scan(r, amin=math.radians(-135), amax=math.radians(135)))
    out, mn, mx, alive = n.histogram(n.now_s())
    check('sensor forward -> bin 18 (body right) at 300 cm', 298 <= out[18] <= 302, f'got {out[18]}')
    check('body left (bin 54) is outside a 270 deg arc facing right', out[54] == UINT16_MAX)
    check('body forward (bin 0) is inside it, clear', out[0] == mx + 1)
    n.destroy_node()

    rclpy.try_shutdown()
    print()
    print('  SELFTEST: %s' % ('PASS' if FAILS == 0 else '%d FAILED' % FAILS))
    return FAILS


if __name__ == '__main__':
    raise SystemExit(main())
