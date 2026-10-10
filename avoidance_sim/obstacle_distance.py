#!/usr/bin/env python3
"""Turn depth clouds and laser scans into PX4's ObstacleDistance polar histogram.

Subscribes : one or more sensors, each a sensor_msgs/PointCloud2 or a
             sensor_msgs/LaserScan (default: /depth_camera/points)
Publishes  : /fmu/in/obstacle_distance   px4_msgs/ObstacleDistance

PX4's collision prevention consumes obstacle_distance and brakes. This node
does perception only; it never commands the vehicle.

Why the merging happens here and not in PX4. PX4 reads ONE obstacle_distance
stream and fuses it with any distance_sensor streams bin by bin, with the rule
in CollisionPrevention::_enterData (v1.17.0): of two sensors that see a bin,
the one with the shorter range wins. It has no rule for two obstacle_distance
publishers: each message overwrites the bins it covers, so two nodes on the
topic alternate and the histogram flickers between them. So the sensors are
merged before PX4 sees any of them: the nearest range per bin across the
sensors that are alive, the observed arc is the union of theirs, and a sensor
that goes quiet drops out of the union. Its bins then report UNKNOWN, and PX4
refuses to move into them: with CP_GO_NO_DATA 0 always, and with 1 too for
any bin it has observed before (CollisionPrevention keeps _data_fov set), so
a sensor dropping out blocks the directions it used to cover. PX4 itself treats a stream that stops as no data
after 0.5 s (RANGE_STREAM_TIMEOUT_US), which is what the all-NaN rule below
relies on.

Sources. The `sources` parameter is a space-separated list of names; each
name has its own parameters:
    <name>.type             'cloud' (PointCloud2) or 'scan' (LaserScan)
    <name>.topic
    <name>.frame            cloud only: 'flu' (Gazebo) or 'optical' (ROS drivers)
    <name>.hfov_deg         cloud only; a scan carries its own arc
    <name>.mount_xyz_frd    sensor position in body FRD, metres
    <name>.yaw_deg          sensor forward relative to body forward,
                            clockwise positive, like the histogram's bearings
    <name>.min_distance_cm, <name>.max_distance_cm
The launch loads such a file with sensors:=file.yaml (config/sensors_lidar.yaml
and config/sensors_example.yaml are the examples).
With `sources` empty the node is the single-camera node it was, built from
the legacy parameters (cloud_topic, cloud_frame, hfov_deg, mount_xyz_frd,
min/max_distance_cm), and a source named `camera` takes those as its
defaults too, so `sources: "camera lidar"` only needs the lidar described.

Defaults match the simulated OAK-D Lite in PX4 v1.17.0
(Tools/simulation/gz/models/OakD-Lite/model.sdf):
    horizontal_fov 1.274 rad = 73.0 deg, clip near 0.2 m, far 19.1 m, 30 Hz.

    python3 test/histogram_selftest.py    # exercises this file with no simulator
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2

from px4_msgs.msg import ObstacleDistance

BINS = 72                      # fixed by the message: uint16[72]
INCREMENT_DEG = 360.0 / BINS   # 5.0
UINT16_MAX = 65535             # "unknown", per the message definition
KINDS = ('cloud', 'scan')
FRAMES = ('flu', 'optical')


def observed_bins(centre_deg, half_deg):
    """Bins whose centre bearing lies within half_deg of centre_deg.

    Bearings are clockwise from body forward, as the message defines them.
    A 73 degree camera straight ahead is centre 0, half 36.5: 15 bins.
    """
    out = np.zeros(BINS, dtype=bool)
    for i in range(BINS):
        d = (i * INCREMENT_DEG - centre_deg + 180.0) % 360.0 - 180.0
        if abs(d) <= half_deg + 1e-9:
            out[i] = True
    return out


class Source:
    """One sensor: its geometry, and the latest thing it said."""

    def __init__(self, name, kind, topic, frame, hfov_deg, mount_xyz_frd,
                 yaw_deg, min_distance_cm, max_distance_cm):
        if kind not in KINDS:
            raise ValueError(f'{name}.type must be one of {KINDS}, not {kind!r}')
        if frame not in FRAMES:
            raise ValueError(f'{name}.frame must be one of {FRAMES}, not {frame!r}')
        self.name, self.kind, self.topic = name, kind, topic
        self.frame = 'flu' if kind == 'scan' else frame   # a scan is planar FLU by definition
        self.hfov = float(hfov_deg)
        self.mount = np.asarray(mount_xyz_frd, dtype=np.float64)
        self.yaw_deg = float(yaw_deg)
        self.min_cm = int(min_distance_cm)
        self.max_cm = int(max_distance_cm)
        # Which bins this sensor can see at all. A cloud's arc is a
        # parameter; a scan's comes with its first message.
        self.observed = (observed_bins(self.yaw_deg, self.hfov / 2.0)
                         if kind == 'cloud' else None)
        self.nearest = None        # per-bin nearest range in cm, inf = nothing
        self.last_t = None         # when the last message arrived, s
        self.valid = False         # the last message carried data at all
        self.nan_frames = 0
        self.logged = False
        self.warned_stale = False
        self.sub = None

    def describe(self):
        arc = ('arc from its first message' if self.observed is None
               else f'{int(self.observed.sum())} of {BINS} bins')
        return (f'{self.name}: {self.kind} on {self.topic}, {arc}, '
                f'yaw {self.yaw_deg:+.0f} deg, mount FRD '
                f'({self.mount[0]:+.2f},{self.mount[1]:+.2f},{self.mount[2]:+.2f}), '
                f'{self.min_cm / 100:.2f} to {self.max_cm / 100:.1f} m')


class ObstacleDistancePublisher(Node):

    def __init__(self, **kwargs):
        super().__init__('obstacle_distance_publisher', **kwargs)

        p = self.declare_parameter
        # The single-camera parameters, kept as they were. They are the
        # defaults of the source named 'camera'.
        p('cloud_topic', '/depth_camera/points')
        # 'flu'     = x forward, y left, z up   (what Gazebo actually publishes)
        # 'optical' = x right, y down, z forward (ROS depth-camera convention)
        # Measured on PX4 v1.17.0 gz_x500_depth: the bridged cloud is FLU.
        # frame_id is 'camera_link' and x held [+0.45,+4.37] facing a wall,
        # so x is forward. Do not assume; the node logs the extents to recheck.
        p('cloud_frame', 'flu')
        p('min_distance_cm', 20)       # SDF near clip 0.2 m
        p('max_distance_cm', 1910)     # SDF far clip 19.1 m
        p('hfov_deg', 73.0)            # SDF horizontal_fov 1.274 rad
        p('height_band_m', 1.0)        # keep cloud points within this of the vehicle plane
        p('mount_xyz_frd', [0.12, -0.03, -0.242])   # SDF pose, FLU -> FRD
        p('publish_hz', 10.0)
        # A 640x480 depth image is 307200 points. Processing all of them takes
        # ~400 ms here, which starves the executor and drops the publish rate
        # from 10 Hz to ~2 Hz. Every Nth point is plenty for a 72-bin histogram.
        p('decimate', 8)
        p('diagnostics', True)
        # A sensor older than this drops out of the histogram. With no sensor
        # left nothing is published and PX4's own stream timeout (0.5 s)
        # holds the aircraft instead of flying on a frozen histogram. 1.0 s
        # covers the slowest measured camera (2.2 Hz).
        p('stale_s', 1.0)
        # Space-separated source names; empty means the one camera above.
        p('sources', '')

        g = lambda n: self.get_parameter(n).value
        self.band = float(g('height_band_m'))
        self.diag = bool(g('diagnostics'))
        self.stale_s = float(g('stale_s'))
        self.decimate = max(1, int(g('decimate')))

        legacy = dict(kind='cloud', topic=g('cloud_topic'), frame=g('cloud_frame'),
                      hfov_deg=g('hfov_deg'), mount_xyz_frd=list(g('mount_xyz_frd')),
                      yaw_deg=0.0, min_distance_cm=g('min_distance_cm'),
                      max_distance_cm=g('max_distance_cm'))
        names = str(g('sources')).split()
        self.sources = []
        if not names:
            self.sources.append(Source('camera', **legacy))
        for name in names:
            d = dict(legacy) if name == 'camera' else dict(
                kind='cloud', topic=f'/{name}/points', frame='flu', hfov_deg=73.0,
                mount_xyz_frd=[0.0, 0.0, 0.0], yaw_deg=0.0,
                min_distance_cm=20, max_distance_cm=1910)
            v = lambda key, default: self.declare_parameter(f'{name}.{key}', default).value
            self.sources.append(Source(
                name,
                kind=v('type', d['kind']),
                topic=v('topic', d['topic']),
                frame=v('frame', d['frame']),
                hfov_deg=v('hfov_deg', float(d['hfov_deg'])),
                mount_xyz_frd=list(v('mount_xyz_frd', [float(x) for x in d['mount_xyz_frd']])),
                yaw_deg=v('yaw_deg', float(d['yaw_deg'])),
                min_distance_cm=v('min_distance_cm', int(d['min_distance_cm'])),
                max_distance_cm=v('max_distance_cm', int(d['max_distance_cm']))))

        for s in self.sources:
            self.get_logger().info(s.describe())
        union = np.zeros(BINS, dtype=bool)
        for s in self.sources:
            if s.observed is not None:
                union |= s.observed
        if any(s.observed is None for s in self.sources):
            self.get_logger().info(
                f'{int(union.sum())} of {BINS} bins observed before any scan arrives. '
                f'The rest report UNKNOWN, which PX4 treats as blocked unless '
                f'CP_GO_NO_DATA=1.')
        else:
            self.get_logger().info(
                f'sensors see {int(union.sum())} of {BINS} bins. The other '
                f'{BINS - int(union.sum())} report UNKNOWN, which PX4 treats '
                f'as blocked unless CP_GO_NO_DATA=1.')

        # PX4 publishes/subscribes best-effort; a reliable subscriber will not match.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5)

        self.pub = self.create_publisher(
            ObstacleDistance, '/fmu/in/obstacle_distance', qos)
        for s in self.sources:
            if s.kind == 'cloud':
                s.sub = self.create_subscription(
                    PointCloud2, s.topic, (lambda m, s=s: self.on_cloud(s, m)), qos)
            else:
                s.sub = self.create_subscription(
                    LaserScan, s.topic, (lambda m, s=s: self.on_scan(s, m)), qos)

        self.create_timer(1.0 / float(g('publish_hz')), self.on_timer)

    # ------------------------------------------------------------------
    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def sensor_to_frd(self, s, pts):
        """Points in the sensor's frame -> body FRD, through yaw and mount."""
        x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
        if s.frame == 'optical':      # x right, y down, z forward
            frd = np.column_stack((z, x, y))
        else:                         # flu: x forward, y left, z up
            frd = np.column_stack((x, -y, -z))
        if s.yaw_deg:
            # Positive yaw turns the sensor's forward toward the body's right,
            # which is clockwise seen from above, the histogram's sense.
            c, sn = math.cos(math.radians(s.yaw_deg)), math.sin(math.radians(s.yaw_deg))
            fx = frd[:, 0] * c - frd[:, 1] * sn
            fy = frd[:, 0] * sn + frd[:, 1] * c
            frd = np.column_stack((fx, fy, frd[:, 2]))
        return frd + s.mount

    def bin_frd(self, s, frd, band):
        """Nearest range per bin, in cm, inf where this sensor saw nothing."""
        if band:
            # Keep a horizontal slab around the vehicle. At low altitude the
            # ground enters this band and marks as an obstacle in every direction.
            frd = frd[np.abs(frd[:, 2]) <= self.band]
        if frd.shape[0] == 0:
            return None
        rng_cm = np.hypot(frd[:, 0], frd[:, 1]) * 100.0
        keep = (rng_cm >= s.min_cm) & (rng_cm <= s.max_cm)
        frd, rng_cm = frd[keep], rng_cm[keep]
        if frd.shape[0] == 0:
            return None
        # Bearing, clockwise-positive from forward, as the message requires.
        brg = np.degrees(np.arctan2(frd[:, 1], frd[:, 0])) % 360.0
        idx = np.floor(brg / INCREMENT_DEG + 0.5).astype(np.int64) % BINS
        nearest = np.full(BINS, np.inf)
        np.minimum.at(nearest, idx, rng_cm)
        return nearest

    def dead(self, s, n_bad, n_total, what):
        """A message with no data in it at all: not an observation of anything."""
        s.nan_frames += 1
        s.valid = False
        s.nearest = None
        if s.nan_frames == 5 or s.nan_frames % 200 == 0:
            self.get_logger().warn(
                f'{s.name}: {what} is all NaN ({n_bad} of {n_total}) for '
                f'{s.nan_frames} frames: the sensor is producing nothing. Dropped '
                f'from the histogram; with no sensor left, nothing is published '
                f'and PX4 holds. Measured cause on VMware: Gazebo\'s default '
                f'renderer on the SVGA3D GPU; start PX4 with '
                f'PX4_GZ_SIM_RENDER_ENGINE=ogre.')

    def on_cloud(self, s, msg: PointCloud2):
        s.last_t = self.now_s()
        s.warned_stale = False
        pts = point_cloud2.read_points_numpy(
            msg, field_names=('x', 'y', 'z'), skip_nans=False)
        # Gazebo marks beyond-range pixels +/-inf, which is a real observation
        # of open space. NaN is something else: a renderer that produced no
        # depth at all. Measured in a VMware VM under Gazebo's default ogre2:
        # 0 of 76,800 points finite in every frame, and the old code turned
        # that into "clear" in every bin. Treat an all-NaN cloud as no data.
        n_total = int(pts.shape[0])
        n_nan = int(np.isnan(pts).any(axis=1).sum()) if n_total else 0
        if n_total == 0 or n_nan >= 0.99 * n_total:
            self.dead(s, n_nan, n_total, 'depth cloud')
            return
        s.nan_frames = 0
        s.valid = True
        pts = pts[np.isfinite(pts).all(axis=1)]
        if pts.size == 0:
            s.nearest = None          # a valid cloud with nothing in range
            return
        if self.decimate > 1:
            pts = pts[::self.decimate]

        if self.diag and not s.logged:
            s.logged = True
            mn, mx = pts.min(axis=0), pts.max(axis=0)
            spans = mx - mn
            fwd = int(np.argmax(np.abs((mn + mx) / 2.0)))
            self.get_logger().info(
                f"{s.name}: cloud frame_id='{msg.header.frame_id}' points={len(pts)}\n"
                f"  x [{mn[0]:+.2f},{mx[0]:+.2f}] span {spans[0]:.2f}\n"
                f"  y [{mn[1]:+.2f},{mx[1]:+.2f}] span {spans[1]:.2f}\n"
                f"  z [{mn[2]:+.2f},{mx[2]:+.2f}] span {spans[2]:.2f}\n"
                f"  axis with the largest offset from zero: "
                f"{'xyz'[fwd]} -> that is most likely the forward/optical axis. "
                f"If it is z, {s.name}.frame='optical' is right; if x, use 'flu'.")

        s.nearest = self.bin_frd(s, self.sensor_to_frd(s, pts), band=True)

    def on_scan(self, s, msg: LaserScan):
        s.last_t = self.now_s()
        s.warned_stale = False
        r = np.asarray(msg.ranges, dtype=np.float64)
        n_total = int(r.size)
        n_nan = int(np.isnan(r).sum()) if n_total else 0
        if n_total == 0 or n_nan >= 0.99 * n_total:
            self.dead(s, n_nan, n_total, 'laser scan')
            return
        s.nan_frames = 0
        s.valid = True
        if s.observed is None:
            # Scan angles are counter-clockwise about z in the sensor frame
            # (REP 103); histogram bearings are clockwise. The arc
            # [angle_min, angle_max] is therefore bearings
            # [yaw - angle_max, yaw - angle_min].
            amin, amax = math.degrees(msg.angle_min), math.degrees(msg.angle_max)
            s.observed = observed_bins(s.yaw_deg - (amin + amax) / 2.0, (amax - amin) / 2.0)
            self.get_logger().info(
                f'{s.name}: scan frame_id={msg.header.frame_id!r}, {n_total} rays over '
                f'{amax - amin:.0f} deg, {msg.range_min:.2f} to {msg.range_max:.1f} m: '
                f'{int(s.observed.sum())} of {BINS} bins')
        a = msg.angle_min + np.arange(n_total) * msg.angle_increment
        # inf is beyond range, an observation of open space; NaN is not a
        # measurement; below range_min is the sensor's own body.
        ok = np.isfinite(r) & (r >= msg.range_min) & (r <= msg.range_max)
        if not ok.any():
            s.nearest = None
            return
        r, a = r[ok], a[ok]
        pts = np.column_stack((r * np.cos(a), r * np.sin(a), np.zeros_like(r)))
        s.nearest = self.bin_frd(s, self.sensor_to_frd(s, pts), band=False)

    def histogram(self, now):
        """The merged histogram at time `now`, or None when no sensor is alive.

        Returns (distances[72] as int64, min_cm, max_cm, alive sources).
        """
        alive = []
        for s in self.sources:
            if not s.valid or s.last_t is None:
                continue
            # A negative age means the clock went backwards (Gazebo restarted,
            # /clock reset), so the data is from the previous run. It is stale,
            # not fresh forever, which is what `age > stale_s` alone made it.
            age = now - s.last_t
            if age < 0.0 or age > self.stale_s:
                if not s.warned_stale:
                    s.warned_stale = True
                    self.get_logger().warn(
                        f'{s.name}: no data for {age:.1f} s, dropped from '
                        f'the histogram. Its bins report UNKNOWN; if it was the last '
                        f'live sensor nothing is published and PX4 holds rather than '
                        f'fly on a frozen histogram.')
                continue
            alive.append(s)
        if not alive:
            return None
        observed = np.zeros(BINS, dtype=bool)
        nearest = np.full(BINS, np.inf)
        for s in alive:
            if s.observed is not None:
                observed |= s.observed
            if s.nearest is not None:
                nearest = np.minimum(nearest, s.nearest)
        min_cm = min(s.min_cm for s in alive)
        max_cm = max(s.max_cm for s in alive)
        out = np.full(BINS, UINT16_MAX, dtype=np.int64)
        out[observed] = max_cm + 1                         # "no obstacle"
        hit = np.isfinite(nearest)
        out[hit] = np.clip(nearest[hit], 0, max_cm).astype(np.int64)
        return out, min_cm, max_cm, alive

    def on_timer(self):
        h = self.histogram(self.now_s())
        if h is None:
            return                      # no usable sensor: PX4 holds on its own timeout
        out, min_cm, max_cm, _ = h
        m = ObstacleDistance()
        m.timestamp = self.get_clock().now().nanoseconds // 1000
        m.frame = ObstacleDistance.MAV_FRAME_BODY_FRD      # 12. Not the default.
        m.sensor_type = ObstacleDistance.MAV_DISTANCE_SENSOR_LASER
        m.increment = float(INCREMENT_DEG)
        m.min_distance = int(min_cm)
        m.max_distance = int(max_cm)
        m.angle_offset = 0.0                               # bin 0 = straight ahead
        # rclpy does NOT enforce the fixed size of uint16[72]: a wrong length is
        # accepted here and fails later. Assert it where it is still debuggable.
        assert out.shape[0] == BINS, f'built {out.shape[0]} bins, need {BINS}'
        m.distances = out.astype(np.uint16).tolist()
        self.pub.publish(m)


def main():
    rclpy.init()
    node = ObstacleDistancePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
