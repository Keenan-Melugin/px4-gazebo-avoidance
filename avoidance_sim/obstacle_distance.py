#!/usr/bin/env python3
"""Turn a depth point cloud into PX4's ObstacleDistance polar histogram.

Subscribes : /depth_camera/points   sensor_msgs/PointCloud2
Publishes  : /fmu/in/obstacle_distance   px4_msgs/ObstacleDistance

PX4's collision prevention consumes obstacle_distance and brakes. This node does
perception only; it never commands the vehicle.

Defaults match the simulated OAK-D Lite in PX4 v1.17.0
(Tools/simulation/gz/models/OakD-Lite/model.sdf):
    horizontal_fov 1.274 rad = 73.0 deg, clip near 0.2 m, far 19.1 m, 30 Hz.
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from px4_msgs.msg import ObstacleDistance

BINS = 72                      # fixed by the message: uint16[72]
INCREMENT_DEG = 360.0 / BINS   # 5.0
UINT16_MAX = 65535             # "unknown", per the message definition


class ObstacleDistancePublisher(Node):

    def __init__(self):
        super().__init__('obstacle_distance_publisher')

        p = self.declare_parameter
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
        p('height_band_m', 1.0)        # keep points within this of the vehicle plane
        p('mount_xyz_frd', [0.12, -0.03, -0.242])   # SDF pose, FLU -> FRD
        p('publish_hz', 10.0)
        # A 640x480 depth image is 307200 points. Processing all of them takes
        # ~400 ms here, which starves the executor and drops the publish rate
        # from 10 Hz to ~2 Hz. Every Nth point is plenty for a 72-bin histogram.
        p('decimate', 8)
        p('diagnostics', True)

        g = lambda n: self.get_parameter(n).value
        self.cloud_frame = g('cloud_frame')
        self.min_cm = int(g('min_distance_cm'))
        self.max_cm = int(g('max_distance_cm'))
        self.hfov = float(g('hfov_deg'))
        self.band = float(g('height_band_m'))
        self.mount = np.array(g('mount_xyz_frd'), dtype=np.float64)
        self.diag = bool(g('diagnostics'))
        self.decimate = max(1, int(g('decimate')))

        # Which bins the camera can see at all. Everything else stays "unknown".
        half = self.hfov / 2.0
        self.observed = np.zeros(BINS, dtype=bool)
        for i in range(BINS):
            centre = i * INCREMENT_DEG
            signed = (centre + 180.0) % 360.0 - 180.0   # -> [-180, 180)
            if abs(signed) <= half:
                self.observed[i] = True
        self.get_logger().info(
            f'camera sees {int(self.observed.sum())} of {BINS} bins '
            f'({self.hfov:.1f} deg). The other '
            f'{BINS - int(self.observed.sum())} report UNKNOWN, which PX4 treats '
            f'as blocked unless CP_GO_NO_DATA=1.')

        # PX4 publishes/subscribes best-effort; a reliable subscriber will not match.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5)

        self.pub = self.create_publisher(
            ObstacleDistance, '/fmu/in/obstacle_distance', qos)
        self.sub = self.create_subscription(
            PointCloud2, g('cloud_topic'), self.on_cloud, qos)

        self.latest = None
        self.create_timer(1.0 / float(g('publish_hz')), self.on_timer)
        self._logged_frame = False

    # ------------------------------------------------------------------
    def to_frd(self, pts: np.ndarray) -> np.ndarray:
        """Map cloud axes onto body FRD: x forward, y right, z down."""
        x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
        if self.cloud_frame == 'optical':      # x right, y down, z forward
            frd = np.column_stack((z, x, y))
        elif self.cloud_frame == 'flu':        # x forward, y left, z up
            frd = np.column_stack((x, -y, -z))
        else:
            raise ValueError(f'unknown cloud_frame {self.cloud_frame!r}')
        return frd + self.mount

    def on_cloud(self, msg: PointCloud2):
        pts = point_cloud2.read_points_numpy(
            msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if pts.size == 0:
            self.latest = None
            return
        # Gazebo emits +/-inf for no-return pixels. skip_nans does not catch
        # those. The range gate below happens to reject them, but rely on it
        # explicitly rather than by accident.
        pts = pts[np.isfinite(pts).all(axis=1)]
        if pts.size == 0:
            self.latest = None
            return
        if self.decimate > 1:
            pts = pts[::self.decimate]

        if self.diag and not self._logged_frame:
            self._logged_frame = True
            mn, mx = pts.min(axis=0), pts.max(axis=0)
            spans = mx - mn
            fwd = int(np.argmax(np.abs((mn + mx) / 2.0)))
            self.get_logger().info(
                f"cloud frame_id='{msg.header.frame_id}' points={len(pts)}\n"
                f"  x [{mn[0]:+.2f},{mx[0]:+.2f}] span {spans[0]:.2f}\n"
                f"  y [{mn[1]:+.2f},{mx[1]:+.2f}] span {spans[1]:.2f}\n"
                f"  z [{mn[2]:+.2f},{mx[2]:+.2f}] span {spans[2]:.2f}\n"
                f"  axis with the largest offset from zero: "
                f"{'xyz'[fwd]} -> that is most likely the forward/optical axis. "
                f"If it is z, cloud_frame='optical' is right; if x, use 'flu'.")

        frd = self.to_frd(pts)

        # Keep a horizontal slab around the vehicle. At low altitude the ground
        # enters this band and marks as an obstacle in every direction.
        frd = frd[np.abs(frd[:, 2]) <= self.band]
        if frd.shape[0] == 0:
            self.latest = None
            return

        rng_cm = np.hypot(frd[:, 0], frd[:, 1]) * 100.0
        keep = (rng_cm >= self.min_cm) & (rng_cm <= self.max_cm)
        frd, rng_cm = frd[keep], rng_cm[keep]
        if frd.shape[0] == 0:
            self.latest = None
            return

        # Bearing, clockwise-positive from forward, as the message requires.
        brg = np.degrees(np.arctan2(frd[:, 1], frd[:, 0])) % 360.0
        idx = np.floor(brg / INCREMENT_DEG + 0.5).astype(np.int64) % BINS

        nearest = np.full(BINS, np.inf)
        np.minimum.at(nearest, idx, rng_cm)
        self.latest = nearest

    def on_timer(self):
        m = ObstacleDistance()
        m.timestamp = self.get_clock().now().nanoseconds // 1000
        m.frame = ObstacleDistance.MAV_FRAME_BODY_FRD      # 12. Not the default.
        m.sensor_type = ObstacleDistance.MAV_DISTANCE_SENSOR_LASER
        m.increment = float(INCREMENT_DEG)
        m.min_distance = self.min_cm
        m.max_distance = self.max_cm
        m.angle_offset = 0.0                               # bin 0 = straight ahead

        clear = self.max_cm + 1                            # "no obstacle"
        out = np.full(BINS, UINT16_MAX, dtype=np.int64)
        out[self.observed] = clear
        if self.latest is not None:
            hit = np.isfinite(self.latest)
            out[hit] = np.clip(self.latest[hit], 0, self.max_cm).astype(np.int64)

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
