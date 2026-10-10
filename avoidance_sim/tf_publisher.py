"""PX4 odometry -> the TF tree and nav_msgs/Odometry, in ROS frame conventions.

Both outputs live here because they are the same PX4 message put through the
same NED-to-ENU conversion, and that conversion is the one that fails
silently. Doing it twice in two nodes would be the worse mistake.

/odom exists for Nav2, which requires nav_msgs/Odometry and will not start
without it. Note the twist is in the CHILD frame (base_link, FLU), per the
nav_msgs convention, not in the odom frame like the pose.

Rate. PX4 publishes odometry at about 100 Hz and this node used to forward
every message. Nothing downstream can use that: RViz draws at 30 frames a
second, Nav2's controller looks TF up at 10 Hz with a 0.2 s tolerance, and
the costmaps at 5 Hz. So it publishes at 30 Hz and drops the rest. Measured
with the live stack, this node alone went from 13.7% to 9.5% of a core, and
most of what remains is receiving the 100 Hz input, which is PX4's choice
rather than ours.

Split out of a single-file prototype. The behaviour here is measured, not
assumed; see docs/how-it-works.md for the numbers and the traps.
"""

import math

from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from visualization_msgs.msg import Marker

from .frames import CAM_XYZ, LIDAR_XYZ, PX4_QOS, ned_to_enu, rotate_inv


def _finite3(a):
    return math.isfinite(a[0]) and math.isfinite(a[1]) and math.isfinite(a[2])


class TfPublisher(Node):
    PUBLISH_HZ = 30.0

    def __init__(self):
        super().__init__('px4_tf_publisher',
                         start_parameter_services=False)  # see rviz_bridge.py
        self.br = TransformBroadcaster(self)
        self.static = StaticTransformBroadcaster(self)
        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry',
                                 self.on_odom, PX4_QOS)
        self.marker = self.create_publisher(Marker, '/drone', 10)
        self.odom = self.create_publisher(Odometry, '/odom', 10)
        self.publish_static()
        self.period = 1.0 / self.PUBLISH_HZ
        self.next_pub = 0.0
        self.n = 0
        self.vel_frame_seen = None

    def publish_static(self):
        # Both sensor frames, whether or not the lidar model is flying: an
        # unused frame costs nothing, a missing one leaves RViz unable to
        # place the scan.
        frames = []
        for child, xyz in (('camera_link', CAM_XYZ), ('lidar_link', LIDAR_XYZ)):
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'base_link'
            t.child_frame_id = child
            t.transform.translation.x = xyz[0]
            t.transform.translation.y = xyz[1]
            t.transform.translation.z = xyz[2]
            t.transform.rotation.w = 1.0
            frames.append(t)
        self.static.sendTransform(frames)
        self.get_logger().info(
            f'static base_link -> camera_link at {CAM_XYZ}, '
            f'lidar_link at {LIDAR_XYZ} (no rotation)')

    def publish_odom(self, now, x, y, z, q, q_body_ned, m):
        """nav_msgs/Odometry for Nav2. q is (w, x, y, z), ENU/FLU.

        The twist belongs in the child frame (base_link, FLU). PX4 reports
        velocity in whichever frame `velocity_frame` names at runtime, so it
        has to be read rather than assumed: NED and FRD are world frames and
        need rotating into the body first, BODY_FRD is already body-fixed and
        only needs the FRD-to-FLU sign flip.
        """
        o = Odometry()
        o.header.stamp = now
        o.header.frame_id = 'odom'
        o.child_frame_id = 'base_link'
        o.pose.pose.position.x, o.pose.pose.position.y = x, y
        o.pose.pose.position.z = z
        o.pose.pose.orientation.w, o.pose.pose.orientation.x = q[0], q[1]
        o.pose.pose.orientation.y, o.pose.pose.orientation.z = q[2], q[3]

        vf = int(m.velocity_frame)
        if vf != self.vel_frame_seen:
            self.vel_frame_seen = vf
            name = {1: 'NED (world)', 2: 'FRD (world)',
                    3: 'BODY_FRD'}.get(vf, 'UNKNOWN(%d)' % vf)
            self.get_logger().info(
                f'PX4 velocity_frame = {vf} = {name}; converting to base_link FLU')

        if _finite3(m.velocity):
            v = (float(m.velocity[0]), float(m.velocity[1]), float(m.velocity[2]))
            if vf == VehicleOdometry.VELOCITY_FRAME_BODY_FRD:
                body_frd = v
            else:
                # World (NED or FRD) -> body FRD, using the body->world rotation.
                body_frd = rotate_inv(q_body_ned, v)
            o.twist.twist.linear.x = body_frd[0]
            o.twist.twist.linear.y = -body_frd[1]   # FRD -> FLU
            o.twist.twist.linear.z = -body_frd[2]

        # angular_velocity is documented as always BODY_FRD.
        if _finite3(m.angular_velocity):
            o.twist.twist.angular.x = float(m.angular_velocity[0])
            o.twist.twist.angular.y = float(-m.angular_velocity[1])
            o.twist.twist.angular.z = float(-m.angular_velocity[2])

        # Diagonal covariance from PX4's variances. Nav2 does not use these
        # heavily, but leaving them zero claims perfect certainty.
        if _finite3(m.position_variance):
            pv = m.position_variance
            o.pose.covariance[0] = float(pv[1])    # x (east) <- PX4 y
            o.pose.covariance[7] = float(pv[0])    # y (north) <- PX4 x
            o.pose.covariance[14] = float(pv[2])
        if _finite3(m.orientation_variance):
            ov = m.orientation_variance
            o.pose.covariance[21] = float(ov[0])
            o.pose.covariance[28] = float(ov[1])
            o.pose.covariance[35] = float(ov[2])
        if _finite3(m.velocity_variance):
            vv = m.velocity_variance
            o.twist.covariance[0] = float(vv[0])
            o.twist.covariance[7] = float(vv[1])
            o.twist.covariance[14] = float(vv[2])

        self.odom.publish(o)

    def on_odom(self, m):
        if not _finite3(m.position) or not (_finite3(m.q) and math.isfinite(m.q[3])):
            return
        # 30 Hz out of a 100 Hz input. Decided on time, not on message count,
        # so it stays 30 Hz if PX4's rate changes.
        now_s = self.get_clock().now().nanoseconds * 1e-9
        # Simulation time goes backwards when Gazebo restarts under a running
        # ROS side. Without this reset the next slot stays at the old time and
        # the transform stops until the new clock catches up: measured
        # 2026-10-10, clock 153 s -> 21 s, Nav2 blind for over 90 s.
        if now_s < self.next_pub - 1.0:
            self.next_pub = now_s
        if now_s < self.next_pub:
            return
        # Schedule the next slot rather than gate on "a period since the last
        # one": that gate quantises to the input spacing and gave 25 Hz from
        # a 100 Hz input. This keeps the average exact and never falls behind.
        self.next_pub = max(self.next_pub + self.period, now_s - self.period)
        now = self.get_clock().now().to_msg()

        # position: NED -> ENU
        n, e, d = float(m.position[0]), float(m.position[1]), float(m.position[2])
        x, y, z = e, n, -d

        # attitude: PX4 q is (w,x,y,z) body->NED; both frame turns applied.
        q_body_ned = (float(m.q[0]), float(m.q[1]), float(m.q[2]), float(m.q[3]))
        q = ned_to_enu(q_body_ned)                 # (w, x, y, z)

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation.w, t.transform.rotation.x = q[0], q[1]
        t.transform.rotation.y, t.transform.rotation.z = q[2], q[3]
        self.br.sendTransform(t)

        self.publish_odom(now, x, y, z, q, q_body_ned, m)

        self.n += 1
        if self.n == 1:
            self.get_logger().info('odometry flowing, publishing odom -> base_link at %.0f Hz'
                                   % self.PUBLISH_HZ)

        # The body marker is static geometry attached to base_link, so
        # republishing it at the TF rate is pure cost. Every 6th publish is
        # 5 Hz, which is ample.
        if self.n % 6 != 1:
            return
        mk = Marker()
        mk.header.frame_id = 'base_link'
        mk.header.stamp = now
        mk.ns, mk.id, mk.type, mk.action = 'drone', 0, Marker.CUBE, Marker.ADD
        mk.scale.x, mk.scale.y, mk.scale.z = 0.45, 0.45, 0.12
        mk.color.r, mk.color.g, mk.color.b, mk.color.a = 0.1, 0.5, 0.95, 0.9
        mk.pose.orientation.w = 1.0
        self.marker.publish(mk)
