"""PX4 odometry -> the TF tree and nav_msgs/Odometry, in ROS frame conventions.

Both outputs live here because they are the same PX4 message put through the
same NED-to-ENU conversion, and that conversion is the one that fails
silently. Doing it twice in two nodes would be the worse mistake.

/odom exists for Nav2, which requires nav_msgs/Odometry and will not start
without it. Note the twist is in the CHILD frame (base_link, FLU), per the
nav_msgs convention, not in the odom frame like the pose.

Split out of a single-file prototype. The behaviour here is measured, not
assumed; see the repository README for the numbers and the traps.
"""


import numpy as np
from scipy.spatial.transform import Rotation

from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from visualization_msgs.msg import Marker

from .frames import CAM_XYZ, PX4_QOS, R_FRD_FLU, R_NED_ENU


class TfPublisher(Node):
    def __init__(self):
        super().__init__('px4_tf_publisher')
        self.br = TransformBroadcaster(self)
        self.static = StaticTransformBroadcaster(self)
        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry',
                                 self.on_odom, PX4_QOS)
        self.marker = self.create_publisher(Marker, '/drone', 10)
        self.odom = self.create_publisher(Odometry, '/odom', 10)
        self.publish_static()
        self.n = 0
        self.vel_frame_seen = None

    def publish_static(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'camera_link'
        t.transform.translation.x = CAM_XYZ[0]
        t.transform.translation.y = CAM_XYZ[1]
        t.transform.translation.z = CAM_XYZ[2]
        t.transform.rotation.w = 1.0
        self.static.sendTransform(t)
        self.get_logger().info(
            f'static base_link -> camera_link at {CAM_XYZ} (no rotation)')

    def publish_odom(self, now, x, y, z, q, q_body_ned, m):
        """nav_msgs/Odometry for Nav2.

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
        o.pose.pose.orientation.x, o.pose.pose.orientation.y = float(q[0]), float(q[1])
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = float(q[2]), float(q[3])

        vf = int(m.velocity_frame)
        if vf != self.vel_frame_seen:
            self.vel_frame_seen = vf
            name = {1: 'NED (world)', 2: 'FRD (world)',
                    3: 'BODY_FRD'}.get(vf, 'UNKNOWN(%d)' % vf)
            self.get_logger().info(
                f'PX4 velocity_frame = {vf} = {name}; converting to base_link FLU')

        if np.isfinite(m.velocity).all():
            v = np.array([float(m.velocity[0]), float(m.velocity[1]),
                          float(m.velocity[2])])
            if vf == VehicleOdometry.VELOCITY_FRAME_BODY_FRD:
                body_frd = v
            else:
                # World (NED or FRD) -> body FRD, using the body->world rotation.
                body_frd = q_body_ned.inv().apply(v)
            o.twist.twist.linear.x = float(body_frd[0])
            o.twist.twist.linear.y = float(-body_frd[1])   # FRD -> FLU
            o.twist.twist.linear.z = float(-body_frd[2])

        # angular_velocity is documented as always BODY_FRD.
        if np.isfinite(m.angular_velocity).all():
            o.twist.twist.angular.x = float(m.angular_velocity[0])
            o.twist.twist.angular.y = float(-m.angular_velocity[1])
            o.twist.twist.angular.z = float(-m.angular_velocity[2])

        # Diagonal covariance from PX4's variances. Nav2 does not use these
        # heavily, but leaving them zero claims perfect certainty.
        if np.isfinite(m.position_variance).all():
            pv = m.position_variance
            o.pose.covariance[0] = float(pv[1])    # x (east) <- PX4 y
            o.pose.covariance[7] = float(pv[0])    # y (north) <- PX4 x
            o.pose.covariance[14] = float(pv[2])
        if np.isfinite(m.orientation_variance).all():
            ov = m.orientation_variance
            o.pose.covariance[21] = float(ov[0])
            o.pose.covariance[28] = float(ov[1])
            o.pose.covariance[35] = float(ov[2])
        if np.isfinite(m.velocity_variance).all():
            vv = m.velocity_variance
            o.twist.covariance[0] = float(vv[0])
            o.twist.covariance[7] = float(vv[1])
            o.twist.covariance[14] = float(vv[2])

        self.odom.publish(o)

    def on_odom(self, m):
        if not np.isfinite(m.position).all() or not np.isfinite(m.q).all():
            return
        now = self.get_clock().now().to_msg()

        # position: NED -> ENU
        n, e, d = float(m.position[0]), float(m.position[1]), float(m.position[2])
        x, y, z = e, n, -d

        # attitude: PX4 q is (w,x,y,z) body->NED. scipy wants (x,y,z,w).
        q_body_ned = Rotation.from_quat([m.q[1], m.q[2], m.q[3], m.q[0]])
        q = (R_NED_ENU * q_body_ned * R_FRD_FLU).as_quat()   # x,y,z,w

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation.x, t.transform.rotation.y = float(q[0]), float(q[1])
        t.transform.rotation.z, t.transform.rotation.w = float(q[2]), float(q[3])
        self.br.sendTransform(t)

        self.publish_odom(now, x, y, z, q, q_body_ned, m)

        # The TF above must go out at full rate, but the body marker is static
        # geometry attached to base_link: republishing it 23 times a second is
        # pure cost. Every 5th message is 4-5 Hz, which is ample.
        if self.n % 5 != 0:
            self.n += 1
            return
        mk = Marker()
        mk.header.frame_id = 'base_link'
        mk.header.stamp = now
        mk.ns, mk.id, mk.type, mk.action = 'drone', 0, Marker.CUBE, Marker.ADD
        mk.scale.x, mk.scale.y, mk.scale.z = 0.45, 0.45, 0.12
        mk.color.r, mk.color.g, mk.color.b, mk.color.a = 0.1, 0.5, 0.95, 0.9
        mk.pose.orientation.w = 1.0
        self.marker.publish(mk)

        self.n += 1
        if self.n == 1:
            self.get_logger().info('odometry flowing, publishing odom -> base_link')
