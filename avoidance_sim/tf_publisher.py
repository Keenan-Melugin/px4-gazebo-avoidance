"""Publishes the TF tree PX4's odometry implies, converted into the ROS frame convention.

Split out of a single-file prototype. The behaviour here is measured, not
assumed; see the repository README for the numbers and the traps.
"""


import numpy as np
from scipy.spatial.transform import Rotation

from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
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
        self.publish_static()
        self.n = 0

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
