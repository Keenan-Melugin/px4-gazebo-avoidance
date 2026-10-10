"""Draws the world's obstacles in RViz by reading the Gazebo world file.

Split out of a single-file prototype. The behaviour here is measured, not
assumed; see docs/how-it-works.md for the numbers and the traps.
"""


import math
import os

from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy)

from visualization_msgs.msg import Marker, MarkerArray

from avoidance_sim import world_geometry


class WorldMarkers(Node):
    """Draw the Gazebo world's static geometry in RViz.

    Gazebo's world frame is ENU and shares PX4's local origin, so the SDF
    poses go into `odom` with no conversion. Measured rather than assumed:
    with the drone at gz (-5.004,-5.046,8.791) the TF read
    odom (-5.036,-5.058,8.778), and gz yaw -77.5 deg matched the ENU yaw of
    -78.4 deg while PX4's NED heading was 168.3 deg.

    If the wall markers and the depth cloud land on top of each other in
    RViz, the whole frame chain is consistent. That is the real check.

    Which file: `world_sdf` (a path) if set, else `world` (a name, the same
    one PX4 was given as PX4_GZ_WORLD) looked up where PX4 looks. The
    parsing lives in world_geometry.py, shared with the measurement
    scripts, so the walls RViz draws and the walls the tests expect come
    from the same code reading the same file.
    """

    def __init__(self):
        super().__init__('world_markers',
                         start_parameter_services=False)  # see rviz_bridge.py
        self.declare_parameter('world', 'walls')
        self.declare_parameter('world_sdf', '')
        self.declare_parameter('skip', ['ground_plane'])
        # `or` matters: the launch file exposes world_sdf with an empty
        # default, and an empty override would otherwise win and leave the
        # markers silently missing.
        spec = (self.get_parameter('world_sdf').value
                or self.get_parameter('world').value or 'walls')
        skip = set(self.get_parameter('skip').value)

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub = self.create_publisher(MarkerArray, '/world_markers', qos)
        self.msg = self.build(spec, skip)
        self.create_timer(2.0, lambda: self.pub.publish(self.msg))
        self.pub.publish(self.msg)

    def build(self, spec, skip):
        arr = MarkerArray()
        try:
            path = world_geometry.resolve_world(spec)
        except FileNotFoundError as exc:
            self.get_logger().error(str(exc))
            return arr
        boxes = world_geometry.load_boxes(path, skip=skip)
        for n, b in enumerate(boxes):
            mk = Marker()
            mk.header.frame_id = 'odom'
            mk.header.stamp = self.get_clock().now().to_msg()
            mk.ns, mk.id = 'world', n
            mk.type, mk.action = Marker.CUBE, Marker.ADD
            mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = b.cx, b.cy, b.cz
            # Yaw about world z. The bundled worlds are all axis-aligned; a
            # yawed box is drawn where it is, and the measurement scripts
            # skip it (world_geometry.faces_ahead says so).
            mk.pose.orientation.z = math.sin(b.yaw / 2.0)
            mk.pose.orientation.w = math.cos(b.yaw / 2.0)
            mk.scale.x, mk.scale.y, mk.scale.z = b.sx, b.sy, b.sz
            mk.color.r, mk.color.g, mk.color.b, mk.color.a = 0.62, 0.62, 0.66, 0.35
            arr.markers.append(mk)

            lbl = Marker()
            lbl.header.frame_id = 'odom'
            lbl.header.stamp = mk.header.stamp
            lbl.ns, lbl.id = 'world_labels', n
            lbl.type, lbl.action = Marker.TEXT_VIEW_FACING, Marker.ADD
            lbl.pose.position.x, lbl.pose.position.y = b.cx, b.cy
            lbl.pose.position.z = 2.0
            lbl.pose.orientation.w = 1.0
            lbl.scale.z = 0.8
            lbl.color.r = lbl.color.g = lbl.color.b = 1.0
            lbl.color.a = 0.8
            lbl.text = b.name
            arr.markers.append(lbl)
            self.get_logger().info(b.describe())
        self.get_logger().info(
            f'published {len(boxes)} world bodies from {os.path.basename(path)} '
            f'(world name {world_geometry.world_name(path)!r})')
        return arr
