"""Draws the world's obstacles in RViz by reading the Gazebo world file.

Split out of a single-file prototype. The behaviour here is measured, not
assumed; see the repository README for the numbers and the traps.
"""


import os
import re

from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy)

from visualization_msgs.msg import Marker, MarkerArray


class WorldMarkers(Node):
    """Draw the Gazebo world's static geometry in RViz.

    Gazebo's world frame is ENU and shares PX4's local origin, so the SDF
    poses go into `odom` with no conversion. Measured rather than assumed:
    with the drone at gz (-5.004,-5.046,8.791) the TF read
    odom (-5.036,-5.058,8.778), and gz yaw -77.5 deg matched the ENU yaw of
    -78.4 deg while PX4's NED heading was 168.3 deg.

    If the wall markers and the depth cloud land on top of each other in
    RViz, the whole frame chain is consistent. That is the real check.
    """

    def __init__(self):
        super().__init__('world_markers')
        default = (f'{os.path.expanduser("~")}/PX4-Autopilot'
                   '/Tools/simulation/gz/worlds/walls.sdf')
        self.declare_parameter('world_sdf', default)
        self.declare_parameter('skip', ['ground_plane'])
        # `or default` matters: the launch file exposes world_sdf with an
        # empty default, and an empty override would otherwise win and leave
        # the markers silently missing.
        path = self.get_parameter('world_sdf').value or default
        skip = set(self.get_parameter('skip').value)

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub = self.create_publisher(MarkerArray, '/world_markers', qos)
        self.msg = self.build(path, skip)
        self.create_timer(2.0, lambda: self.pub.publish(self.msg))
        self.pub.publish(self.msg)

    def build(self, path, skip):
        arr = MarkerArray()
        if not os.path.isfile(path):
            self.get_logger().error(f'world file not found: {path}')
            return arr
        text = open(path).read()
        n = 0
        for m in re.finditer(r'<model name=[\'"]([^\'"]+)[\'"]>(.*?)</model>',
                             text, re.S):
            name, body = m.group(1), m.group(2)
            if name in skip:
                continue
            pose = re.search(r'<pose>([^<]+)</pose>', body)
            box = re.search(r'<box>\s*<size>([^<]+)</size>', body, re.S)
            if not (pose and box):
                continue
            px, py, pz = [float(v) for v in pose.group(1).split()[:3]]
            sx, sy, sz = [float(v) for v in box.group(1).split()[:3]]
            mk = Marker()
            mk.header.frame_id = 'odom'
            mk.header.stamp = self.get_clock().now().to_msg()
            mk.ns, mk.id = 'world', n
            mk.type, mk.action = Marker.CUBE, Marker.ADD
            mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = px, py, pz
            mk.pose.orientation.w = 1.0
            mk.scale.x, mk.scale.y, mk.scale.z = sx, sy, sz
            mk.color.r, mk.color.g, mk.color.b, mk.color.a = 0.62, 0.62, 0.66, 0.35
            arr.markers.append(mk)

            lbl = Marker()
            lbl.header.frame_id = 'odom'
            lbl.header.stamp = mk.header.stamp
            lbl.ns, lbl.id = 'world_labels', n
            lbl.type, lbl.action = Marker.TEXT_VIEW_FACING, Marker.ADD
            lbl.pose.position.x, lbl.pose.position.y = px, py
            lbl.pose.position.z = 2.0
            lbl.pose.orientation.w = 1.0
            lbl.scale.z = 0.8
            lbl.color.r = lbl.color.g = lbl.color.b = 1.0
            lbl.color.a = 0.8
            lbl.text = name
            arr.markers.append(lbl)
            n += 1
            self.get_logger().info(
                f'{name}: centre ({px:+.1f},{py:+.1f},{pz:+.1f}) '
                f'size ({sx:.1f},{sy:.1f},{sz:.1f}) -> '
                f'east {px-sx/2:+.1f}..{px+sx/2:+.1f}, '
                f'north {py-sy/2:+.1f}..{py+sy/2:+.1f}')
        self.get_logger().info(f'published {n} world bodies from '
                               f'{os.path.basename(path)}')
        return arr
