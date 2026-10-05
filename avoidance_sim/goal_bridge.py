"""Turns RViz's flat Goal Pose tool into a PX4 reposition command.

Split out of a single-file prototype. The behaviour here is measured, not
assumed; see the repository README for the numbers and the traps.
"""


import math

from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleCommand, VehicleLocalPosition
from visualization_msgs.msg import Marker

from .frames import PX4_QOS


class GoalBridge(Node):
    def __init__(self):
        super().__init__('rviz_goal_bridge')
        self.home = None
        self.alt = None
        # Origin comes from VehicleLocalPosition.ref_lat/ref_lon, NOT from
        # HomePosition. HomePosition is published only when home is SET, and
        # the uXRCE-DDS publishers are VOLATILE, so a subscriber that starts
        # after that event waits forever and every goal is silently dropped.
        # ref_lat/ref_lon is also the NED frame origin, which is the right
        # reference for an offset in NED metres; home is only where it armed.
        self.create_subscription(VehicleLocalPosition,
                                 '/fmu/out/vehicle_local_position_v1',
                                 self.on_local, PX4_QOS)
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 10)
        self.cmd = self.create_publisher(VehicleCommand,
                                         '/fmu/in/vehicle_command', PX4_QOS)
        self.mk = self.create_publisher(Marker, '/goal_marker', 10)
        self.get_logger().info(
            'ready: use the "2D Goal Pose" button in RViz to fly somewhere')

    def on_local(self, m):
        if math.isfinite(m.z):
            self.alt = -m.z
        if m.xy_global and math.isfinite(m.ref_lat) and math.isfinite(m.ref_lon):
            if self.home is None:
                self.get_logger().info(
                    f'frame origin: {m.ref_lat:.7f}, {m.ref_lon:.7f}')
            self.home = (m.ref_lat, m.ref_lon, m.ref_alt)

    def on_goal(self, msg: PoseStamped):
        if self.home is None:
            self.get_logger().warn(
                'no frame origin yet (xy_global false?), ignoring goal')
            return
        # RViz fixed frame is ENU: x east, y north
        east = msg.pose.position.x
        north = msg.pose.position.y
        alt = self.alt if self.alt and self.alt > 1.0 else 6.0

        lat0, lon0, _ = self.home
        lat = lat0 + north / 111320.0
        lon = lon0 + east / (111320.0 * math.cos(math.radians(lat0)))

        c = VehicleCommand()
        c.timestamp = self.get_clock().now().nanoseconds // 1000
        c.command = VehicleCommand.VEHICLE_CMD_DO_REPOSITION
        c.param1 = -1.0          # default ground speed
        c.param2 = 1.0           # required: PX4 will not switch modes without it
        c.param5 = float(lat)
        c.param6 = float(lon)
        c.param7 = float(alt)
        c.target_system = 1
        c.target_component = 1
        c.source_system = 1
        c.source_component = 1
        c.from_external = True
        self.cmd.publish(c)
        self.get_logger().info(
            f'goal: north {north:+.2f} east {east:+.2f} alt {alt:.1f} '
            f'-> reposition ({lat:.7f}, {lon:.7f})')

        mk = Marker()
        mk.header.frame_id = 'odom'
        mk.header.stamp = self.get_clock().now().to_msg()
        mk.ns, mk.id, mk.type, mk.action = 'goal', 0, Marker.CYLINDER, Marker.ADD
        mk.pose.position.x = float(east)
        mk.pose.position.y = float(north)
        mk.pose.position.z = float(alt) / 2.0
        mk.pose.orientation.w = 1.0
        mk.scale.x, mk.scale.y, mk.scale.z = 0.5, 0.5, alt
        mk.color.r, mk.color.g, mk.color.b, mk.color.a = 1.0, 0.75, 0.1, 0.45
        self.mk.publish(mk)
