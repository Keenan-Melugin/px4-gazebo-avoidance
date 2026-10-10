"""The draggable 3D waypoint, with arrows for position and a ring for heading.

Split out of a single-file prototype. The behaviour here is measured, not
assumed; see the repository README for the numbers and the traps.
"""


import math

from rclpy.node import Node

from geometry_msgs.msg import PointStamped, PoseStamped
from interactive_markers import InteractiveMarkerServer, MenuHandler
from px4_msgs.msg import VehicleCommand, VehicleLocalPosition
from visualization_msgs.msg import (InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker)

from .frames import PX4_QOS, YAW_SUFFIX, quat_onto, wrap, yaw_of


class Goal3D(Node):
    """A genuine 3D waypoint you drag in RViz, then right-click to fly to.

    Why this shape:
      - RViz's "2D Goal Pose" tool projects the click onto the ground plane
        (PoseTool calls getViewportPointProjectionOnXYPlane), so z is always 0.
        It cannot express altitude, full stop.
      - RViz's "Publish Point" tool is different: it does a depth-buffer pick
        (PointTool calls getViewPicker()->get3DPoint) with NO plane fallback.
        So clicking the depth cloud or a wall yields that surface's true 3D
        coordinate on /clicked_point. We use it to PLACE the goal, not to fly
        to it, because the thing you click is usually the obstacle.
      - MOVE_3D is not an altitude handle: its shift-drag moves along the
        camera ray, not world Z. Altitude needs a MOVE_AXIS arrow.
      - Markerless MOVE_AXIS controls need no geometry; RViz's client
        auto-generates the arrow pair.

    DO_REPOSITION already carries altitude in param7, so full 3D needs no
    Offboard mode and no setpoint stream.
    """

    def __init__(self):
        super().__init__('goal_3d',
                         start_parameter_services=False)  # see rviz_bridge.py
        self.origin = None
        self.pose = None
        self.cmd = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', PX4_QOS)
        self.create_subscription(VehicleLocalPosition,
                                 '/fmu/out/vehicle_local_position_v1',
                                 self.on_local, PX4_QOS)
        self.create_subscription(PointStamped, '/clicked_point',
                                 self.on_click, 10)

        self.server = InteractiveMarkerServer(self, 'goal_3d')
        self.pilot_goal = self.create_publisher(
            PoseStamped, '/avoidance_sim/pilot_goal', 10)
        self.menu = MenuHandler()
        self.menu.insert('FLY HERE (avoidance ON)',
                         callback=lambda fb: self.fly_piloted())
        self.menu.insert('FLY HERE (direct, NO avoidance)',
                         callback=lambda fb: self.fly())
        self.menu.insert('STOP', callback=lambda fb: self.stop_pilot())
        self.menu.insert('snap to aircraft', callback=lambda fb: self.snap())

        im = InteractiveMarker()
        im.header.frame_id = 'odom'
        im.name = 'goal'
        im.description = '3D goal: arrows move, ring turns, right-click to fly'
        im.scale = 1.5
        im.pose.position.z = 6.0
        im.pose.orientation.w = 1.0

        ball = Marker()
        ball.type = Marker.SPHERE
        ball.scale.x = ball.scale.y = ball.scale.z = 0.6
        ball.color.r, ball.color.g, ball.color.b, ball.color.a = 0.15, 0.85, 0.3, 0.85
        ball.pose.orientation.w = 1.0
        # Which way it will face on arrival. Points along the marker's
        # local +x, and ENU +x is east, so an unrotated goal faces east.
        nose = Marker()
        nose.type = Marker.ARROW
        nose.scale.x, nose.scale.y, nose.scale.z = 1.4, 0.12, 0.2
        nose.color.r, nose.color.g, nose.color.b, nose.color.a = 1.0, 0.9, 0.1, 0.9
        nose.pose.orientation.w = 1.0

        grab = InteractiveMarkerControl()
        grab.name = 'grab'
        grab.interaction_mode = InteractiveMarkerControl.MENU
        grab.always_visible = True
        grab.markers.append(ball)
        grab.markers.append(nose)
        im.controls.append(grab)

        # Three world-axis arrows. FIXED so they stay world-aligned.
        for name, q in (('move_east', (1.0, 1.0, 0.0, 0.0)),
                        ('move_north', (1.0, 0.0, 0.0, 1.0)),
                        ('move_up', (1.0, 0.0, 1.0, 0.0))):
            c = InteractiveMarkerControl()
            c.name = name
            c.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            c.orientation_mode = InteractiveMarkerControl.FIXED
            quat_onto(c, *q)
            im.controls.append(c)

        # Yaw ring. Same quaternion as the up arrow, because ROTATE_AXIS turns
        # about the control's local x rotated by it, and FIXED keeps that axis
        # on world Z instead of letting it tumble with the marker.
        ring = InteractiveMarkerControl()
        ring.name = 'turn'
        ring.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        ring.orientation_mode = InteractiveMarkerControl.FIXED
        quat_onto(ring, 1.0, 0.0, 1.0, 0.0)
        im.controls.append(ring)

        self.server.insert(im, feedback_callback=self.on_feedback)
        self.menu.apply(self.server, im.name)
        self.server.applyChanges()
        self.pose = im.pose
        self.get_logger().info(
            'goal_3d up: drag the arrows for east/north/altitude, the ring '
            'to turn, right-click the green ball to fly. The yellow arrow is '
            'the heading it will hold. Publish Point places it (a real pick).')

    def on_local(self, m):
        if m.xy_global and math.isfinite(m.ref_lat):
            self.origin = (m.ref_lat, m.ref_lon, m.ref_alt)
        self.here = (m.y, m.x, -m.z) if math.isfinite(m.x) else None

    def on_feedback(self, fb):
        if fb.event_type in (InteractiveMarkerFeedback.POSE_UPDATE,
                             InteractiveMarkerFeedback.MOUSE_UP):
            self.pose = fb.pose

    def on_click(self, msg: PointStamped):
        """Publish Point gives a real 3D surface point. Place the goal there
        rather than flying to it: what you clicked is usually the obstacle."""
        p = self.server.get('goal')
        if p is None:
            return
        p.pose.position.x = msg.point.x
        p.pose.position.y = msg.point.y
        p.pose.position.z = max(1.0, msg.point.z)
        self.pose = p.pose
        self.server.insert(p, feedback_callback=self.on_feedback)
        self.menu.apply(self.server, 'goal')
        self.server.applyChanges()
        self.get_logger().info(
            f'goal placed from click: east {msg.point.x:+.2f} '
            f'north {msg.point.y:+.2f} alt {p.pose.position.z:.2f} '
            f'(frame {msg.header.frame_id})')

    def snap(self):
        if getattr(self, 'here', None) is None:
            self.get_logger().warn('no position yet')
            return
        e, n, u = self.here
        g = self.server.get('goal')
        g.pose.position.x, g.pose.position.y, g.pose.position.z = e, n, max(1.0, u)
        self.pose = g.pose
        self.server.insert(g, feedback_callback=self.on_feedback)
        self.menu.apply(self.server, 'goal')
        self.server.applyChanges()
        self.get_logger().info('goal snapped to the aircraft')

    def fly_piloted(self):
        """Hand the goal to the software pilot, which flies it on sticks in
        Position mode so collision prevention applies."""
        if self.pose is None:
            self.get_logger().warn('no goal pose yet')
            return
        if self.pose.position.z < 1.0:
            self.get_logger().warn('refusing: goal at or below ground')
            return
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        # The ring is part of this marker, so its heading is intended.
        msg.header.frame_id = 'odom' + YAW_SUFFIX
        msg.pose = self.pose
        self.pilot_goal.publish(msg)
        self.get_logger().info(
            'handed to the software pilot, heading %+.0f deg (avoidance ON)'
            % math.degrees(wrap(math.pi / 2.0 - yaw_of(self.pose.orientation))))

    def stop_pilot(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'STOP'
        self.pilot_goal.publish(msg)
        self.get_logger().info('STOP sent')

    def fly(self):
        if self.origin is None or self.pose is None:
            self.get_logger().warn('no frame origin yet, cannot fly')
            return
        east, north = self.pose.position.x, self.pose.position.y
        alt = self.pose.position.z
        if alt < 1.0:
            self.get_logger().warn(
                f'refusing: goal altitude {alt:.2f} m is at or below ground')
            return
        lat0, lon0, _ = self.origin
        lat = lat0 + north / 111320.0
        lon = lon0 + east / (111320.0 * math.cos(math.radians(lat0)))

        c = VehicleCommand()
        c.timestamp = self.get_clock().now().nanoseconds // 1000
        c.command = VehicleCommand.VEHICLE_CMD_DO_REPOSITION
        c.param1 = -1.0
        c.param2 = 1.0                 # required, PX4 will not switch modes without it
        c.param5 = float(lat)
        c.param6 = float(lon)
        c.param7 = float(alt)          # this is what makes it a 3D waypoint
        c.target_system = 1
        c.target_component = 1
        c.source_system = 1
        c.source_component = 1
        c.from_external = True
        self.cmd.publish(c)
        self.get_logger().info(
            f'FLY HERE: east {east:+.2f} north {north:+.2f} alt {alt:.2f} m')
