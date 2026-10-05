"""The right-click menu above the aircraft: arm, take off, land, disarm.

Split out of a single-file prototype. The behaviour here is measured, not
assumed; see the repository README for the numbers and the traps.
"""


from rclpy.node import Node

from interactive_markers import InteractiveMarkerServer, MenuHandler
from px4_msgs.msg import VehicleCommand, VehicleCommandAck, VehicleStatus
from visualization_msgs.msg import (InteractiveMarker,
    InteractiveMarkerControl,
    Marker)

from .frames import PX4_QOS


class CommandMarker(Node):
    """Right-click the drone in RViz to arm / take off / land / disarm.

    RViz panels are C++ only (rviz_common PanelFactory is a PluginlibFactory
    over a C++ base), so a dockable button panel cannot be written in Python.
    An InteractiveMarker menu can, costs no compile step, and carries correctly
    labelled entries, which a repurposed third-party panel would not.

    Takeoff is deliberately a TWO command sequence, mode then arm, gated on the
    ack. That is what PX4's own `commander takeoff` does
    (Commander.cpp:338-342) and the intuitive arm-then-takeoff order does not
    work: NAV_TAKEOFF only changes the mode intention and ignores its lat/lon/
    alt params entirely, so the altitude comes from MIS_TAKEOFF_ALT.
    """

    FORCE = 21196.0          # PX4 magic value: override preflight / disarm in air

    def __init__(self):
        super().__init__('px4_command_marker')
        self.cmd = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', PX4_QOS)
        self.create_subscription(VehicleCommandAck, '/fmu/out/vehicle_command_ack',
                                 self.on_ack, PX4_QOS)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1',
                                 self.on_status, PX4_QOS)
        self.pending_arm = False
        self.armed = None

        self.server = InteractiveMarkerServer(self, 'px4_commands')
        self.menu = MenuHandler()
        self.menu.insert('ARM', callback=lambda fb: self.arm())
        self.menu.insert('TAKEOFF', callback=lambda fb: self.takeoff())
        self.menu.insert('LAND', callback=lambda fb: self.land())
        self.menu.insert('DISARM', callback=lambda fb: self.disarm())
        self.menu.insert('DISARM (FORCE, in air)',
                         callback=lambda fb: self.disarm(force=True))

        im = InteractiveMarker()
        im.header.frame_id = 'base_link'      # follows the aircraft
        im.name = 'px4_commands'
        im.description = 'right-click: PX4 commands'
        im.scale = 1.0
        im.pose.position.z = 1.0              # float above it so it stays clickable
        im.pose.orientation.w = 1.0

        ball = Marker()
        ball.type = Marker.SPHERE
        ball.scale.x = ball.scale.y = ball.scale.z = 0.5
        ball.color.r, ball.color.g, ball.color.b, ball.color.a = 1.0, 0.45, 0.1, 0.85
        ball.pose.orientation.w = 1.0

        ctrl = InteractiveMarkerControl()
        ctrl.interaction_mode = InteractiveMarkerControl.MENU
        ctrl.always_visible = True
        ctrl.markers.append(ball)
        im.controls.append(ctrl)

        self.server.insert(im)
        self.menu.apply(self.server, im.name)
        self.server.applyChanges()
        self.get_logger().info(
            'command marker up: right-click the orange ball above the drone')

    # ---------------- plumbing ----------------
    def send(self, command, p1=0.0, p2=0.0):
        m = VehicleCommand()
        m.timestamp = self.get_clock().now().nanoseconds // 1000   # message is [us]
        m.command = int(command)
        m.param1, m.param2 = float(p1), float(p2)
        m.target_system = 1
        m.target_component = 1
        m.source_system = 1
        m.source_component = 1
        m.from_external = True
        self.cmd.publish(m)

    def on_status(self, m):
        self.armed = (m.arming_state == 2)

    def on_ack(self, m):
        if (self.pending_arm
                and m.command == VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF):
            self.pending_arm = False
            if m.result == 0:
                self.get_logger().info('takeoff mode accepted, arming')
                self.send(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                          VehicleCommand.ARMING_ACTION_ARM, 0.0)
            else:
                self.get_logger().warn(
                    f'takeoff mode rejected (result {m.result}), not arming')

    # ---------------- actions ----------------
    def arm(self):
        self.get_logger().info('ARM')
        self.send(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                  VehicleCommand.ARMING_ACTION_ARM, 0.0)

    def disarm(self, force=False):
        self.get_logger().info('DISARM' + (' (forced)' if force else ''))
        self.send(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                  VehicleCommand.ARMING_ACTION_DISARM,
                  self.FORCE if force else 0.0)

    def takeoff(self):
        self.get_logger().info('TAKEOFF: mode first, arm on ack')
        self.pending_arm = True
        self.send(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF)

    def land(self):
        self.get_logger().info('LAND')
        self.send(VehicleCommand.VEHICLE_CMD_NAV_LAND)
