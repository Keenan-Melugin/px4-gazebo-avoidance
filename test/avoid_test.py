#!/usr/bin/env python3
"""Avoidance regression, from a controlled start position.

The previous run could not judge anything because the aircraft had drifted
100 m east of both walls, so there was nothing ahead of it. This repositions
with avoidance off (otherwise it cannot get back past the walls it is being
protected from), then turns avoidance on and flies at a known wall.
"""
import math
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import (VehicleCommand, VehicleLocalPosition,
                          VehicleStatus)

QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                 durability=DurabilityPolicy.VOLATILE,
                 history=HistoryPolicy.KEEP_LAST, depth=5)

WALL_FACE = 4.50      # box1, east face
CP_DIST = 2.0
START_EAST = -4.0


def param(name, value):
    subprocess.run(["px4-param", "set", name, str(value)],
                   capture_output=True, timeout=20)


class A(Node):
    def __init__(self):
        super().__init__('avoid_test')
        self.goal = self.create_publisher(PoseStamped, '/avoidance_sim/pilot_goal', 10)
        self.cmd = self.create_publisher(VehicleCommand,
                                         '/fmu/in/vehicle_command', QOS)
        self.create_subscription(VehicleLocalPosition,
                                 '/fmu/out/vehicle_local_position_v1',
                                 self.on_pos, QOS)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1',
                                 self.on_status, QOS)
        self.pos = None
        self.armed = None
        self.healthy = None

    def on_status(self, m):
        self.armed = (m.arming_state == 2)
        self.healthy = m.pre_flight_checks_pass

    def on_pos(self, m):
        if math.isfinite(m.x):
            self.pos = (m.x, m.y, -m.z)

    def send(self, east, north, alt):
        g = PoseStamped()
        g.header.stamp = self.get_clock().now().to_msg()
        g.header.frame_id = 'odom'
        g.pose.position.x = float(east)
        g.pose.position.y = float(north)
        g.pose.position.z = float(alt)
        g.pose.orientation.w = 1.0
        self.goal.publish(g)

    def stop_pilot(self):
        """Deactivate the pilot so it stops commanding throttle.

        PX4 denies arming while the throttle stick is above center, and an
        active climb goal holds it there, so this has to happen before arming.
        """
        g = PoseStamped()
        g.header.stamp = self.get_clock().now().to_msg()
        g.header.frame_id = 'STOP'
        self.goal.publish(g)

    def arm(self):
        v = VehicleCommand()
        v.timestamp = self.get_clock().now().nanoseconds // 1000
        v.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        v.param1, v.param2 = 1.0, 21196.0
        v.target_system = v.target_component = 1
        v.source_system = v.source_component = 1
        v.from_external = True
        self.cmd.publish(v)


def spin(n, s):
    t = time.time()
    while time.time() - t < s:
        rclpy.spin_once(n, timeout_sec=0.05)


def main():
    rclpy.init()
    n = A()
    spin(n, 4.0)
    if n.pos is None:
        print("  no position"); return 1
    print("  start north %+.2f east %+.2f alt %.2f" % n.pos)

    print("  stopping the pilot first, so the throttle stick returns to center")
    n.stop_pilot()
    spin(n, 4.0)

    # Arming has two preconditions that both bite in practice. PX4 refuses
    # while the throttle stick is above center, which an active climb goal
    # holds it at, hence the stop above. And it refuses until the barometer
    # and EKF have settled, which takes tens of seconds after boot, so a
    # single arm command sent too early fails and is never retried.
    if not n.armed:
        for attempt in range(20):
            if n.healthy:
                n.arm()
                spin(n, 2.0)
                if n.armed:
                    print("  armed after %d attempt(s)" % (attempt + 1))
                    break
            else:
                n.stop_pilot()
            spin(n, 3.0)
        if not n.armed:
            print("  could not arm: healthy=%s. Check the PX4 console."
                  % n.healthy)
            return 1
    else:
        print("  already armed")
    spin(n, 2.0)

    print("\n  repositioning to east %.1f with avoidance OFF" % START_EAST)
    param("CP_DIST", -1.0)
    spin(n, 2.0)
    n.send(START_EAST, 0.0, 6.0)
    for i in range(22):
        spin(n, 3.0)
        if abs(n.pos[1] - START_EAST) < 1.5 and n.pos[2] > 4.0:
            break
    print("  now north %+.2f east %+.2f alt %.2f" % n.pos)
    if n.pos[1] > WALL_FACE:
        print("  could not get west of the wall, cannot run the test")
        return 1

    print("\n  avoidance ON, flying at the wall (face east %+.2f)" % WALL_FACE)
    param("CP_DIST", CP_DIST)
    spin(n, 2.0)
    n.send(20.0, 0.0, 6.0)
    for i in range(14):
        spin(n, 3.0)
        print("    t+%2ds  north %+6.2f east %+6.2f alt %.2f"
              % ((i + 1) * 3, n.pos[0], n.pos[1], n.pos[2]))

    east = n.pos[1]
    gap = WALL_FACE - east
    print()
    print("  ===== RESULT =====")
    print("  settled at east %+.2f, so %.2f m short of the wall face" % (east, gap))
    print("  CP_DIST is %.1f" % CP_DIST)
    if 1.0 < gap < 3.5:
        print("  PASS: braked short of the wall, consistent with the earlier")
        print("        1.98 m and 2.60 m measurements")
    elif gap <= 1.0:
        print("  FAIL: got too close, or through it")
    else:
        print("  FAIL: stopped %.2f m out, far more than CP_DIST" % gap)
    n.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
