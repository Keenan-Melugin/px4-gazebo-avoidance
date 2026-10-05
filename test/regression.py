#!/usr/bin/env python3
"""Did splitting the file change how it flies?

Re-measures the two things that were measured before the split: it holds a
commanded heading, and collision prevention stops it short of a wall. Picks the
wall by position rather than assuming one, which is the mistake the first
version of this test made.
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus, VehicleCommand

QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                 durability=DurabilityPolicy.VOLATILE,
                 history=HistoryPolicy.KEEP_LAST, depth=5)

# walls.sdf, as the bridge parses it. East extent of each north-south wall.
WALL_FACES_EAST = (4.5, 11.5)
CP_DIST = 2.0


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class R(Node):
    def __init__(self):
        super().__init__('regression')
        self.goal = self.create_publisher(PoseStamped, '/evtol/pilot_goal', 10)
        self.cmd = self.create_publisher(VehicleCommand,
                                         '/fmu/in/vehicle_command', QOS)
        self.create_subscription(VehicleLocalPosition,
                                 '/fmu/out/vehicle_local_position_v1',
                                 self.on_pos, QOS)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1',
                                 self.on_st, QOS)
        self.pos = None
        self.yaw = 0.0
        self.arm = None

    def on_pos(self, m):
        if math.isfinite(m.x):
            self.pos = (m.x, m.y, -m.z)
            self.yaw = m.heading

    def on_st(self, m):
        self.arm = m.arming_state

    def send(self, east, north, alt, hdg=None):
        g = PoseStamped()
        g.header.stamp = self.get_clock().now().to_msg()
        g.header.frame_id = 'odom' if hdg is None else 'odom+yaw'
        g.pose.position.x = float(east)
        g.pose.position.y = float(north)
        g.pose.position.z = float(alt)
        enu = math.pi / 2.0 - math.radians(hdg or 0.0)
        g.pose.orientation.w = math.cos(enu / 2.0)
        g.pose.orientation.z = math.sin(enu / 2.0)
        self.goal.publish(g)

    def vcmd(self, c, p1=0.0, p2=0.0):
        v = VehicleCommand()
        v.timestamp = self.get_clock().now().nanoseconds // 1000
        v.command = c
        v.param1, v.param2 = p1, p2
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
    n = R()
    spin(n, 4.0)
    if n.pos is None:
        print("  no position"); return 1
    print("  start north %+.2f east %+.2f alt %.2f armed %s"
          % (n.pos[0], n.pos[1], n.pos[2], n.arm))
    fails = 0

    if n.pos[2] < 4.0:
        n.vcmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 21196.0)
        spin(n, 3.0)
        n.send(n.pos[1], n.pos[0], 7.0)
        for _ in range(30):
            spin(n, 1.0)
            if n.pos[2] > 6.0:
                break
        print("  climbed to %.2f m" % n.pos[2])

    print()
    print("  TEST 1: heading. Commanding 4 headings on the spot.")
    east, north = n.pos[1], n.pos[0]
    worst = 0.0
    for hdg in (0.0, 90.0, 180.0, -90.0):
        n.send(east, north, 7.0, hdg)
        spin(n, 22.0)
        err = abs(math.degrees(wrap(math.radians(hdg) - n.yaw)))
        worst = max(worst, err)
        print("    %+4.0f deg -> settled %+7.1f, error %4.1f deg  %s"
              % (hdg, math.degrees(n.yaw), err, "ok" if err <= 12 else "FAIL"))
    if worst > 12:
        fails += 1
    print("    worst heading error: %.1f deg" % worst)

    print()
    print("  TEST 2: avoidance. Fly hard east and see what stops it.")
    n.send(100.0, north, 7.0)
    for i in range(16):
        spin(n, 3.0)
    east_f = n.pos[1]
    ahead = [f for f in WALL_FACES_EAST if f > east_f]
    print("    settled at east %+.2f" % east_f)
    if not ahead:
        print("    no wall ahead, cannot judge            FAIL")
        fails += 1
    else:
        face = min(ahead)
        gap = face - east_f
        ok = abs(gap - CP_DIST) < 0.7
        print("    nearest wall ahead at east %+.1f, gap %.2f m vs CP_DIST %.1f  %s"
              % (face, gap, CP_DIST, "ok" if ok else "FAIL"))
        if not ok:
            fails += 1

    print()
    print("  ===== %s =====" % ("NO REGRESSION" if fails == 0
                                else "%d REGRESSION(S)" % fails))
    n.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
