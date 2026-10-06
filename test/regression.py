#!/usr/bin/env python3
"""The brake-mode half of the regression gate: did a change alter how it flies?

Two measurements, the same two the README numbers come from: the aircraft
holds a commanded heading, and collision prevention stops it short of a wall
at CP_DIST. Run it after any change to the pilot, the frames or the obstacle
node; if either number moves, the change did something it was not meant to.

It puts the pilot in brake mode first, so it is a valid gate whatever the last
script left the stack in. The plan-mode half is nav2_flight.py; gate.py runs
both and keeps score.

Picks the wall by position rather than assuming one, which is the mistake the
first version of this test made. The exit code is the number of failures.
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus, VehicleCommand
from std_msgs.msg import String

QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                 durability=DurabilityPolicy.VOLATILE,
                 history=HistoryPolicy.KEEP_LAST, depth=5)
# Same profile as avoidance_sim/frames.py MODE_QOS. The mode is retained state,
# and a VOLATILE publisher would not match the pilot's subscriber at all.
MODE_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL,
                      history=HistoryPolicy.KEEP_LAST, depth=1)

# walls.sdf, as the bridge parses it. East extent of each north-south wall.
WALL_FACES_EAST = (4.5, 11.5)
CP_DIST = 2.0
NAV_POSCTL = 2


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class R(Node):
    def __init__(self):
        super().__init__('regression')
        self.goal = self.create_publisher(PoseStamped, '/avoidance_sim/pilot_goal', 10)
        self.mode = self.create_publisher(String, '/avoidance_sim/mode', MODE_QOS)
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
        self.nav = None

    def on_pos(self, m):
        if math.isfinite(m.x):
            self.pos = (m.x, m.y, -m.z)
            self.yaw = m.heading

    def on_st(self, m):
        self.arm = m.arming_state
        self.nav = m.nav_state

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
        print("  no position")
        return 1
    print("  start north %+.2f east %+.2f alt %.2f armed %s"
          % (n.pos[0], n.pos[1], n.pos[2], n.arm))
    fails = 0

    # Brake mode, and wait for PX4 to actually be in Position mode. The pilot
    # re-requests it every two seconds, so this converges; if it does not,
    # the test still runs and the numbers will say so.
    m = String()
    m.data = 'brake'
    n.mode.publish(m)
    for _ in range(30):
        spin(n, 0.5)
        if n.nav == NAV_POSCTL:
            break
    print("  brake mode: nav_state %s %s"
          % (n.nav, "(Position)" if n.nav == NAV_POSCTL
             else "(NOT Position; collision prevention may not apply)"))

    if n.pos[2] < 4.0:
        n.vcmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 21196.0)
        spin(n, 3.0)
        if n.arm != 2:
            # Found by the clean-clone test: on a fresh install the x500
            # airframe's NAV_DLL_ACT default of 2 waits for a ground station.
            print("  PX4 did not arm (arming_state %s). On a fresh install the usual"
                  % n.arm)
            print("  cause is NAV_DLL_ACT at the airframe default of 2, waiting for a")
            print("  ground station. Start PX4 with PX4_PARAM_NAV_DLL_ACT=0 (README,")
            print("  Run) and retry.")
            return 1
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
    print("  TEST 2: avoidance. Face east, fly hard east, see what stops it.")
    # Heading matters here and it is not cosmetic. The camera covers 73
    # degrees ahead and nothing else, and CP_GO_NO_DATA 1 lets PX4 move into
    # directions it has no data for. TEST 1 leaves the aircraft facing west,
    # and the first version of this test then flew it east backwards: the
    # camera never saw the wall, nothing braked, and a 15 m wall at 7 m
    # altitude is a collision. The estimator diverged and every later
    # measurement was garbage. So hold east while flying east.
    n.send(east, north, 7.0, 90.0)
    for _ in range(20):
        spin(n, 1.0)
        if abs(math.degrees(wrap(math.radians(90.0) - n.yaw))) < 10.0:
            break
    print("    facing %+.0f deg" % math.degrees(n.yaw))
    n.send(100.0, north, 7.0, 90.0)
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
    return fails


if __name__ == '__main__':
    raise SystemExit(main())
