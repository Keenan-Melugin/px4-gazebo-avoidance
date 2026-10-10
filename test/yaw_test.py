#!/usr/bin/env python3
"""Does the pilot now turn the aircraft to a commanded heading?

Publishes goals exactly as the RViz marker does, including the +yaw intent
suffix, and measures the heading it settles on. Also checks that a goal
WITHOUT the suffix leaves the heading alone, which is the opt-in guarantee.
"""
import math
import time

import rclpy
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from regression import guarded  # noqa: E402
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy)
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus, VehicleCommand

QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                 durability=DurabilityPolicy.VOLATILE,
                 history=HistoryPolicy.KEEP_LAST, depth=5)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class T(Node):
    def __init__(self):
        super().__init__('yaw_test')
        self.goal = self.create_publisher(PoseStamped, '/avoidance_sim/pilot_goal', 10)
        self.cmd = self.create_publisher(VehicleCommand,
                                         '/fmu/in/vehicle_command', QOS)
        self.create_subscription(VehicleLocalPosition,
                                 '/fmu/out/vehicle_local_position_v1',
                                 self.on_pos, QOS)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1',
                                 self.on_st, QOS)
        self.pos = None
        self.yaw = 0.0
        self.nav = None
        self.arm = None

    def on_pos(self, m):
        if math.isfinite(m.x):
            self.pos = (m.x, m.y, -m.z)
            self.yaw = m.heading

    def on_st(self, m):
        self.nav = m.nav_state
        self.arm = m.arming_state

    def send(self, east, north, alt, hdg_deg=None):
        # Wait for the pilot to be connected before the first goal. A goal
        # published on a new publisher before discovery completes is lost
        # without a trace: measured 2026-10-10, the heading test's first
        # command never reached the pilot in two runs of three.
        if not getattr(self, '_goal_matched', False):
            t0 = time.time()
            while self.goal.get_subscription_count() == 0 and time.time() - t0 < 10.0:
                rclpy.spin_once(self, timeout_sec=0.1)
            self._goal_matched = True
        """hdg_deg is the NED heading wanted, or None for position only."""
        g = PoseStamped()
        g.header.stamp = self.get_clock().now().to_msg()
        g.header.frame_id = 'odom' if hdg_deg is None else 'odom+yaw'
        g.pose.position.x = float(east)
        g.pose.position.y = float(north)
        g.pose.position.z = float(alt)
        enu = math.pi / 2.0 - math.radians(hdg_deg or 0.0)
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


def spin(n, secs):
    t = time.time()
    while time.time() - t < secs:
        rclpy.spin_once(n, timeout_sec=0.05)


def main():
    rclpy.init()
    n = T()
    # Up to 20 s for the first data. A fixed 4 s was enough on the
    # development machine and not on a 4-core one, where DDS discovery had
    # not finished and the script quit before measuring anything.
    for _ in range(40):
        spin(n, 0.5)
        if n.pos is not None:
            break
    if n.pos is None:
        print("  no position"); return 1
    print("  start: alt %.2f  heading %+.1f  nav %s  armed %s"
          % (n.pos[2], math.degrees(n.yaw), n.nav, n.arm))

    if n.pos[2] < 4.0:
        print("  arming and climbing on the pilot")
        n.vcmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 21196.0)
        spin(n, 3.0)
        e0, nn0 = n.pos[1], n.pos[0]
        n.send(e0, nn0, 7.0)
        for _ in range(30):
            spin(n, 1.0)
            if n.pos[2] > 6.0:
                break
        print("  now alt %.2f" % n.pos[2])
    if n.pos[2] < 4.0:
        print("  FAILED to get airborne"); return 1

    east, north, alt = n.pos[1], n.pos[0], 7.0
    print("\n  holding position east %.1f north %.1f, turning on the spot" % (east, north))
    print("  %-14s %-12s %-12s %s" % ("commanded", "settled", "error", "verdict"))
    bad = 0
    for hdg in (0.0, 90.0, 180.0, -90.0):
        n.send(east, north, alt, hdg)
        spin(n, 22.0)
        got = math.degrees(n.yaw)
        err = math.degrees(wrap(math.radians(hdg) - n.yaw))
        ok = abs(err) <= 12.0
        if not ok:
            bad += 1
        print("  %-14s %-12s %-12s %s"
              % ("%+.0f deg" % hdg, "%+.1f deg" % got, "%+.1f deg" % err,
                 "PASS" if ok else "FAIL"))

    # the opt-in guarantee
    print("\n  now a position-only goal: heading must NOT change")
    before = n.yaw
    n.send(east + 3.0, north, alt)          # no heading
    spin(n, 20.0)
    drift = math.degrees(wrap(n.yaw - before))
    print("    heading %+.1f -> %+.1f, drift %+.1f deg  %s"
          % (math.degrees(before), math.degrees(n.yaw), drift,
             "PASS" if abs(drift) <= 12.0 else "FAIL"))
    if abs(drift) > 12.0:
        bad += 1

    print("\n  ===== %s =====" % ("ALL PASS" if bad == 0 else "%d FAILED" % bad))
    n.destroy_node()
    rclpy.try_shutdown()
    return bad


if __name__ == '__main__':
    raise SystemExit(guarded(main))
