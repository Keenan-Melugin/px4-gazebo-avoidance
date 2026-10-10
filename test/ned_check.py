#!/usr/bin/env python3
"""Is the FLU-to-NED conversion in plan mode correct?

The earlier body-frame check was invalid. Plan mode slaves the heading to the
direction of travel, so a sustained lateral command makes the aircraft turn to
face that way and fly forward instead, and "body right" is then measured in a
frame that is itself rotating.

The meaningful question is the world one: with the nose starting north, a
command to go RIGHT must move the aircraft EAST, whatever it does with its
heading on the way. That is immune to the rotation.
"""
import math
import subprocess
import time

import rclpy
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from regression import ensure_airborne  # noqa: E402
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus

QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                 durability=DurabilityPolicy.VOLATILE,
                 history=HistoryPolicy.KEEP_LAST, depth=5)
# Must match the pilot's subscriber. A VOLATILE publisher to a TRANSIENT_LOCAL
# subscriber is not a QoS mismatch warning, it is silence.
MODE_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL,
                      history=HistoryPolicy.KEEP_LAST, depth=1)


class C(Node):
    def __init__(self):
        super().__init__('ned_check')
        self.mode = self.create_publisher(String, '/avoidance_sim/mode',
                                          MODE_QOS)
        self.cv = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pilot = self.create_publisher(PoseStamped,
                                           '/avoidance_sim/pilot_goal', 10)
        self.create_subscription(VehicleLocalPosition,
                                 '/fmu/out/vehicle_local_position_v1',
                                 self.on_pos, QOS)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1',
                                 self.on_st, QOS)
        self.pos = None
        self.yaw = 0.0
        self.alt = 0.0
        self.nav = None
        self.drive = None
        self.create_timer(0.1, self.tick)

    def on_pos(self, m):
        if math.isfinite(m.x):
            self.pos = (m.x, m.y)
            self.alt = -m.z
            self.yaw = m.heading

    def on_st(self, m):
        self.nav = m.nav_state

    def tick(self):
        if self.drive is not None:
            t = Twist()
            t.linear.x, t.linear.y = self.drive
            self.cv.publish(t)

    def set_mode(self, s):
        m = String(); m.data = s; self.mode.publish(m)

    def goal(self, e, n, a, h):
        # Wait for the pilot to be connected before the first goal. A goal
        # published on a new publisher before discovery completes is lost
        # without a trace (measured 2026-10-10 in yaw_test.py).
        if not getattr(self, '_goal_matched', False):
            t0 = time.time()
            while self.pilot.get_subscription_count() == 0 and time.time() - t0 < 10.0:
                rclpy.spin_once(self, timeout_sec=0.1)
            self._goal_matched = True
        g = PoseStamped()
        g.header.stamp = self.get_clock().now().to_msg()
        g.header.frame_id = 'odom+yaw'
        g.pose.position.x, g.pose.position.y, g.pose.position.z = float(e), float(n), float(a)
        enu = math.pi / 2.0 - math.radians(h)
        g.pose.orientation.w = math.cos(enu / 2.0)
        g.pose.orientation.z = math.sin(enu / 2.0)
        self.pilot.publish(g)


def spin(n, s):
    t = time.time()
    while time.time() - t < s:
        rclpy.spin_once(n, timeout_sec=0.02)


def trial(n, label, drive, want):
    """want: 'north', 'east', 'west'. Returns pass/fail."""
    # re-point north in brake mode so every trial starts the same
    n.drive = None
    n.set_mode('brake')
    spin(n, 3.0)
    n.goal(n.pos[1], n.pos[0], 9.0, 0.0)
    for _ in range(25):
        spin(n, 1.0)
        if abs(math.degrees(n.yaw)) < 8:
            break
    n.set_mode('plan')
    n.drive = (0.0, 0.0)
    spin(n, 7.0)
    p0 = n.pos
    y0 = math.degrees(n.yaw)
    n.drive = drive
    spin(n, 11.0)
    p1 = n.pos
    y1 = math.degrees(n.yaw)
    n.drive = (0.0, 0.0)
    spin(n, 3.0)
    dn, de = p1[0] - p0[0], p1[1] - p0[1]
    got = {"north": dn, "east": de, "west": -de}[want]
    ok = got > 1.0
    print("  %-26s world dn %+6.2f de %+6.2f | heading %+.0f -> %+.0f | %s"
          % (label, dn, de, y0, y1, "ok" if ok else "WRONG"))
    return ok


def main():
    rclpy.init()
    # Take off first if needed: this used to assume the aircraft was
    # already flying, and on a fresh stack measured nothing (2026-10-10).
    if ensure_airborne() is None:
        print("  could not arm and take off"); return 1
    n = C()
    # Up to 20 s for the first data. A fixed 4 s was enough on the
    # development machine and not on a 4-core one, where DDS discovery had
    # not finished and the script quit before measuring anything.
    for _ in range(40):
        spin(n, 0.5)
        if n.pos is not None:
            break
    if n.pos is None:
        print("  no position"); return 1
    subprocess.run(["px4-param", "set", "CP_DIST", "2.0"], capture_output=True)
    print("  alt %.1f, CP_DIST 2.0 throughout" % n.alt)
    if n.alt < 6.0:
        print("  ABORT: not airborne"); return 2
    print()
    print("  each trial: re-point north in brake mode, switch to plan, command,")
    print("  then measure WORLD displacement over 11 s")
    print()
    results = [
        trial(n, "forward (FLU +x)", (1.0, 0.0), "north"),
        trial(n, "right (FLU -y)", (0.0, -1.0), "east"),
        trial(n, "left (FLU +y)", (0.0, 1.0), "west"),
    ]
    print()
    print("  ===== %s =====" % ("NED CONVERSION CORRECT" if all(results)
                                else "CONVERSION WRONG"))
    n.drive = None
    n.set_mode('brake')
    spin(n, 2.0)
    n.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
