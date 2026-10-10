#!/usr/bin/env python3
"""Separate 'does it turn' from 'does it stay turned'.

Streams zero sticks itself (exactly what the idle pilot does) and watches the
heading. If heading walks away with zero yaw stick, the problem is not the
control law and no amount of gain tuning will fix it.
"""
import math
import time

import rclpy
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from regression import ensure_airborne, guarded, set_pilot_mode  # noqa: E402
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy)
from px4_msgs.msg import (ManualControlSetpoint, VehicleLocalPosition,
                          VehicleStatus)

QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                 durability=DurabilityPolicy.VOLATILE,
                 history=HistoryPolicy.KEEP_LAST, depth=5)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class H(Node):
    def __init__(self):
        super().__init__('hold_test')
        self.pub = self.create_publisher(ManualControlSetpoint,
                                         '/fmu/in/manual_control_input', QOS)
        self.create_subscription(VehicleLocalPosition,
                                 '/fmu/out/vehicle_local_position_v1',
                                 self.on_pos, QOS)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1',
                                 self.on_st, QOS)
        self.yawstick = 0.0
        self.yaw = None
        self.alt = 0.0
        self.nav = None
        self.create_timer(0.02, self.tick)

    def on_pos(self, m):
        if math.isfinite(m.x):
            self.yaw = m.heading
            self.alt = -m.z

    def on_st(self, m):
        self.nav = m.nav_state

    def tick(self):
        m = ManualControlSetpoint()
        m.timestamp = self.get_clock().now().nanoseconds // 1000
        m.timestamp_sample = m.timestamp
        m.valid = True
        m.data_source = ManualControlSetpoint.SOURCE_MAVLINK_0
        m.roll = m.pitch = m.throttle = 0.0
        m.yaw = self.yawstick
        self.pub.publish(m)


def spin(n, secs):
    t = time.time()
    while time.time() - t < secs:
        rclpy.spin_once(n, timeout_sec=0.05)


def main():
    rclpy.init()
    # Take off first if needed: this used to assume the aircraft was
    # already flying, and on a fresh stack measured nothing (2026-10-10).
    if ensure_airborne() is None:
        print("  could not arm and take off"); return 1
    # This script streams its own sticks. With the pilot streaming too,
    # two publishers interleave on one PX4 input and the zero-stick hold
    # measures the pilot, not PX4. guarded() puts brake back afterwards.
    set_pilot_mode('external')
    n = H()
    # Up to 20 s for the first data. A fixed 4 s was enough on the
    # development machine and not on a 4-core one, where DDS discovery had
    # not finished and the script quit before measuring anything.
    for _ in range(40):
        spin(n, 0.5)
        if n.yaw is not None:
            break
    if n.yaw is None:
        print("  no heading"); return 1

    print("  TEST A: zero yaw stick for 30 s. Does the heading stay put?")
    print("    alt %.1f  nav %s" % (n.alt, n.nav))
    y0 = n.yaw
    for i in range(6):
        spin(n, 5.0)
        print("      t+%2ds  heading %+7.1f   drift %+6.1f deg"
              % ((i + 1) * 5, math.degrees(n.yaw),
                 math.degrees(wrap(n.yaw - y0))))
    total = math.degrees(wrap(n.yaw - y0))
    print("    drift over 30 s: %+.1f deg  -> %s"
          % (total, "HOLDS" if abs(total) < 10 else "DOES NOT HOLD"))

    print()
    print("  TEST B: turn 90 deg at a moderate stick, then release and watch.")
    y0 = n.yaw
    target = wrap(y0 + math.radians(90.0))
    n.yawstick = 0.35
    t0 = time.time()
    while time.time() - t0 < 25.0:
        spin(n, 0.1)
        if abs(math.degrees(wrap(target - n.yaw))) < 3.0:
            break
    turned = math.degrees(wrap(n.yaw - y0))
    n.yawstick = 0.0
    print("    released at %+.1f deg turned (wanted +90), after %.1f s"
          % (turned, time.time() - t0))
    for i in range(5):
        spin(n, 3.0)
        print("      +%2ds  heading %+7.1f   past target %+6.1f deg"
              % ((i + 1) * 3, math.degrees(n.yaw),
                 math.degrees(wrap(n.yaw - target))))
    coast = math.degrees(wrap(n.yaw - target))
    print("    coast past the cut point: %+.1f deg" % coast)

    print()
    print("  ===== READING =====")
    if abs(total) >= 10:
        print("  Heading does NOT hold with zero stick, so the 'overshoot' in")
        print("  the earlier test was drift measured 22 s after arrival, not")
        print("  a control problem. Fix the hold, not the gains.")
    elif abs(coast) > 10:
        print("  Heading holds, but it coasts %.0f deg past the cut point, so" % abs(coast))
        print("  the control law needs to brake rather than simply stop.")
    else:
        print("  Both fine: holds, and coasts only %.1f deg." % abs(coast))
    n.yawstick = 0.0
    spin(n, 1.0)
    n.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(guarded(main))
