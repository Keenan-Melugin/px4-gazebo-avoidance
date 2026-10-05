#!/usr/bin/env python3
"""Find the smallest yaw stick that actually rotates the aircraft.

The pilot was commanding 0.10 and the heading did not budge for 40 s, so there
is a threshold between 0.10 and 0.50 (0.50 was measured at 24 deg/s). This
sweeps it so the gain design rests on a number instead of a guess.
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy)
from px4_msgs.msg import ManualControlSetpoint, VehicleLocalPosition

QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                 durability=DurabilityPolicy.VOLATILE,
                 history=HistoryPolicy.KEEP_LAST, depth=5)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class P(Node):
    def __init__(self):
        super().__init__('yaw_threshold')
        self.pub = self.create_publisher(ManualControlSetpoint,
                                         '/fmu/in/manual_control_input', QOS)
        self.create_subscription(VehicleLocalPosition,
                                 '/fmu/out/vehicle_local_position_v1',
                                 self.on_pos, QOS)
        self.stick = 0.0
        self.yaw = None
        self.alt = 0.0
        self.create_timer(0.02, self.tick)

    def on_pos(self, m):
        if math.isfinite(m.x):
            self.yaw = m.heading
            self.alt = -m.z

    def tick(self):
        m = ManualControlSetpoint()
        m.timestamp = self.get_clock().now().nanoseconds // 1000
        m.timestamp_sample = m.timestamp
        m.valid = True
        m.data_source = ManualControlSetpoint.SOURCE_MAVLINK_0
        m.roll = m.pitch = m.throttle = 0.0
        m.yaw = self.stick
        self.pub.publish(m)


def spin(n, secs):
    t = time.time()
    while time.time() - t < secs:
        rclpy.spin_once(n, timeout_sec=0.05)


def main():
    rclpy.init()
    n = P()
    spin(n, 4.0)
    if n.yaw is None:
        print("  no heading"); return 1
    print("  airborne at %.1f m, heading %+.1f" % (n.alt, math.degrees(n.yaw)))
    print()
    print("  %-8s %-14s %s" % ("stick", "rate", "rotates?"))
    first = None
    for stick in (0.05, 0.08, 0.10, 0.12, 0.15, 0.20, 0.30, 0.50):
        n.stick = 0.0
        spin(n, 2.5)
        y0 = n.yaw
        t0 = time.time()
        n.stick = stick
        spin(n, 4.0)
        dt = time.time() - t0
        d = math.degrees(wrap(n.yaw - y0))
        n.stick = 0.0
        spin(n, 2.5)
        rate = d / dt
        moves = abs(rate) > 2.0
        if moves and first is None:
            first = stick
        print("  %-8.2f %-14s %s"
              % (stick, "%+.1f deg/s" % rate, "yes" if moves else "NO"))

    print()
    print("  ===== CONCLUSION =====")
    if first is None:
        print("  nothing rotated it up to 0.50, something else is wrong")
    else:
        print("  smallest stick that turns the aircraft: %.2f" % first)
        print("  so the pilot must never command less than that while a turn")
        print("  is still wanted, or it stalls short of the heading.")
    n.stick = 0.0
    spin(n, 1.0)
    n.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
