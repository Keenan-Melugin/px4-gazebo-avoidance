#!/usr/bin/env python3
"""Does the XY stick have a dead band, and what velocity does it command?

The yaw stick turned out to have one at about 0.10, which broke proportional
heading control. Nav2 will ask for slow velocities on final approach, and if
those map into a dead band the aircraft will stall short of every goal in
exactly the same way. Worth knowing before writing the mapping rather than
after debugging it.

Also measures the stick-to-velocity slope, so the conversion is fitted to
this airframe rather than assumed from MPC_VEL_MANUAL.
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from px4_msgs.msg import ManualControlSetpoint, VehicleLocalPosition

QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                 durability=DurabilityPolicy.VOLATILE,
                 history=HistoryPolicy.KEEP_LAST, depth=5)


class P(Node):
    def __init__(self):
        super().__init__('xy_threshold')
        self.pub = self.create_publisher(ManualControlSetpoint,
                                         '/fmu/in/manual_control_input', QOS)
        self.create_subscription(VehicleLocalPosition,
                                 '/fmu/out/vehicle_local_position_v1',
                                 self.on_pos, QOS)
        self.pitch = 0.0
        self.thr = 0.0
        self.pos = None
        self.vel = (0.0, 0.0)
        self.yaw = 0.0
        self.alt = 0.0
        self.create_timer(0.02, self.tick)

    def on_pos(self, m):
        if math.isfinite(m.x):
            self.pos = (m.x, m.y)
            self.alt = -m.z
            self.yaw = m.heading
            if math.isfinite(m.vx):
                self.vel = (m.vx, m.vy)

    def fwd_speed(self):
        """Speed along the current heading, which is what pitch commands."""
        vn, ve = self.vel
        return vn * math.cos(self.yaw) + ve * math.sin(self.yaw)

    def tick(self):
        m = ManualControlSetpoint()
        m.timestamp = self.get_clock().now().nanoseconds // 1000
        m.timestamp_sample = m.timestamp
        m.valid = True
        m.data_source = ManualControlSetpoint.SOURCE_MAVLINK_0
        m.roll = m.yaw = 0.0
        m.pitch = self.pitch
        m.throttle = self.thr
        self.pub.publish(m)


def spin(n, s):
    t = time.time()
    while time.time() - t < s:
        rclpy.spin_once(n, timeout_sec=0.05)


def main():
    rclpy.init()
    n = P()
    spin(n, 4.0)
    if n.pos is None:
        print("  no position"); return 1
    print("  airborne at %.1f m" % n.alt)
    if n.alt < 4.0:
        print("  NOT AIRBORNE, run this with the aircraft flying"); return 1

    print()
    print("  %-8s %-18s %s" % ("pitch", "steady speed", "moves?"))
    rows = []
    for stick in (0.02, 0.04, 0.06, 0.08, 0.10, 0.15, 0.25, 0.40):
        n.pitch = 0.0
        spin(n, 5.0)
        n.pitch = stick
        # let it accelerate to a steady speed, then sample
        spin(n, 5.0)
        samples = []
        t = time.time()
        while time.time() - t < 3.0:
            rclpy.spin_once(n, timeout_sec=0.05)
            samples.append(n.fwd_speed())
        v = sum(samples) / max(1, len(samples))
        n.pitch = 0.0
        spin(n, 5.0)
        moves = abs(v) > 0.08
        rows.append((stick, v))
        print("  %-8.2f %-18s %s" % (stick, "%+.3f m/s" % v,
                                     "yes" if moves else "NO"))

    print()
    print("  ===== FIT =====")
    live = [(s, v) for s, v in rows if abs(v) > 0.08]
    if len(live) >= 2:
        # least squares through the moving points
        xs = [s for s, _ in live]
        ys = [v for _, v in live]
        nn = len(xs)
        mx, my = sum(xs) / nn, sum(ys) / nn
        num = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
        den = sum((x - mx) ** 2 for x in xs)
        slope = num / den if den else 0.0
        icpt = my - slope * mx
        dz = -icpt / slope if slope else 0.0
        print("  speed = (stick - %.3f) * %.2f m/s per unit" % (dz, slope))
        print("  dead band edge: about %.3f" % dz)
        print("  full stick would be %.1f m/s (MPC_VEL_MANUAL says 10.0)"
              % (slope * (1.0 - dz)))
        smallest = min(s for s, _ in live)
        print("  smallest stick that moved it: %.2f" % smallest)
        if dz > 0.02:
            print("  -> the mapping must offset past this, like yaw does")
        else:
            print("  -> no meaningful dead band, a plain linear map is fine")
    else:
        print("  not enough moving points to fit")

    n.pitch = 0.0
    spin(n, 1.0)
    n.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
