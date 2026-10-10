#!/usr/bin/env python3
"""Does the pilot's velocity mode actually achieve the commanded speed?

Under Nav2 the aircraft tracked about 0.06 m/s against commands of 0.5 to
0.9 m/s. Two candidates: the stick mapping is too weak, or PX4's collision
prevention is braking. This bypasses Nav2 entirely, publishes a constant
/cmd_vel, and measures the achieved body speed with collision prevention on
and then off.

If both are slow, the mapping is wrong. If only the first is slow, the two
layers are fighting, which is the cost of keeping PX4 braking under Nav2.
"""
import math
import subprocess
import time

import rclpy
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from regression import guarded  # noqa: E402
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from geometry_msgs.msg import PoseStamped, Twist
from px4_msgs.msg import ManualControlSetpoint, VehicleLocalPosition

QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                 durability=DurabilityPolicy.VOLATILE,
                 history=HistoryPolicy.KEEP_LAST, depth=5)

WANT = 1.0      # m/s forward


def param(name, value):
    """Set a PX4 parameter; raises if px4-param fails, so a trial never
    runs on a value that was not actually set."""
    r = subprocess.run(["px4-param", "set", name, str(value)],
                       capture_output=True, text=True, timeout=20)
    if r.returncode != 0:
        raise RuntimeError('px4-param set %s %s failed: %s'
                           % (name, value, (r.stderr or r.stdout).strip()))


class AB(Node):
    def __init__(self):
        super().__init__('velmode_ab')
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pilot = self.create_publisher(PoseStamped,
                                           '/avoidance_sim/pilot_goal', 10)
        self.create_subscription(VehicleLocalPosition,
                                 '/fmu/out/vehicle_local_position_v1',
                                 self.on_pos, QOS)
        self.create_subscription(ManualControlSetpoint,
                                 '/fmu/in/manual_control_input',
                                 self.on_stick, QOS)
        self.pos = None
        self.vel = (0.0, 0.0)
        self.yaw = 0.0
        self.alt = 0.0
        self.stick = None
        self.send = False
        self.create_timer(0.1, self.tick)

    def on_pos(self, m):
        if math.isfinite(m.x):
            self.pos = (m.x, m.y)
            self.alt = -m.z
            self.yaw = m.heading
            if math.isfinite(m.vx):
                self.vel = (m.vx, m.vy)

    def on_stick(self, m):
        self.stick = (m.pitch, m.roll, m.throttle)

    def fwd(self):
        vn, ve = self.vel
        return vn * math.cos(self.yaw) + ve * math.sin(self.yaw)

    def tick(self):
        if self.send:
            t = Twist()
            t.linear.x = WANT
            self.cmd.publish(t)

    def goal(self, east, north, alt, hdg):
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
        g.pose.position.x, g.pose.position.y = float(east), float(north)
        g.pose.position.z = float(alt)
        enu = math.pi / 2.0 - math.radians(hdg)
        g.pose.orientation.w = math.cos(enu / 2.0)
        g.pose.orientation.z = math.sin(enu / 2.0)
        self.pilot.publish(g)


def spin(n, s):
    t = time.time()
    while time.time() - t < s:
        rclpy.spin_once(n, timeout_sec=0.02)


def trial(n, label, cp):
    param("CP_DIST", cp)
    spin(n, 2.0)
    n.send = False
    spin(n, 6.0)
    p0 = n.pos
    t0 = time.time()
    n.send = True
    speeds, sticks = [], []
    while time.time() - t0 < 10.0:
        rclpy.spin_once(n, timeout_sec=0.02)
        if time.time() - t0 > 3.0:          # let it accelerate
            speeds.append(n.fwd())
            if n.stick:
                sticks.append(n.stick[0])
    n.send = False
    p1 = n.pos
    spin(n, 5.0)
    travelled = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    avg = sum(speeds) / max(1, len(speeds))
    st = sum(sticks) / max(1, len(sticks))
    print("  %-22s commanded %.2f m/s -> achieved %+.2f m/s, "
          "travelled %.2f m, mean pitch stick %+.3f"
          % (label, WANT, avg, travelled, st))
    return avg, st


def main():
    rclpy.init()
    n = AB()
    # Up to 20 s for the first data. A fixed 4 s was enough on the
    # development machine and not on a 4-core one, where DDS discovery had
    # not finished and the script quit before measuring anything.
    for _ in range(40):
        spin(n, 0.5)
        if n.pos is not None:
            break
    if n.pos is None:
        print("  no position"); return 1
    print("  aircraft at alt %.2f, heading %+.0f" % (n.alt, math.degrees(n.yaw)))
    if n.alt < 4.0:
        print("  needs to be airborne above 4 m; run after a flight test")
        return 1

    # Point north, away from the near wall, so the forward direction is open.
    print("  pointing north and settling")
    n.goal(n.pos[1], n.pos[0], n.alt, 0.0)
    spin(n, 22.0)
    print("  heading now %+.0f" % math.degrees(n.yaw))

    print()
    print("  expected stick for %.2f m/s: %.3f  (0.114 + v/5.02)"
          % (WANT, 0.114 + WANT / 5.02))
    print()
    # Collision prevention is switched off for the second trial. The
    # finally puts it back whatever happens, Ctrl-C included: a script that
    # died between the two left every later test flying with no brake.
    try:
        on, st_on = trial(n, "CP on (CP_DIST 2.0)", 2.0)
        off, st_off = trial(n, "CP off (CP_DIST -1)", -1.0)
    finally:
        n.send = False
        param("CP_DIST", 2.0)
        print("  CP_DIST restored to 2.0")

    print()
    print("  ===== READING =====")
    if off > 0.6 * WANT and on < 0.4 * WANT:
        print("  The mapping is fine; PX4 collision prevention is the limiter.")
        print("  That is the cost of keeping PX4 braking underneath Nav2: it")
        print("  will not let the planner route close to obstacles.")
    elif off < 0.4 * WANT:
        print("  Slow with avoidance OFF too (%.2f m/s), so the stick mapping" % off)
        print("  is the problem, not the interaction. Mean stick was %+.3f" % st_off)
        print("  against the %.3f the model predicts." % (0.114 + WANT / 5.02))
    else:
        print("  Both reasonable: on %.2f, off %.2f m/s." % (on, off))
    n.send = False
    spin(n, 1.0)
    n.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(guarded(main))
