#!/usr/bin/env python3
"""Is /odom's twist actually in base_link FLU, as nav_msgs requires?

PX4 reports velocity in the NED world frame, so it gets rotated into the body
before publishing. That rotation is the kind that looks plausible and is
wrong, so this measures it: hold a known heading, fly a known direction, and
check the sign of the body-frame velocity.

FLU means x forward, y LEFT, z up. So with the nose north:
    flying north -> twist.linear.x positive
    flying east  -> twist.linear.y NEGATIVE (east is to the right, not left)
    climbing     -> twist.linear.z positive
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleCommand, VehicleLocalPosition, VehicleStatus

QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                 durability=DurabilityPolicy.VOLATILE,
                 history=HistoryPolicy.KEEP_LAST, depth=5)


class T(Node):
    def __init__(self):
        super().__init__('twist_check')
        self.goal = self.create_publisher(PoseStamped,
                                          '/avoidance_sim/pilot_goal', 10)
        self.cmd = self.create_publisher(VehicleCommand,
                                         '/fmu/in/vehicle_command', QOS)
        self.create_subscription(VehicleLocalPosition,
                                 '/fmu/out/vehicle_local_position_v1',
                                 self.on_pos, QOS)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1',
                                 self.on_st, QOS)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.pos = None
        self.yaw = 0.0
        self.armed = None
        self.healthy = None
        self.twist = None

    def on_pos(self, m):
        if math.isfinite(m.x):
            self.pos = (m.x, m.y, -m.z)
            self.yaw = m.heading

    def on_st(self, m):
        self.armed = (m.arming_state == 2)
        self.healthy = m.pre_flight_checks_pass

    def on_odom(self, m):
        t = m.twist.twist.linear
        self.twist = (t.x, t.y, t.z)

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

    def stop(self):
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


def peak(n, secs):
    """Largest-magnitude body velocity seen over the window."""
    best = (0.0, 0.0, 0.0)
    t = time.time()
    while time.time() - t < secs:
        rclpy.spin_once(n, timeout_sec=0.05)
        if self_t := n.twist:
            if max(abs(v) for v in self_t) > max(abs(v) for v in best):
                best = self_t
    return best


def main():
    rclpy.init()
    n = T()
    spin(n, 4.0)
    if n.pos is None or n.twist is None:
        print("  no position or no /odom"); return 1

    n.stop()
    spin(n, 4.0)
    if not n.armed:
        for _ in range(20):
            if n.healthy:
                n.arm(); spin(n, 2.0)
                if n.armed:
                    break
            spin(n, 3.0)
    if not n.armed:
        print("  could not arm"); return 1

    east, north = n.pos[1], n.pos[0]
    print("  climbing and pointing north")
    n.send(east, north, 8.0, 0.0)
    for _ in range(25):
        spin(n, 1.0)
        if n.pos[2] > 7.0 and abs(math.degrees(n.yaw)) < 10:
            break
    print("  at alt %.1f heading %+.1f" % (n.pos[2], math.degrees(n.yaw)))
    spin(n, 4.0)

    results = {}
    trials = (("north (forward)", east, north + 12.0, 8.0),
              ("east (right)", east + 12.0, north, 8.0),
              ("up", east, north, 14.0))
    for label, ge, gn, ga in trials:
        n.send(east, north, 8.0, 0.0)
        spin(n, 12.0)
        n.send(ge, gn, ga, 0.0)
        v = peak(n, 9.0)
        results[label] = v
        print("  %-16s body twist x %+6.2f  y %+6.2f  z %+6.2f"
              % (label, v[0], v[1], v[2]))
        n.send(east, north, 8.0, 0.0)
        spin(n, 10.0)

    print()
    print("  ===== VERDICT =====")
    ok = True
    fx = results["north (forward)"][0]
    ry = results["east (right)"][1]
    uz = results["up"][2]
    for label, got, want, why in (
            ("forward -> +x", fx, 1, "flying north with the nose north is body forward"),
            ("right -> -y", ry, -1, "FLU y is LEFT, so flying right must be negative"),
            ("up -> +z", uz, 1, "FLU z is up")):
        good = (got > 0.3) if want > 0 else (got < -0.3)
        ok = ok and good
        print("  %-14s %+6.2f  %s   (%s)"
              % (label, got, "ok" if good else "WRONG SIGN", why))
    print()
    print("  %s" % ("twist is correctly in base_link FLU"
                    if ok else "TWIST FRAME IS WRONG, Nav2 would mis-plan"))
    n.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
