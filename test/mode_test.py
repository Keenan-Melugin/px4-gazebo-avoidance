#!/usr/bin/env python3
"""Does the brake/plan toggle work, and does plan mode escape collision prevention?

The point of the toggle is that avoidance belongs to exactly one layer at a
time. Brake mode is Position mode on sticks, where PX4 collision prevention is
the whole mechanism. Plan mode is Offboard, where PX4 holds no collision
prevention at all, so a planner can approach obstacles in order to route
around them.

The decisive check: CP_DIST stays at 2.0 throughout. That value previously
deadlocked the planner at 0.00 m/s against a 1.50 m/s command. If plan mode
works, the aircraft flies at the commanded speed with that value untouched.

Also checks the NED conversion. TrajectorySetpoint.velocity is the NED world
frame and cmd_vel is base_link FLU, which is the kind of rotation that
produces believable wrong numbers.
"""
import math
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from px4_msgs.msg import VehicleCommand, VehicleLocalPosition, VehicleStatus

QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                 durability=DurabilityPolicy.VOLATILE,
                 history=HistoryPolicy.KEEP_LAST, depth=5)

POSCTL, OFFBOARD = 2, 14


def param(name, value):
    subprocess.run(["px4-param", "set", name, str(value)],
                   capture_output=True, timeout=20)


class M(Node):
    def __init__(self):
        super().__init__('mode_test')
        self.mode = self.create_publisher(String, '/avoidance_sim/mode', 10)
        self.cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pilot = self.create_publisher(PoseStamped,
                                           '/avoidance_sim/pilot_goal', 10)
        self.cmd = self.create_publisher(VehicleCommand,
                                         '/fmu/in/vehicle_command', QOS)
        self.create_subscription(VehicleLocalPosition,
                                 '/fmu/out/vehicle_local_position_v1',
                                 self.on_pos, QOS)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1',
                                 self.on_st, QOS)
        self.pos = None
        self.vel = (0.0, 0.0, 0.0)
        self.yaw = 0.0
        self.alt = 0.0
        self.nav = None
        self.armed = None
        self.healthy = None
        self.drive = None
        self.create_timer(0.1, self.tick)

    def on_pos(self, m):
        if math.isfinite(m.x):
            self.pos = (m.x, m.y)
            self.alt = -m.z
            self.yaw = m.heading
            if math.isfinite(m.vx):
                self.vel = (m.vx, m.vy, m.vz)

    def on_st(self, m):
        self.nav = m.nav_state
        self.armed = (m.arming_state == 2)
        self.healthy = m.pre_flight_checks_pass

    def tick(self):
        if self.drive is not None:
            t = Twist()
            t.linear.x, t.linear.y = self.drive[0], self.drive[1]
            self.cmd_vel.publish(t)

    def set_mode(self, name):
        m = String()
        m.data = name
        self.mode.publish(m)

    def goal(self, east, north, alt, hdg):
        g = PoseStamped()
        g.header.stamp = self.get_clock().now().to_msg()
        g.header.frame_id = 'odom+yaw'
        g.pose.position.x, g.pose.position.y = float(east), float(north)
        g.pose.position.z = float(alt)
        enu = math.pi / 2.0 - math.radians(hdg)
        g.pose.orientation.w = math.cos(enu / 2.0)
        g.pose.orientation.z = math.sin(enu / 2.0)
        self.pilot.publish(g)

    def stop_pilot(self):
        g = PoseStamped()
        g.header.frame_id = 'STOP'
        self.pilot.publish(g)

    def arm(self):
        v = VehicleCommand()
        v.timestamp = self.get_clock().now().nanoseconds // 1000
        v.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        v.param1, v.param2 = 1.0, 21196.0
        v.target_system = v.target_component = 1
        v.source_system = v.source_component = 1
        v.from_external = True
        self.cmd.publish(v)

    def body_speed(self):
        vn, ve, _ = self.vel
        return (vn * math.cos(self.yaw) + ve * math.sin(self.yaw),
                -vn * math.sin(self.yaw) + ve * math.cos(self.yaw))


def spin(n, s):
    t = time.time()
    while time.time() - t < s:
        rclpy.spin_once(n, timeout_sec=0.02)


def main():
    rclpy.init()
    n = M()
    spin(n, 4.0)
    if n.pos is None:
        print("  no position"); return 1

    param("CP_DIST", 2.0)
    param("CP_GO_NO_DATA", 1)
    print("  CP_DIST 2.0 for the whole test, deliberately never changed")

    n.set_mode('brake')
    n.stop_pilot()
    spin(n, 5.0)
    if not n.armed:
        for _ in range(20):
            if n.healthy:
                n.arm(); spin(n, 2.0)
                if n.armed:
                    break
            spin(n, 3.0)
    if not n.armed:
        print("  could not arm"); return 1

    print("  climbing, nose north, well clear of the walls")
    n.goal(-2.0, -14.0, 9.0, 0.0)
    for _ in range(60):
        spin(n, 1.0)
        if n.alt > 8.0 and abs(n.pos[0] + 14) < 2.5:
            break
    print("  at north %+.2f east %+.2f alt %.2f heading %+.0f"
          % (n.pos[0], n.pos[1], n.alt, math.degrees(n.yaw)))
    if n.alt < 6.0:
        print("  ABORT: not airborne"); return 2

    fails = []

    print()
    print("  === 1. brake mode should be Position mode ===")
    n.set_mode('brake')
    spin(n, 6.0)
    print("     nav_state %s (want %d POSCTL)" % (n.nav, POSCTL))
    if n.nav != POSCTL:
        fails.append("brake mode did not give POSCTL")

    print()
    print("  === 2. plan mode should become Offboard ===")
    n.set_mode('plan')
    n.drive = (0.0, 0.0)
    spin(n, 10.0)
    print("     nav_state %s (want %d OFFBOARD)" % (n.nav, OFFBOARD))
    if n.nav != OFFBOARD:
        fails.append("plan mode did not give OFFBOARD")

    print()
    print("  === 3. plan mode must fly despite CP_DIST 2.0 ===")
    print("     (this is the value that deadlocked the planner at 0.00 m/s)")
    n.drive = (1.0, 0.0)
    spin(n, 6.0)
    sp = []
    t = time.time()
    while time.time() - t < 8.0:
        rclpy.spin_once(n, timeout_sec=0.02)
        sp.append(n.body_speed()[0])
    fwd = sum(sp) / max(1, len(sp))
    print("     commanded 1.00 m/s forward -> achieved %+.2f m/s" % fwd)
    if fwd < 0.5:
        fails.append("plan mode achieved only %.2f m/s" % fwd)

    print()
    print("  === 4. the NED conversion: command right, check it goes right ===")
    n.drive = (0.0, -1.0)      # FLU y negative = right
    spin(n, 6.0)
    sp = []
    t = time.time()
    while time.time() - t < 8.0:
        rclpy.spin_once(n, timeout_sec=0.02)
        sp.append(n.body_speed()[1])
    right = sum(sp) / max(1, len(sp))
    print("     commanded 1.00 m/s RIGHT -> body-right %+.2f m/s" % right)
    if right < 0.4:
        fails.append("lateral velocity wrong: %.2f (expected positive)" % right)

    print()
    print("  === 5. toggle back to brake ===")
    n.drive = None
    n.set_mode('brake')
    spin(n, 8.0)
    print("     nav_state %s (want %d POSCTL)" % (n.nav, POSCTL))
    if n.nav != POSCTL:
        fails.append("did not return to POSCTL")

    print()
    print("  ===== %s =====" % ("ALL PASS" if not fails else "FAILURES"))
    for f in fails:
        print("    - %s" % f)
    n.drive = None
    spin(n, 1.0)
    n.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
