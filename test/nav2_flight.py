#!/usr/bin/env python3
"""Does Nav2 route the aircraft AROUND a wall, rather than stopping at it?

The scenario is box2 in walls.sdf: a wall running east-west at north +4.5 to
+5.5, spanning east -8 to +2. Starting south of it and asking for a goal north
of it means the only way through is around its east end at east +2.

The base stack cannot do this: collision prevention brakes and the aircraft
sits in front of the wall. A path planner should go around.

Uses the /navigate_to_pose action directly rather than /goal_pose, because
/goal_pose is also consumed by this package's own GoalBridge, which sends a
PX4 reposition with no avoidance. Two different things would react to one
message.
"""
import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path
from px4_msgs.msg import VehicleCommand, VehicleLocalPosition, VehicleStatus

QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                 durability=DurabilityPolicy.VOLATILE,
                 history=HistoryPolicy.KEEP_LAST, depth=5)

START = (-2.0, 0.0)      # east, north: south of box2, within its east-west span
GOAL = (-2.0, 10.0)      # east, north: north of box2
WALL_NORTH = 4.5         # box2's south face
ALT = 8.0


class N2(Node):
    def __init__(self):
        super().__init__('nav2_flight')
        self.pilot = self.create_publisher(PoseStamped,
                                           '/avoidance_sim/pilot_goal', 10)
        self.cmd = self.create_publisher(VehicleCommand,
                                         '/fmu/in/vehicle_command', QOS)
        self.create_subscription(VehicleLocalPosition,
                                 '/fmu/out/vehicle_local_position_v1',
                                 self.on_pos, QOS)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1',
                                 self.on_st, QOS)
        self.create_subscription(Path, '/plan', self.on_plan, 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.ac = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.pos = None
        self.yaw = 0.0
        self.armed = None
        self.healthy = None
        self.plan_len = None
        self.plan_n = 0
        self.cmds = 0
        self.last_cmd = None
        self.track = []

    def on_pos(self, m):
        if math.isfinite(m.x):
            self.pos = (m.x, m.y, -m.z)
            self.yaw = m.heading

    def on_st(self, m):
        self.armed = (m.arming_state == 2)
        self.healthy = m.pre_flight_checks_pass

    def on_plan(self, m):
        self.plan_n += 1
        if len(m.poses) > 1:
            d = 0.0
            for a, b in zip(m.poses, m.poses[1:]):
                d += math.hypot(b.pose.position.x - a.pose.position.x,
                                b.pose.position.y - a.pose.position.y)
            self.plan_len = d

    def on_cmd(self, m):
        self.cmds += 1
        self.last_cmd = (m.linear.x, m.linear.y, m.angular.z)

    def goal_pilot(self, east, north, alt, hdg=None):
        g = PoseStamped()
        g.header.stamp = self.get_clock().now().to_msg()
        g.header.frame_id = 'odom' if hdg is None else 'odom+yaw'
        g.pose.position.x, g.pose.position.y = float(east), float(north)
        g.pose.position.z = float(alt)
        enu = math.pi / 2.0 - math.radians(hdg or 0.0)
        g.pose.orientation.w = math.cos(enu / 2.0)
        g.pose.orientation.z = math.sin(enu / 2.0)
        self.pilot.publish(g)

    def stop_pilot(self):
        g = PoseStamped()
        g.header.stamp = self.get_clock().now().to_msg()
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


def spin(n, s):
    t = time.time()
    while time.time() - t < s:
        rclpy.spin_once(n, timeout_sec=0.05)


def main():
    rclpy.init()
    n = N2()
    spin(n, 4.0)
    if n.pos is None:
        print("  no position"); return 1

    n.stop_pilot()
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

    print("  positioning at east %.1f north %.1f alt %.1f, facing north"
          % (START[0], START[1], ALT))
    n.goal_pilot(START[0], START[1], ALT, 0.0)
    for _ in range(40):
        spin(n, 1.0)
        if (abs(n.pos[1] - START[0]) < 1.2 and abs(n.pos[0] - START[1]) < 1.2
                and n.pos[2] > ALT - 1.5):
            break
    print("  at east %+.2f north %+.2f alt %.2f heading %+.0f"
          % (n.pos[1], n.pos[0], n.pos[2], math.degrees(n.yaw)))

    # Enforce, do not merely report. An earlier version printed the position
    # and carried on, and then claimed PASS because the aircraft was already
    # 29 m past the wall before the goal was even sent.
    bad = []
    if abs(n.pos[1] - START[0]) > 2.0:
        bad.append("east %+.2f, wanted %+.1f" % (n.pos[1], START[0]))
    if abs(n.pos[0] - START[1]) > 2.0:
        bad.append("north %+.2f, wanted %+.1f" % (n.pos[0], START[1]))
    if n.pos[0] > WALL_NORTH:
        bad.append("already past the wall")
    if n.pos[2] < ALT - 2.0:
        bad.append("alt %.2f, wanted %.1f. Below ~3 m the costmap slab "
                   "includes the ground, which fills it with obstacles"
                   % (n.pos[2], ALT))
    if bad:
        print()
        print("  ABORT: preconditions not met, so any result would be")
        print("         meaningless:")
        for b in bad:
            print("           - %s" % b)
        print("         The aircraft is probably in a degraded state. Restart")
        print("         PX4 and Gazebo, then rerun.")
        return 2

    print("  letting the camera build the costmap")
    spin(n, 8.0)

    print()
    print("  waiting for the navigate_to_pose action server")
    if not n.ac.wait_for_server(timeout_sec=15.0):
        print("  action server never appeared"); return 1

    g = NavigateToPose.Goal()
    g.pose.header.frame_id = 'odom'
    g.pose.header.stamp = n.get_clock().now().to_msg()
    g.pose.pose.position.x = float(GOAL[0])
    g.pose.pose.position.y = float(GOAL[1])
    g.pose.pose.orientation.w = 1.0
    print("  sending Nav2 goal: east %.1f north %.1f (wall at north %.1f)"
          % (GOAL[0], GOAL[1], WALL_NORTH))
    fut = n.ac.send_goal_async(g)
    t = time.time()
    while not fut.done() and time.time() - t < 15:
        rclpy.spin_once(n, timeout_sec=0.1)
    if not fut.done():
        print("  goal was never accepted"); return 1
    gh = fut.result()
    if not gh.accepted:
        print("  Nav2 REJECTED the goal"); return 1
    print("  accepted")

    res = gh.get_result_async()
    t0 = time.time()
    crossed = False
    while time.time() - t0 < 110:
        rclpy.spin_once(n, timeout_sec=0.1)
        if n.pos:
            n.track.append((n.pos[1], n.pos[0]))
            if n.pos[0] > WALL_NORTH + 1.0:
                crossed = True
        if int(time.time() - t0) % 6 == 0:
            time.sleep(0.25)
            c = n.last_cmd or (0, 0, 0)
            print("    t+%3ds east %+6.2f north %+6.2f | plans %2d len %s | "
                  "cmd_vel %4d fwd%+5.2f left%+5.2f"
                  % (time.time() - t0, n.pos[1], n.pos[0], n.plan_n,
                     ("%.1f m" % n.plan_len) if n.plan_len else "none",
                     n.cmds, c[0], c[1]))
        if res.done():
            break

    print()
    print("  ===== RESULT =====")
    print("  plans produced:      %d" % n.plan_n)
    print("  cmd_vel messages:    %d" % n.cmds)
    print("  final east %+.2f north %+.2f" % (n.pos[1], n.pos[0]))
    east_excursion = max(abs(e - START[0]) for e, _ in n.track) if n.track else 0
    print("  furthest sideways excursion from the start line: %.2f m"
          % east_excursion)
    if n.plan_n == 0:
        print("  FAIL: the planner never produced a path. If the log says")
        print("        'unknown', the costmap has not seen enough and")
        print("        allow_unknown must be true.")
    elif n.cmds == 0:
        print("  FAIL: paths planned but no cmd_vel, so the controller is not")
        print("        following them.")
    elif crossed:
        print("  PASS: got past the wall at north %.1f, which the base "
              "stack cannot do." % WALL_NORTH)
        print("        Sideways excursion %.1f m, so it routed around rather "
              "than through." % east_excursion)
    else:
        print("  PARTIAL: Nav2 planned and drove, but never got past north")
        print("           %.1f. Either the detour is longer than the time" % WALL_NORTH)
        print("           allowed, or it is stuck against the wall.")
    n.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
