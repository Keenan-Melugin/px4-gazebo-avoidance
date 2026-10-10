#!/usr/bin/env python3
"""Does Nav2 route the aircraft AROUND a wall, rather than stopping at it?

The default scenario is box2 in walls.sdf: a wall running east-west at north +4.5 to
+5.5, spanning east -8 to +2. Starting south of it and asking for a goal north
of it means the only way through is around its east end at east +2.

The base stack cannot do this: collision prevention brakes and the aircraft
sits in front of the wall. A path planner should go around.

Uses the /navigate_to_pose action directly rather than /goal_pose, because
/goal_pose is also consumed by this package's own GoalBridge, which sends a
PX4 reposition with no avoidance. Two different things would react to one
message.
"""
import argparse
import math
import time

import rclpy
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from regression import guarded, require_modelled, route_to  # noqa: E402
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path
from px4_msgs.msg import VehicleCommand, VehicleLocalPosition, VehicleStatus

from avoidance_sim import world_geometry

QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                 durability=DurabilityPolicy.VOLATILE,
                 history=HistoryPolicy.KEEP_LAST, depth=5)
# Must match the pilot's subscriber. A VOLATILE publisher to a TRANSIENT_LOCAL
# subscriber is not a QoS mismatch warning, it is silence.
MODE_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL,
                      history=HistoryPolicy.KEEP_LAST, depth=1)

# Standoff matters, and it is geometry rather than taste. box2 is 10 m wide
# (east -8 to +2) and the camera sees a 73 degree arc, so at range R it covers
# 2*R*tan(36.5) = 1.48*R of width. To see a 10 m obstacle AND both of its ends
# the aircraft has to observe from about 10 m back; from 3.5 m it sees 5 m of
# wall and no ends, so the planner can never find a way round.
START = (-2.0, -9.0)     # east, north: 13.5 m south of box2's face. --start
GOAL = (-2.0, 10.0)      # east, north: north of box2. --goal
ALT = 8.0                # --alt
# The wall itself comes from the world file (--world): the first box north of
# the start on the start's east line, through the same parser RViz's wall
# markers use. For walls that is box2's south face at north +4.5.


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
        self.mode_pub = self.create_publisher(String, '/avoidance_sim/mode',
                                              MODE_QOS)
        self.nav = None
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
        self.nav = m.nav_state

    def set_mode(self, name):
        m = String()
        m.data = name
        self.mode_pub.publish(m)

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

    def speed(self):
        """Ground speed over the last ~2 s of track, by wall clock.

        The first version divided by an assumed 2.0 s for 20 samples; the
        sample interval is not fixed, so it read 0.05 m/s while the aircraft
        covered 9 m in 5 s. Timestamps now travel with the samples.
        """
        if len(self.track) < 2:
            return 0.0
        now = self.track[-1][2]
        i = len(self.track) - 1
        while i > 0 and now - self.track[i][2] < 2.0:
            i -= 1
        a, b = self.track[i], self.track[-1]
        dt = b[2] - a[2]
        return math.hypot(b[0] - a[0], b[1] - a[1]) / dt if dt > 0.2 else 0.0

    def goal_pilot(self, east, north, alt, hdg=None):
        # Wait for the pilot to be connected before the first goal. A goal
        # published on a new publisher before discovery completes is lost
        # without a trace: measured 2026-10-10, the heading test's first
        # command never reached the pilot in two runs of three.
        if not getattr(self, '_goal_matched', False):
            t0 = time.time()
            while self.pilot.get_subscription_count() == 0 and time.time() - t0 < 10.0:
                rclpy.spin_once(self, timeout_sec=0.1)
            self._goal_matched = True
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
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--world', default='walls',
                    help='world name (as PX4_GZ_WORLD) or path to its .sdf')
    ap.add_argument('--start', nargs=2, type=float, metavar=('EAST', 'NORTH'),
                    default=START, help='where to position first')
    ap.add_argument('--goal', nargs=2, type=float, metavar=('EAST', 'NORTH'),
                    default=GOAL, help='the Nav2 goal, beyond the wall')
    ap.add_argument('--alt', type=float, default=ALT)
    a = ap.parse_args()
    start, goal, alt = tuple(a.start), tuple(a.goal), a.alt
    world = world_geometry.resolve_world(a.world)
    require_modelled(world)
    boxes = world_geometry.load_boxes(world)
    hit = world_geometry.first_face_ahead(boxes, start[0], start[1], alt, 'north')
    if hit is None:
        print("  no box north of east %+.1f north %+.1f at %.1f m in %s: nothing"
              " to route around. Pick a start with a wall ahead of it."
              % (start[0], start[1], alt, a.world))
        return 2
    wall_north, wall = hit
    print("  world %s: %s, near face at north %+.1f, spanning east %+.1f..%+.1f,"
          " %.1f m thick" % (world_geometry.world_name(world), wall.name,
                              wall_north, wall.x_min, wall.x_max, wall.sy))
    if goal[1] <= wall.y_max:
        print("  the goal (north %+.1f) is not beyond the wall's far face (north"
              " %+.1f)" % (goal[1], wall.y_max))
        return 2

    rclpy.init()
    n = N2()
    try:
        return fly(n, start, goal, alt, wall_north, wall, boxes)
    finally:
        # Every exit, including an ABORT, a timeout and Ctrl-C, leaves nothing
        # driving. A Nav2 goal left running kept publishing /cmd_vel after
        # this script ended, and the next script's goals were overridden by
        # it: the late-session failures that moved between scripts.
        cleanup(n)
        n.destroy_node()
        rclpy.try_shutdown()


def cancel_all(n, wait=3.0):
    """Cancel every Nav2 navigate_to_pose goal. True if the request was sent."""
    try:
        from action_msgs.srv import CancelGoal
        cli = n.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        if cli.wait_for_service(timeout_sec=5.0):
            cli.call_async(CancelGoal.Request())     # blank request = cancel all
            spin(n, wait)
            return True
    except Exception as exc:
        print("  could not cancel: %s" % exc)
    return False


def cleanup(n):
    print("  cleanup: cancelling Nav2 goals, back to BRAKE, STOP")
    cancel_all(n)
    n.set_mode('brake')
    spin(n, 2.0)
    n.stop_pilot()
    spin(n, 2.0)


def fly(n, start, goal, alt, wall_north, wall, boxes):
    # Up to 20 s for the first position. A fixed 4 s was enough on the
    # development machine and not on a 4-core one, where DDS discovery had not
    # finished and the test quit with "no position".
    for _ in range(40):
        spin(n, 0.5)
        if n.pos is not None:
            break
    if n.pos is None:
        print("  no position after 20 s"); return 1

    # Cancel anything Nav2 is still driving. Without this a goal left running
    # from a previous attempt keeps publishing cmd_vel, the pilot stays in
    # velocity mode, and the repositioning goal below is silently ignored.
    if n.ac.wait_for_server(timeout_sec=10.0) and cancel_all(n):
        print("  cancelled any running Nav2 goal")
    n.set_mode('brake')
    spin(n, 1.0)
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

    # Reposition facing the way it goes, then turn north. The camera sees 73
    # degrees ahead only, so flying to the start facing north flew blind:
    # measured 2026-10-10, from east 100 it flew west into a wall facing
    # north and turned the aircraft over.
    print("  positioning at east %.1f north %.1f alt %.1f, facing the route"
          % (start[0], start[1], alt))
    # By a route round the world's boxes. A straight line from where the gate
    # leaves the aircraft (north 10) to the start passes through box2.
    if not route_to(start, alt, boxes):
        print("  ABORT: could not reach the start by a clear route")
        return 2
    n.goal_pilot(start[0], start[1], alt, 0.0)
    for _ in range(20):
        spin(n, 1.0)
        if abs(math.degrees(n.yaw)) < 10.0:
            break
    print("  at east %+.2f north %+.2f alt %.2f heading %+.0f"
          % (n.pos[1], n.pos[0], n.pos[2], math.degrees(n.yaw)))

    # Enforce, do not merely report. An earlier version printed the position
    # and carried on, and then claimed PASS because the aircraft was already
    # 29 m past the wall before the goal was even sent.
    bad = []
    if abs(n.pos[1] - start[0]) > 2.0:
        bad.append("east %+.2f, wanted %+.1f" % (n.pos[1], start[0]))
    if abs(n.pos[0] - start[1]) > 2.0:
        bad.append("north %+.2f, wanted %+.1f" % (n.pos[0], start[1]))
    if n.pos[0] > wall_north:
        bad.append("already past the wall")
    if n.pos[2] < alt - 2.0:
        bad.append("alt %.2f, wanted %.1f. Below ~3 m the costmap slab "
                   "includes the ground, which fills it with obstacles"
                   % (n.pos[2], alt))
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
    g.pose.pose.position.x = float(goal[0])
    g.pose.pose.position.y = float(goal[1])
    g.pose.pose.orientation.w = 1.0
    # Positioning used brake mode on purpose: the pilot's own goal flying is
    # the reliable way to get into place. Planning needs plan mode, which is
    # Offboard, where PX4 has no collision prevention to veto the planner.
    print("  switching to PLAN mode (Offboard)")
    n.set_mode('plan')
    for _ in range(40):
        spin(n, 0.5)
        if n.nav == 14:
            break
    print("  nav_state %s (14 = OFFBOARD)" % n.nav)
    if n.nav != 14:
        print("  ABORT: plan mode did not reach Offboard, so this would measure")
        print("         the stick path with collision prevention live, which is")
        print("         the deadlock already recorded.")
        return 2

    print("  sending Nav2 goal: east %.1f north %.1f (wall at north %.1f)"
          % (goal[0], goal[1], wall_north))
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
    while time.time() - t0 < 300:
        rclpy.spin_once(n, timeout_sec=0.1)
        if n.pos:
            n.track.append((n.pos[1], n.pos[0], time.time()))
            if n.pos[0] > wall.y_max:
                crossed = True
        if int(time.time() - t0) % 6 == 0:
            time.sleep(0.25)
            c = n.last_cmd or (0, 0, 0)
            print("    t+%3ds east %+6.2f north %+6.2f | plans %2d len %s | "
                  "cmd_vel %4d fwd%+5.2f left%+5.2f | %.2f m/s actual"
                  % (time.time() - t0, n.pos[1], n.pos[0], n.plan_n,
                     ("%.1f m" % n.plan_len) if n.plan_len else "none",
                     n.cmds, c[0], c[1], n.speed()))
        if res.done():
            break

    print()
    print("  ===== RESULT =====")
    print("  plans produced:      %d" % n.plan_n)
    print("  cmd_vel messages:    %d" % n.cmds)
    print("  final east %+.2f north %+.2f" % (n.pos[1], n.pos[0]))
    east_excursion = max(abs(p[0] - start[0]) for p in n.track) if n.track else 0
    print("  furthest sideways excursion from the start line: %.2f m"
          % east_excursion)
    rc = 1
    if n.plan_n == 0:
        print("  FAIL: the planner never produced a path. If the log says")
        print("        'unknown', the costmap has not seen enough and")
        print("        allow_unknown must be true.")
    elif n.cmds == 0:
        print("  FAIL: paths planned but no cmd_vel, so the controller is not")
        print("        following them.")
    elif crossed and math.hypot(n.pos[1] - goal[0], n.pos[0] - goal[1]) > 4.0:
        # Crossing the wall line is not enough. Measured once with the lidar
        # model: the controller hugged the wall's face at 0.1 m, the rotors
        # touched it, and the tumbling aircraft crossed north 5.5 on its way
        # to the ground 78 m away. That printed PASS.
        print("  FAIL: crossed the wall line but ended %.1f m from the goal, at"
              " %.1f m altitude. A crash or a runaway, not a route."
              % (math.hypot(n.pos[1] - goal[0], n.pos[0] - goal[1]), n.pos[2]))
    elif crossed:
        rc = 0
        print("  PASS: got past the wall at north %.1f and ended %.1f m from the"
              " goal, which the base stack cannot do."
              % (wall_north, math.hypot(n.pos[1] - goal[0], n.pos[0] - goal[1])))
        print("        Sideways excursion %.1f m, so it routed around rather "
              "than through." % east_excursion)
    else:
        print("  PARTIAL: Nav2 planned and drove, but never got past north")
        print("           %.1f. Either the detour is longer than the time" % wall_north)
        print("           allowed, or it is stuck against the wall.")
    print()
    return rc


if __name__ == '__main__':
    raise SystemExit(guarded(main))
