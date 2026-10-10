#!/usr/bin/env python3
"""The brake-mode half of the regression gate: did a change alter how it flies?

Two measurements, the same two the README numbers come from: the aircraft
holds a commanded heading, and collision prevention stops it short of a wall
at CP_DIST. Run it after any change to the pilot, the frames or the obstacle
node; if either number moves, the change did something it was not meant to.

It puts the pilot in brake mode first, so it is a valid gate whatever the last
script left the stack in. Its node class R is the one to reuse for a new
measurement; template_measure.py shows how. The plan-mode half is nav2_flight.py; gate.py runs
both and keeps score.

Picks the wall by position rather than assuming one, which is the mistake the
first version of this test made, and reads the walls from the world file
(--world, default walls) rather than carrying their coordinates. The exit
code is the number of failures.
"""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus, VehicleCommand
from std_msgs.msg import String

from avoidance_sim import world_geometry

QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                 durability=DurabilityPolicy.VOLATILE,
                 history=HistoryPolicy.KEEP_LAST, depth=5)
# Same profile as avoidance_sim/frames.py MODE_QOS. The mode is retained state,
# and a VOLATILE publisher would not match the pilot's subscriber at all.
MODE_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL,
                      history=HistoryPolicy.KEEP_LAST, depth=1)

# The wall ahead comes from the world file (--world), through the same parser
# RViz's wall markers use, so the test and the picture always agree.
CP_DIST = 2.0
NAV_POSCTL = 2


def bearing(e0, n0, e1, n1):
    """Compass bearing in degrees from (e0, n0) to (e1, n1): 0 north, 90 east.

    For repositioning: the camera sees 73 degrees ahead and nothing else, and
    CP_GO_NO_DATA 1 lets PX4 fly into what it cannot see, so a reposition must
    face the way it goes or it flies blind. Measured 2026-10-10: a reposition
    facing north while flying west flew into a wall and turned the aircraft
    over.
    """
    if abs(e1 - e0) < 0.5 and abs(n1 - n0) < 0.5:
        return 0.0
    return math.degrees(math.atan2(e1 - e0, n1 - n0))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class R(Node):
    def __init__(self):
        super().__init__('regression')
        self.goal = self.create_publisher(PoseStamped, '/avoidance_sim/pilot_goal', 10)
        self.mode = self.create_publisher(String, '/avoidance_sim/mode', MODE_QOS)
        self.cmd = self.create_publisher(VehicleCommand,
                                         '/fmu/in/vehicle_command', QOS)
        self.create_subscription(VehicleLocalPosition,
                                 '/fmu/out/vehicle_local_position_v1',
                                 self.on_pos, QOS)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1',
                                 self.on_st, QOS)
        self.pos = None
        self.yaw = 0.0
        self.arm = None
        self.nav = None

    def on_pos(self, m):
        if math.isfinite(m.x):
            self.pos = (m.x, m.y, -m.z)
            self.yaw = m.heading

    def on_st(self, m):
        self.arm = m.arming_state
        self.nav = m.nav_state

    def send(self, east, north, alt, hdg=None):
        # Wait for the pilot to be connected before the first goal. A goal
        # published on a new publisher before discovery completes is lost
        # without a trace: measured 2026-10-10, the heading test's first
        # command never reached the pilot in two runs of three.
        if not getattr(self, '_goal_matched', False):
            t0 = time.time()
            while self.goal.get_subscription_count() == 0 and time.time() - t0 < 10.0:
                rclpy.spin_once(self, timeout_sec=0.1)
            self._goal_matched = True
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

    def vcmd(self, c, p1=0.0, p2=0.0):
        v = VehicleCommand()
        v.timestamp = self.get_clock().now().nanoseconds // 1000
        v.command = c
        v.param1, v.param2 = p1, p2
        v.target_system = v.target_component = 1
        v.source_system = v.source_component = 1
        v.from_external = True
        self.cmd.publish(v)


def spin(n, s):
    t = time.time()
    while time.time() - t < s:
        rclpy.spin_once(n, timeout_sec=0.05)


def set_pilot_mode(mode):
    """Publish a pilot mode with the retained QoS the pilot subscribes with."""
    n = R()
    m = String()
    m.data = mode
    spin(n, 1.0)
    n.mode.publish(m)
    spin(n, 1.5)
    n.destroy_node()


def ensure_airborne(alt=7.0, goto=None):
    """Arm, take off and hold, from whatever state the last script left.

    The single-purpose scripts measure things in flight and used to assume
    the aircraft was already up, which on a fresh stack meant no measurement.
    Call this after rclpy.init(). It uses its own node and destroys it, so the
    calling script's node is untouched. `goto` is an optional (east, north) to
    fly to first, at `alt`, in brake mode with collision prevention live.
    Returns the (north, east, alt) it ended at, or None if it could not arm.
    """
    n = R()
    for _ in range(40):
        spin(n, 0.5)
        if n.pos is not None:
            break
    if n.pos is None:
        n.destroy_node()
        return None
    m = String()
    m.data = 'brake'
    n.mode.publish(m)
    spin(n, 2.0)
    if n.arm != 2:
        stop = PoseStamped()
        stop.header.frame_id = 'STOP'          # centre the sticks so PX4 will arm
        n.goal.publish(stop)
        spin(n, 2.0)
        for _ in range(20):
            n.vcmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 21196.0)
            spin(n, 2.0)
            if n.arm == 2:
                break
        if n.arm != 2:
            n.destroy_node()
            return None
    east, north = n.pos[1], n.pos[0]
    n.send(east, north, alt)
    for _ in range(30):
        spin(n, 1.0)
        if n.pos[2] > alt - 1.0:
            break
    if goto is not None:
        n.send(goto[0], goto[1], alt)
        for _ in range(60):
            spin(n, 1.0)
            if abs(n.pos[1] - goto[0]) < 1.0 and abs(n.pos[0] - goto[1]) < 1.0:
                break
    spin(n, 3.0)
    out = (n.pos[0], n.pos[1], n.pos[2])
    n.destroy_node()
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--world', default='walls',
                    help='world name (as PX4_GZ_WORLD) or path to its .sdf')
    a = ap.parse_args()
    world = world_geometry.resolve_world(a.world)
    boxes = world_geometry.load_boxes(world)
    print("  world %s: %d boxes" % (world_geometry.world_name(world), len(boxes)))

    rclpy.init()
    n = R()
    # Up to 20 s for the first position. A fixed 4 s was enough on the
    # development machine and not on a 4-core one, where DDS discovery had not
    # finished and the test quit with "no position".
    for _ in range(40):
        spin(n, 0.5)
        if n.pos is not None:
            break
    if n.pos is None:
        print("  no position after 20 s")
        return 1
    print("  start north %+.2f east %+.2f alt %.2f armed %s"
          % (n.pos[0], n.pos[1], n.pos[2], n.arm))
    fails = 0

    # Brake mode, and wait for PX4 to actually be in Position mode. The pilot
    # re-requests it every two seconds, so this converges; if it does not,
    # the test still runs and the numbers will say so.
    m = String()
    m.data = 'brake'
    n.mode.publish(m)
    for _ in range(30):
        spin(n, 0.5)
        if n.nav == NAV_POSCTL:
            break
    print("  brake mode: nav_state %s %s"
          % (n.nav, "(Position)" if n.nav == NAV_POSCTL
             else "(NOT Position; collision prevention may not apply)"))

    if n.pos[2] < 4.0:
        n.vcmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 21196.0)
        spin(n, 3.0)
        if n.arm != 2:
            # Found by the clean-clone test: on a fresh install the x500
            # airframe's NAV_DLL_ACT default of 2 waits for a ground station.
            print("  PX4 did not arm (arming_state %s). On a fresh install the usual"
                  % n.arm)
            print("  cause is NAV_DLL_ACT at the airframe default of 2, waiting for a")
            print("  ground station. The launch sets it to 0 through px4-param once PX4")
            print("  answers: check terminal 2 for the [px4_params] lines, or run")
            print("  px4-param show NAV_DLL_ACT, then retry.")
            return 1
        n.send(n.pos[1], n.pos[0], 7.0)
        for _ in range(30):
            spin(n, 1.0)
            if n.pos[2] > 6.0:
                break
        print("  climbed to %.2f m" % n.pos[2])
    else:
        # Already airborne, left there by a previous run: hold position for
        # a few seconds before asking for headings. A heading command sent
        # in the same second the pilot came up once settled 108 degrees
        # off; every later one was within 6.
        n.send(n.pos[1], n.pos[0], 7.0)
        spin(n, 8.0)

    # TEST 2 flies east from this line and measures a brake, so the wall
    # ahead has to be wide on both sides of the line. Near a wall's end,
    # collision prevention does something else that is also correct:
    # CP_GUIDE_ANG (30 deg) steers the setpoint toward free space, and the
    # aircraft slides round the end instead of stopping. Measured: started
    # 0.5 m inside box1's north end, it went round it, then round box4's,
    # and reached east 100 with no wall left to brake at. The plan-mode
    # half parks the aircraft at that end (north 10), so first move well
    # inside the span of the wall ahead: 4 m clear of either end.
    east, north = n.pos[1], n.pos[0]
    ahead = world_geometry.faces_ahead(boxes, east, north, 7.0, 'east', margin=0.5)
    b = ahead[0][1] if ahead else None
    if b is None:
        spans = [x for x in boxes
                 if not x.rotated and x.x_min > east and x.sy >= 2.5
                 and x.z_min <= 7.0 <= x.z_max]
        if spans:
            b = min(spans, key=lambda x: abs(
                min(max(north, x.y_min + 1.0), x.y_max - 1.0) - north))
    if b is not None:
        lo, hi = b.y_min + 4.0, b.y_max - 4.0
        target = (b.y_min + b.y_max) / 2.0 if lo > hi else min(max(north, lo), hi)
        if abs(target - north) > 0.3:
            print("  moving from north %+.2f to %+.2f, well inside %s's span "
                  "(north %+.1f..%+.1f)" % (north, target, b.name, b.y_min, b.y_max))
            n.send(east, target, 7.0)
            for _ in range(40):
                spin(n, 1.0)
                if abs(n.pos[0] - target) < 0.6:
                    break
            spin(n, 3.0)
            print("  now north %+.2f east %+.2f" % (n.pos[0], n.pos[1]))

    print()
    print("  TEST 1: heading. Commanding 4 headings on the spot.")
    east, north = n.pos[1], n.pos[0]
    worst = 0.0
    for hdg in (0.0, 90.0, 180.0, -90.0):
        n.send(east, north, 7.0, hdg)
        spin(n, 22.0)
        err = abs(math.degrees(wrap(math.radians(hdg) - n.yaw)))
        worst = max(worst, err)
        print("    %+4.0f deg -> settled %+7.1f, error %4.1f deg  %s"
              % (hdg, math.degrees(n.yaw), err, "ok" if err <= 12 else "FAIL"))
    if worst > 12:
        fails += 1
    print("    worst heading error: %.1f deg" % worst)

    print()
    print("  TEST 2: avoidance. Face east, fly hard east, see what stops it.")
    # Heading matters here and it is not cosmetic. The camera covers 73
    # degrees ahead and nothing else, and CP_GO_NO_DATA 1 lets PX4 move into
    # directions it has no data for. TEST 1 leaves the aircraft facing west,
    # and the first version of this test then flew it east backwards: the
    # camera never saw the wall, nothing braked, and a 15 m wall at 7 m
    # altitude is a collision. The estimator diverged and every later
    # measurement was garbage. So hold east while flying east.
    n.send(east, north, 7.0, 90.0)
    for _ in range(20):
        spin(n, 1.0)
        if abs(math.degrees(wrap(math.radians(90.0) - n.yaw))) < 10.0:
            break
    print("    facing %+.0f deg" % math.degrees(n.yaw))
    # Is there a wall on this line at all? Checked before flying, so a start
    # off the end of every wall fails in a second instead of after 48 s of
    # flying east into nothing. The half-metre margin covers an aircraft
    # parked right at a wall's end, which the plan-mode half leaves it at.
    pre = world_geometry.faces_ahead(boxes, n.pos[1], n.pos[0], n.pos[2], 'east',
                                     margin=0.5)
    if not pre:
        print("    no wall east of north %+.2f at %.1f m in this world, not flying"
              "  FAIL" % (n.pos[0], n.pos[2]))
        print("    (move the aircraft onto a line with a wall ahead and rerun)")
        fails += 1
    else:
        face, box = pre[0]
        print("    wall ahead: %s, face at east %+.1f" % (box.name, face))
        # Always the same run-up: exactly 8 m from the face, facing it. From
        # closer, the aircraft cannot move toward the wall at all. From
        # further, the approach is long enough for CP_GUIDE_ANG to steer it
        # sideways: measured 2026-10-10 from 15 m out, it drifted 8 m south
        # before braking and then slid round the wall's end. Fly to the
        # run-up point facing the way it is going, so the camera sees the
        # route, then turn to face the wall.
        rx = face - 8.0
        if abs(n.pos[1] - rx) > 1.0:
            print("    %.1f m from the face; moving to the 8 m run-up point"
                  % (face - n.pos[1]))
            n.send(rx, north, 7.0, bearing(n.pos[1], n.pos[0], rx, north))
            for _ in range(60):
                spin(n, 1.0)
                if abs(n.pos[1] - rx) < 0.8 and abs(n.pos[0] - north) < 0.8:
                    break
        n.send(rx, north, 7.0, 90.0)
        for _ in range(20):
            spin(n, 1.0)
            if abs(math.degrees(wrap(math.radians(90.0) - n.yaw))) < 10.0:
                break
        spin(n, 2.0)
        east, north = n.pos[1], n.pos[0]
        print("    run-up from east %+.2f north %+.2f, facing %+.0f deg"
              % (east, north, math.degrees(n.yaw)))
        # Push east and watch, rather than push for a fixed 48 s and read
        # the end position. Collision prevention brakes at CP_DIST, which
        # is the number this test is for; but while the stick keeps
        # pushing, CP_GUIDE_ANG (30 deg) also slides the aircraft along the
        # face toward free space, metres per 10 s, and given long enough it
        # finds a gap or an end and the end position says nothing about
        # braking. Measured from PX4's log: braked at 2.0 m, then crept
        # 4.6 m north into the gap between box1 and box2 and round box1.
        # So: the closest approach to the face, and stop pushing once it
        # has moved and then stood still for 4 s.
        n.send(100.0, north, 7.0, 90.0)
        gap_min, east_prev, still, moved, t_used = 1e9, n.pos[1], 0, False, 0.0
        for i in range(96):
            spin(n, 0.5)
            t_used += 0.5
            east_now = n.pos[1]
            gap_min = min(gap_min, face - east_now)
            if east_now - east_prev > 0.5:
                moved = True
            still = still + 1 if abs(east_now - east_prev) < 0.08 else 0
            east_prev = east_now
            if moved and still >= 8:
                break
        east_f, north_f = n.pos[1], n.pos[0]
        print("    closest approach %.2f m from the face; stopped after %.0f s at "
              "east %+.2f north %+.2f (crept %+.2f m sideways)"
              % (gap_min, t_used, east_f, north_f, north_f - north))
        if not moved:
            print("    never moved east, cannot judge a standoff   FAIL")
            fails += 1
        elif gap_min < 0.0:
            print("    went through or round %s   FAIL" % box.name)
            fails += 1
        else:
            ok = abs(gap_min - CP_DIST) < 0.7
            print("    standoff %.2f m vs CP_DIST %.1f  %s"
                  % (gap_min, CP_DIST, "ok" if ok else "FAIL"))
            if not ok:
                fails += 1

    # Park where it stopped. The east-100 goal would otherwise stay active
    # after this script exits, and on a slow machine an IMU stall that blanks
    # the obstacle data for a moment is enough for CP_GO_NO_DATA 1 to let the
    # aircraft through the wall. Seen once on the clean-clone instance.
    n.send(n.pos[1], n.pos[0], 7.0, 90.0)
    spin(n, 2.0)

    print()
    print("  ===== %s =====" % ("NO REGRESSION" if fails == 0
                                else "%d REGRESSION(S)" % fails))
    n.destroy_node()
    rclpy.try_shutdown()
    return fails


if __name__ == '__main__':
    raise SystemExit(main())
