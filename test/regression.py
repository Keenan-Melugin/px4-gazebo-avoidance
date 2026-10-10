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
import os
import sys
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
STANDOFF_UNDER = 0.25   # m inside CP_DIST still counted a pass
STANDOFF_OVER = 0.7     # m outside it; recorded runs reach 2.60
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


def px4_param(*args):
    """Run PX4's px4-param client. Finds it on PATH, else under PX4_ROOT.

    The scripts used to call a bare "px4-param", which needs PX4's build
    directory on PATH and raised FileNotFoundError without it.
    """
    import shutil
    import subprocess
    exe = shutil.which('px4-param')
    if exe is None:
        root = os.environ.get('PX4_ROOT') or os.path.expanduser('~/PX4-Autopilot')
        exe = os.path.join(root, 'build', 'px4_sitl_default', 'bin', 'px4-param')
    if not os.path.exists(exe):
        raise RuntimeError("px4-param not found on PATH or under PX4_ROOT (%s); "
                           "add PX4's build/px4_sitl_default/bin to PATH" % exe)
    return subprocess.run([exe] + [str(a) for a in args],
                          capture_output=True, text=True, timeout=20)


def set_param(name, value):
    """Set a PX4 parameter, raising if px4-param reports a failure."""
    r = px4_param('set', name, value)
    if r.returncode != 0:
        raise RuntimeError('px4-param set %s %s failed: %s'
                           % (name, value, (r.stderr or r.stdout).strip()))


def get_param(name):
    """A PX4 parameter's current value as a float, or None."""
    import re
    try:
        r = px4_param('show', name)
    except (RuntimeError, OSError):
        return None
    m = re.search(r'%s\b.*?:\s*(-?[0-9.]+)' % re.escape(name), r.stdout)
    return float(m.group(1)) if m else None


def leg_clearance(boxes, a, b, step=0.5):
    """Smallest distance to any box along the straight leg a -> b, (east, north).

    The start point is skipped: the aircraft is already there, possibly
    parked close to a wall.
    """
    n = max(1, int(math.hypot(b[0] - a[0], b[1] - a[1]) / step))
    return min(world_geometry.clearance(boxes, a[0] + (b[0] - a[0]) * k / n,
                                        a[1] + (b[1] - a[1]) * k / n)
               for k in range(1, n + 1))


def route(boxes, a, b, min_clear=2.5, extent=20.0, step=1.0):
    """Legs from a to b keeping min_clear from every box: [b], [w, b] or None.

    2.5 m is the 2.0 m standoff plus a margin. A leg that comes closer gets
    braked by collision prevention part way, which is how the brake test
    used to start from the wrong line: measured 2026-10-10, moving south from
    where the plan-mode half parks it (north 10), box2 stopped the move at
    north 7.8, the test flew its brake from there, 2.1 m inside box1's end,
    and slid round the end to east 100.

    A shortest path over a 1 m grid of clear points, then cut down to the
    fewest straight legs that stay clear. Returns the legs' end points.
    """
    if leg_clearance(boxes, a, b) >= min_clear:
        return [b]
    import heapq
    x0 = min(a[0], b[0]) - extent
    y0 = min(a[1], b[1]) - extent
    nx = int((abs(a[0] - b[0]) + 2 * extent) / step) + 1
    ny = int((abs(a[1] - b[1]) + 2 * extent) / step) + 1

    def pt(c):
        return (x0 + c[0] * step, y0 + c[1] * step)

    def cell(p):
        return (int(round((p[0] - x0) / step)), int(round((p[1] - y0) / step)))

    free = {}
    # A start closer than min_clear (the aircraft stopped near a wall) used to
    # strand the search: every neighbour was blocked, so it returned None and
    # every later script failed. Within 3 m of the start, cells no closer to
    # a box than the start itself are allowed, which lets it back away.
    start_clear = world_geometry.clearance(boxes, a[0], a[1])

    def ok(c):
        if c not in free:
            p = pt(c)
            cl = world_geometry.clearance(boxes, p[0], p[1])
            near_start = math.hypot(p[0] - a[0], p[1] - a[1]) <= 3.0
            free[c] = (0 <= c[0] < nx and 0 <= c[1] < ny
                       and (cl >= min_clear
                            or (near_start and cl >= start_clear - 0.05)))
        return free[c]

    start, goal = cell(a), cell(b)
    free[start] = free[goal] = True
    dist, prev, heap = {start: 0.0}, {}, [(0.0, start)]
    while heap:
        d, c = heapq.heappop(heap)
        if c == goal:
            break
        if d > dist.get(c, float('inf')):
            continue
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                nb = (c[0] + dx, c[1] + dy)
                if (dx or dy) and ok(nb):
                    nd = d + math.hypot(dx, dy) * step
                    if nd < dist.get(nb, float('inf')):
                        dist[nb], prev[nb] = nd, c
                        heapq.heappush(heap, (nd, nb))
    if goal not in dist:
        return None
    path = [goal]
    while path[-1] != start:
        path.append(prev[path[-1]])
    path = [a] + [pt(c) for c in reversed(path[:-1])][:-1] + [b]
    # Shortcut: from each point, jump to the furthest point still in sight.
    legs, i = [], 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and leg_clearance(boxes, path[i], path[j]) < min_clear:
            j -= 1
        legs.append(path[j])
        i = j
    return legs


def pick_line(boxes, east, north, alt=7.0):
    """The wall to brake at and the north of the line to fly at it on.

    The wall straight ahead (east) if there is one, otherwise every wall wide
    enough, nearest first. A line qualifies if it is 4 m inside the wall's
    ends (closer, and the 30 degree guidance slides round the end instead of
    braking), its 8 m run-up point is 2.5 m clear of every box, and the run
    to the face passes 2.5 m clear of every other box. Returns (box, north),
    or (None, None) if no wall in the world has such a line.
    """
    ahead = [b for _, b in world_geometry.faces_ahead(boxes, east, north, alt,
                                                      'east', margin=0.5)]
    spans = sorted((x for x in boxes
                    if not x.rotated and x.sy >= 8.0 and x.z_min <= alt <= x.z_max),
                   key=lambda x: math.hypot(
                       x.x_min - 8.0 - east,
                       min(max(north, x.y_min + 4.0), x.y_max - 4.0) - north))
    for b in ahead[:1] + [x for x in spans if x not in ahead[:1]]:
        lo, hi = b.y_min + 4.0, b.y_max - 4.0
        if lo > hi:
            continue
        others = [x for x in boxes if x is not b]
        for t in sorted((lo + 0.5 * i for i in range(int((hi - lo) / 0.5) + 1)),
                        key=lambda t: abs(t - north)):
            if (world_geometry.clearance(boxes, b.x_min - 8.0, t) >= 2.5
                    and leg_clearance(others, (b.x_min - 8.0, t), (b.x_min, t)) >= 2.5):
                return b, t
    return None, None


def world_boxes(default='walls'):
    """The boxes of the world named by --world on the command line, or walls.

    For scripts that take no other arguments, so they need no argparse.
    """
    w = default
    for i, arg in enumerate(sys.argv):
        if arg == '--world' and i + 1 < len(sys.argv):
            w = sys.argv[i + 1]
        elif arg.startswith('--world='):
            w = arg.split('=', 1)[1]
    path = world_geometry.resolve_world(w)
    require_modelled(path)
    boxes = world_geometry.load_boxes(path)
    print("  world %s: %d boxes" % (world_geometry.world_name(path), len(boxes)))
    return boxes


def require_modelled(path):
    """Stop if the world holds obstacles the scripts cannot see.

    Routes, open air and run-up lines are planned from the plain boxes alone.
    Included models and meshes would be flown through as if absent.
    """
    missing = world_geometry.unmodelled(path)
    if missing:
        print("  ABORT: %s has %d obstacle(s) the test geometry cannot model "
              "(only plain <box> models are read), for example %s. Routes and "
              "open air computed without them would fly through them."
              % (world_geometry.world_name(path), len(missing), missing[0]))
        raise SystemExit(2)


def open_air(boxes, legs, near, margin=5.0, extent=60.0, step=2.0):
    """The start point nearest `near` from which `legs` stay clear of boxes.

    legs are (east, north) offsets flown one after another, facing north.
    margin is the distance kept from every box along the whole path: 5 m is
    the 2.0 m standoff, braking from about 1.5 m/s, and room to spare.
    Measured 2026-10-10: the mode test's sideways leg, flown from wherever it
    happened to be, hit box1 at 1 m/s in plan mode, where nothing brakes,
    and the aircraft fell to the ground. In Position mode the same mistake is
    quieter: collision prevention brakes the leg and the measurement reads
    low. Returns (east, north), or None.
    """
    k = int(extent / step)
    cands = sorted(((near[0] + i * step, near[1] + j * step)
                    for i in range(-k, k + 1) for j in range(-k, k + 1)),
                   key=lambda p: math.hypot(p[0] - near[0], p[1] - near[1]))
    for p in cands:
        if world_geometry.clearance(boxes, p[0], p[1]) < margin:
            continue
        pts = [p]
        for de, dn in legs:
            pts.append((pts[-1][0] + de, pts[-1][1] + dn))
        if all(leg_clearance(boxes, a, b) >= margin for a, b in zip(pts, pts[1:])):
            return p
    return None


def fly_route(n, boxes, dest, alt=7.0):
    """Fly to dest (east, north) by a clear route, facing each leg.

    Facing matters: the camera sees 73 degrees ahead and nothing else.
    Returns True if it arrived within 0.8 m, False if it did not or no
    clear route exists.
    """
    legs = route(boxes, (n.pos[1], n.pos[0]), dest)
    if legs is None:
        print("    no route to east %+.1f north %+.1f that stays 2.5 m from every box"
              % dest)
        return False
    if len(legs) > 1:
        print("    direct path is blocked; going via east %+.1f north %+.1f" % legs[0])
    for e, no in legs:
        n.send(e, no, alt, bearing(n.pos[1], n.pos[0], e, no))
        for _ in range(60):
            spin(n, 1.0)
            if abs(n.pos[1] - e) < 0.8 and abs(n.pos[0] - no) < 0.8:
                break
        else:
            # The next leg was only checked from where this one ends.
            print("    leg to east %+.1f north %+.1f not reached in 60 s "
                  "(at east %+.1f north %+.1f)" % (e, no, n.pos[1], n.pos[0]))
            return False
    spin(n, 2.0)
    return abs(n.pos[1] - dest[0]) < 0.8 and abs(n.pos[0] - dest[1]) < 0.8


def route_to(dest, alt, boxes):
    """Fly to dest (east, north) at alt by a clear route, on a node of its own.

    For scripts whose own node class has no send(): twist_check and
    nav2_flight used to reposition in one straight line, which from where
    the gate leaves the aircraft passes through box2.
    """
    n = R()
    for _ in range(40):
        spin(n, 0.5)
        if n.pos is not None:
            break
    ok = n.pos is not None and fly_route(n, boxes, dest, alt)
    n.destroy_node()
    return ok


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


def set_pilot_mode(mode, also=None):
    """Publish a pilot mode with the retained QoS the pilot subscribes with.

    `also` is the caller's own node, spun alongside. A script that streams
    its own sticks must pass it: spinning only this helper's node starved the
    script's 50 Hz timer for 2.5 s, longer than COM_RC_LOSS_T (0.5 s), so PX4
    saw the sticks stop at the very moment the script took them over.
    Switch back to brake BEFORE stopping your own stream, for the same reason.
    """
    from rclpy.executors import SingleThreadedExecutor
    n = R()
    ex = SingleThreadedExecutor()
    ex.add_node(n)
    if also is not None:
        ex.add_node(also)

    def run(s):
        t = time.time()
        while time.time() - t < s:
            ex.spin_once(timeout_sec=0.02)
    m = String()
    m.data = mode
    run(1.0)
    n.mode.publish(m)
    run(1.5)
    ex.remove_node(n)
    if also is not None:
        ex.remove_node(also)
    n.destroy_node()


def leave_safe(cancel_nav2=True):
    """Brake mode, any Nav2 goal cancelled, and a STOP goal: a known state.

    Safe to call with or without rclpy initialised; it initialises and shuts
    down around itself when it has to. A script that ended in external or
    plan mode, or with a Nav2 goal still running, used to hand that state to
    the next script, and the next script's goals were overridden by it.
    """
    own = not rclpy.ok()
    if own:
        rclpy.init()
    n = R()
    try:
        if cancel_nav2:
            from action_msgs.srv import CancelGoal
            cli = n.create_client(CancelGoal,
                                  '/navigate_to_pose/_action/cancel_goal')
            if cli.wait_for_service(timeout_sec=1.0):
                cli.call_async(CancelGoal.Request())   # blank = cancel all
        # Brake only if the pilot is not in brake already. Re-sending brake is
        # the explicit way to take Position mode back, so sending it every
        # time would cancel a Land or Return in progress.
        latched = []
        sub = n.create_subscription(String, '/avoidance_sim/mode',
                                    lambda m: latched.append(m.data), MODE_QOS)
        spin(n, 1.0)
        n.destroy_subscription(sub)
        if latched and latched[-1].strip().lower() != 'brake':
            m = String()
            m.data = 'brake'
            n.mode.publish(m)
        t0 = time.time()
        while n.goal.get_subscription_count() == 0 and time.time() - t0 < 5.0:
            spin(n, 0.1)
        stop = PoseStamped()
        stop.header.frame_id = 'STOP'
        n.goal.publish(stop)
        spin(n, 1.5)
    finally:
        n.destroy_node()
        if own:
            rclpy.try_shutdown()


def guarded(run):
    """Run a test's body between two leave_safe() calls, whatever it does.

    The body owns rclpy.init() and shutdown as before. A Ctrl-C or an
    exception still gets the closing leave_safe().
    """
    leave_safe()
    rtf_warning()
    try:
        return run()
    finally:
        leave_safe()


def rtf_warning(window=3.0):
    """Print the simulation's real-time factor and warn below 0.9.

    The scripts time their windows on the wall clock while PX4 runs on
    simulated time, so at a real-time factor of 0.5 a "5 s" acceleration is
    2.5 s of simulated flight and rates come out halved. Measured: 1.00
    headless on the development machine, 0.55 to 0.89 with the Gazebo GUI.
    """
    from rosgraph_msgs.msg import Clock
    own = not rclpy.ok()
    if own:
        rclpy.init()
    n = Node('rtf_probe')
    seen = []
    n.create_subscription(Clock, '/clock',
                          lambda m: seen.append(m.clock.sec + m.clock.nanosec * 1e-9),
                          QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                     history=HistoryPolicy.KEEP_LAST, depth=10))
    # Discovery first (a new node takes seconds to match here), then the
    # measurement window from the first message.
    t_disc = time.time()
    while not seen and time.time() - t_disc < 10.0:
        rclpy.spin_once(n, timeout_sec=0.05)
    seen.clear()
    t0 = time.time()
    while time.time() - t0 < window:
        rclpy.spin_once(n, timeout_sec=0.05)
    elapsed = time.time() - t0
    n.destroy_node()
    if own:
        rclpy.try_shutdown()
    if len(seen) < 2:
        print("  real-time factor: unknown (no /clock)")
        return
    rtf = (seen[-1] - seen[0]) / max(1e-3, elapsed)
    print("  real-time factor %.2f%s" % (
        rtf, "" if rtf >= 0.9 else
        "  WARNING: below 0.9, so timed windows and measured rates are off"))


def ensure_airborne(alt=7.0, goto=None, legs=None, margin=5.0):
    """Arm, take off and hold, from whatever state the last script left.

    The single-purpose scripts measure things in flight and used to assume
    the aircraft was already up, which on a fresh stack meant no measurement.
    Call this after rclpy.init(). It uses its own node and destroys it, so the
    calling script's node is untouched. `goto` is an optional (east, north) to
    fly to first, at `alt`, in brake mode with collision prevention live,
    by a route that keeps clear of the world's boxes. `legs` instead names
    the (east, north) legs the script is about to fly facing north: the
    aircraft goes to the nearest start where they keep `margin` from every
    box (see open_air), and turns to face north. Returns the (north, east,
    alt) it ended at, or None if it could not arm or find open air.
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
    else:
        print("  climb to %.1f m not reached (at %.1f m)" % (alt, n.pos[2]))
        n.destroy_node()
        return None
    boxes = world_boxes() if (goto is not None or legs is not None) else None
    if legs is not None:
        goto = open_air(boxes, legs, (n.pos[1], n.pos[0]), margin)
        if goto is None:
            print("  no start in this world keeps %.0f m from every box for "
                  "these legs" % margin)
            n.destroy_node()
            return None
        print("  open air for the legs: east %+.1f north %+.1f" % goto)
    if goto is not None:
        if not fly_route(n, boxes, goto, alt):
            print("  did not reach east %+.1f north %+.1f" % goto)
            n.destroy_node()
            return None
    if legs is not None:
        n.send(n.pos[1], n.pos[0], alt, 0.0)       # face north
        # 10 degrees: the pilot counts a heading as reached within 8
        # (ARRIVED_YAW) and stops turning there, so a tighter test failed
        # on an aircraft settled 5 degrees off.
        for _ in range(25):
            spin(n, 1.0)
            if abs(math.degrees(wrap(n.yaw))) < 10.0:
                break
        else:
            print("  did not turn to face north (heading %+.0f)"
                  % math.degrees(n.yaw))
            n.destroy_node()
            return None
    spin(n, 3.0)
    out = (n.pos[0], n.pos[1], n.pos[2])
    n.destroy_node()
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--world', default='walls',
                    help='world name (as PX4_GZ_WORLD) or path to its .sdf')
    ap.add_argument('--cp-dist', type=float, default=None,
                    help='the CP_DIST to judge the standoff against; read from '
                         'PX4 when omitted')
    a = ap.parse_args()
    world = world_geometry.resolve_world(a.world)
    require_modelled(world)
    boxes = world_geometry.load_boxes(world)
    global CP_DIST
    cp = a.cp_dist if a.cp_dist is not None else get_param('CP_DIST')
    if cp is None:
        print("  could not read CP_DIST from PX4; judging against %.1f" % CP_DIST)
    elif cp <= 0:
        print("  CP_DIST is %.1f in PX4: collision prevention is OFF, so there "
              "is no standoff to measure  ABORT" % cp)
        return 1
    else:
        CP_DIST = cp
    print("  judging the standoff against CP_DIST %.2f m" % CP_DIST)
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

    # Brake mode, and wait for PX4 to actually be in Position mode. Sending
    # brake asks for Position mode even when the pilot is already in brake
    # (that is how a user takes it back after a Land or a failsafe), and the
    # pilot repeats the request each second until PX4 enters it. If it never
    # does, the test still runs and the numbers will say so.
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
        # Retried, like ensure_airborne: PX4 refuses to arm for tens of
        # seconds after boot while the estimator settles.
        for _ in range(20):
            n.vcmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 21196.0)
            spin(n, 3.0)
            if n.arm == 2:
                break
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
    b, target = pick_line(boxes, east, north)
    placed = True
    if b is None:
        print("  no wall in this world has a clear 8 m run-up line  ABORT")
        placed = False
        fails += 1
    elif (abs(target - north) > 0.3
          or abs(east - (b.x_min - 8.0)) > 0.8):
        # Straight to the 8 m run-up point on that line, by a clear route.
        print("  moving to the run-up point for %s: east %+.1f north %+.2f, "
              "well inside its span (north %+.1f..%+.1f)"
              % (b.name, b.x_min - 8.0, target, b.y_min, b.y_max))
        placed = fly_route(n, boxes, (b.x_min - 8.0, target))
        print("  now north %+.2f east %+.2f" % (n.pos[0], n.pos[1]))
        if not placed:
            print("  did not reach that line  ABORT (TEST 2 would measure the"
                  " wrong thing)")
            fails += 1

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
                                     margin=0.5) if placed else []
    if not placed:
        print("    skipped: not on a measurable line (see above)")
    elif not pre:
        print("    no wall east of north %+.2f at %.1f m in this world, not flying"
              "  FAIL" % (n.pos[0], n.pos[2]))
        print("    (move the aircraft onto a line with a wall ahead and rerun)")
        fails += 1
    else:
        face, box = pre[0]
        print("    wall ahead: %s, face at east %+.1f" % (box.name, face))
    span = (box.y_max - box.y_min) if pre else 0.0
    if pre and span < 8.0:
        # Narrower than 8 m and the 30 degree guidance steers round the end
        # instead of braking, so the number would measure the wrong thing.
        print("    %s spans only %.1f m north-south; need 8 m to measure a brake"
              "  ABORT" % (box.name, span))
        fails += 1
        pre = []
    if pre:
        # Always the same run-up: exactly 8 m from the face, facing it. From
        # closer, the aircraft cannot move toward the wall at all. From
        # further, the approach is long enough for CP_GUIDE_ANG to steer it
        # sideways: measured 2026-10-10 from 15 m out, it drifted 8 m south
        # before braking and then slid round the wall's end. Fly to the
        # run-up point facing the way it is going, so the camera sees the
        # route, then turn to face the wall.
        rx = face - 8.0
        # The transit to the run-up point is flown before anything is
        # measured, so check it is open air first. Collision prevention would
        # brake the transit too, and the test would then measure the brake
        # from wherever that left it, not from 8 m. 1.5 m keeps the path
        # outside the 2.0 m standoff band minus the airframe's own half-width.
        e0, n0 = n.pos[1], n.pos[0]
        steps = max(1, int(math.hypot(rx - e0, north - n0) / 0.5))
        # The start point is skipped: the aircraft is already there.
        tight = min(world_geometry.clearance(boxes, e0 + (rx - e0) * k / steps,
                                             n0 + (north - n0) * k / steps)
                    for k in range(1, steps + 1))
        if tight < 1.5:
            print("    the transit to the run-up point passes %.2f m from a box"
                  "  ABORT" % tight)
            print("    (start the test from a clearer spot, or move the run-up)")
            fails += 1
            pre = []
    if pre:
        if abs(n.pos[1] - rx) > 1.0:
            print("    %.1f m from the face; moving to the 8 m run-up point"
                  % (face - n.pos[1]))
            n.send(rx, north, 7.0, bearing(n.pos[1], n.pos[0], rx, north))
            for _ in range(60):
                spin(n, 1.0)
                if abs(n.pos[1] - rx) < 0.8 and abs(n.pos[0] - north) < 0.8:
                    break
        if abs(n.pos[1] - rx) > 1.0 or abs(n.pos[0] - north) > 1.0:
            # Measuring from wherever it stopped would report a standoff
            # from an unknown run-up, which is the number this test exists
            # to keep comparable.
            print("    did not reach the run-up point (east %+.2f north %+.2f, "
                  "wanted %+.2f %+.2f)  ABORT"
                  % (n.pos[1], n.pos[0], rx, north))
            fails += 1
            pre = []
    if pre:
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
            # Asymmetric on purpose. Closer than the setpoint is the unsafe
            # direction and gets 0.25 m. Further is conservative, and the
            # measured runs reach 2.60 m, so it keeps 0.7 m.
            ok = CP_DIST - STANDOFF_UNDER <= gap_min <= CP_DIST + STANDOFF_OVER
            print("    standoff %.2f m vs CP_DIST %.1f (allowed %.2f to %.2f)  %s"
                  % (gap_min, CP_DIST, CP_DIST - STANDOFF_UNDER,
                     CP_DIST + STANDOFF_OVER, "ok" if ok else "FAIL"))
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
    raise SystemExit(guarded(main))
