#!/usr/bin/env python3
"""Template for a measurement script. Copy it, rename it, keep the shape.

    cp test/template_measure.py test/my_measure.py
    python3 test/my_measure.py --world walls

The shape new scripts should follow (the gate's two halves do; the older
single-purpose scripts predate it), because it is what makes a number mean
something a week later:

  1. Say what is being measured, in the docstring, with the question it
     answers.
  2. Read the world (--world) rather than carrying coordinates.
  3. Put the aircraft in a known state first: mode, armed, airborne, position.
  4. Change one thing, wait a fixed time, read a number.
  5. Print the number with its conditions, decide PASS or FAIL against a
     stated threshold, and return the number of failures as the exit code.

This example measures how far the aircraft drifts when left alone for 20 s in
brake mode, which is a thing nobody has measured. Replace the middle.

The node class R in regression.py is the reference: it subscribes to PX4's
position and status, publishes goals, the mode and vehicle commands, and has
send(east, north, alt, heading) and vcmd(). Reuse it rather than writing
another. docs/data.md lists the topics it uses, for driving the aircraft from
anything else.
"""
import argparse
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleCommand
from std_msgs.msg import String

from avoidance_sim import world_geometry
from regression import R, guarded, spin, NAV_POSCTL


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--world', default='walls',
                    help='world name (as PX4_GZ_WORLD) or path to its .sdf')
    ap.add_argument('--hold', type=float, default=20.0, help='seconds to leave it alone')
    ap.add_argument('--limit', type=float, default=1.0, help='drift that counts as a FAIL, m')
    a = ap.parse_args()

    # 2. The world, so a wall can be found if the measurement needs one.
    world = world_geometry.resolve_world(a.world)
    boxes = world_geometry.load_boxes(world)
    print("  world %s: %d boxes" % (world_geometry.world_name(world), len(boxes)))

    rclpy.init()
    n = R()
    for _ in range(40):                       # up to 20 s for the first position
        spin(n, 0.5)
        if n.pos is not None:
            break
    if n.pos is None:
        print("  no position after 20 s")
        return 1
    fails = 0

    # 3. A known state: brake mode, armed, airborne at 7 m, holding here.
    m = String()
    m.data = 'brake'
    n.mode.publish(m)
    for _ in range(30):
        spin(n, 0.5)
        if n.nav == NAV_POSCTL:
            break
    if n.pos[2] < 4.0:
        n.vcmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 21196.0)
        spin(n, 3.0)
        n.send(n.pos[1], n.pos[0], 7.0)
        for _ in range(30):
            spin(n, 1.0)
            if n.pos[2] > 6.0:
                break
    n.send(n.pos[1], n.pos[0], 7.0)
    spin(n, 8.0)
    east0, north0, alt0 = n.pos[1], n.pos[0], n.pos[2]
    print("  holding at east %+.2f north %+.2f alt %.2f, heading %+.0f"
          % (east0, north0, alt0, math.degrees(n.yaw)))

    # 4. The one thing: stop commanding, wait, read. A goal whose frame_id is
    #    STOP tells the pilot to drop its goal and centre the sticks.
    g = PoseStamped()
    g.header.frame_id = 'STOP'
    n.goal.publish(g)
    spin(n, a.hold)
    drift = math.hypot(n.pos[1] - east0, n.pos[0] - north0)
    dalt = n.pos[2] - alt0

    # 5. The number, its conditions, the verdict.
    print()
    print("  ===== RESULT =====")
    print("  drift over %.0f s with no goal: %.2f m horizontal, %+.2f m vertical"
          % (a.hold, drift, dalt))
    ok = drift <= a.limit
    print("  %s: limit %.1f m" % ("PASS" if ok else "FAIL", a.limit))
    if not ok:
        fails += 1

    n.send(n.pos[1], n.pos[0], 7.0)           # leave it holding, for the next script
    spin(n, 2.0)
    n.destroy_node()
    rclpy.try_shutdown()
    return fails


if __name__ == '__main__':
    raise SystemExit(guarded(main))
