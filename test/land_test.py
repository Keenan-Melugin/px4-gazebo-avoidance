#!/usr/bin/env python3
"""Does a Land stand while the pilot is flying a goal?

The first fix for this only stopped the pilot re-asking for Position mode. It
passed a check made while no goal was active, and was still wrong: with a goal
active the pilot's sticks keep changing, PX4's stick override
(COM_RC_OVERRIDE, default 1) takes a multicopter from an automatic mode back
to Position mode when the stick input moves, and the Land ended at the first
throttle step. This flies a goal, commands LAND half way, and checks:

  1. PX4 stays in Land (nav_state 18) and keeps descending;
  2. a new pilot goal during the Land is refused;
  3. publishing brake takes over again (Position mode, nav_state 2).

Exit code 0 only if all three hold.
"""
import os
import sys
import time

import rclpy
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from regression import (R, ensure_airborne, guarded, spin,  # noqa: E402
                        NAV_POSCTL)
from px4_msgs.msg import VehicleCommand  # noqa: E402
from std_msgs.msg import String  # noqa: E402

NAV_LAND = 18
LEG = 20.0      # m north, long enough to still be flying when LAND arrives


def main():
    rclpy.init()
    if ensure_airborne(alt=8.0, legs=[(0.0, LEG)]) is None:
        print("  could not take off into open air"); return 1
    n = R()
    for _ in range(40):
        spin(n, 0.5)
        if n.pos is not None and n.nav is not None:
            break
    print("  at north %+.1f east %+.1f alt %.1f, nav %s"
          % (n.pos[0], n.pos[1], n.pos[2], n.nav))
    fails = []

    print("\n  flying a %.0f m goal north, then LAND half way" % LEG)
    n.send(n.pos[1], n.pos[0] + LEG, 8.0, 0.0)
    spin(n, 3.0)            # mid-leg: the aircraft covers 20 m in about 6 s
    # LAND where it is: NaN position, as a ground station sends it.
    c = VehicleCommand()
    c.timestamp = n.get_clock().now().nanoseconds // 1000
    c.command = VehicleCommand.VEHICLE_CMD_NAV_LAND
    c.param4 = c.param5 = c.param6 = c.param7 = float('nan')
    c.target_system = c.target_component = 1
    c.source_system = c.source_component = 1
    c.from_external = True
    n.cmd.publish(c)
    # Both checks happen in the air. A Land from 8 m takes about 12 s, and
    # once PX4 has landed and disarmed it returns to Position mode by itself,
    # which a later check would misread as the Land being cancelled.
    seen, alt0 = [], n.pos[2]
    t0 = time.time()
    while time.time() - t0 < 4.0:
        spin(n, 0.2)
        if not seen or seen[-1] != n.nav:
            seen.append(n.nav)
    after = seen[seen.index(NAV_LAND):] if NAV_LAND in seen else []
    held = n.nav == NAV_LAND and NAV_POSCTL not in after
    print("  1. states after LAND: %s; altitude %.1f -> %.1f m  %s"
          % (seen, alt0, n.pos[2], "PASS (Land kept)" if held else "FAIL"))
    if not held:
        fails.append("Land did not stand while a goal was active")
    if n.pos[2] > alt0 - 1.0:
        fails.append("not descending in Land")

    print("\n  2. a new pilot goal during the Land, still in the air")
    n.send(n.pos[1] + 5.0, n.pos[0], 8.0)
    seen2 = []
    t0 = time.time()
    while time.time() - t0 < 3.0:
        spin(n, 0.2)
        if not seen2 or seen2[-1] != n.nav:
            seen2.append(n.nav)
    ok2 = seen2 == [NAV_LAND] and n.pos[2] > 1.0
    print("     states %s at %.1f m  %s" % (
        seen2, n.pos[2], "PASS (refused)" if ok2 else
        ("FAIL (Land cancelled)" if NAV_POSCTL in seen2
         else "INCONCLUSIVE (landed before the check)")))
    if not ok2:
        fails.append("a goal cancelled the Land, or the check came too late")

    print("\n  3. brake on /avoidance_sim/mode takes over")
    m = String()
    m.data = 'brake'
    n.mode.publish(m)
    for _ in range(20):
        spin(n, 0.5)
        if n.nav == NAV_POSCTL:
            break
    ok3 = n.nav == NAV_POSCTL
    print("     nav_state %s  %s" % (n.nav, "PASS" if ok3 else "FAIL"))
    if not ok3:
        fails.append("brake did not take Position mode back")
    # Hold where it is, and climb back clear of the ground.
    n.send(n.pos[1], n.pos[0], 7.0)
    spin(n, 6.0)

    print("\n  ===== %s =====" % ("ALL PASS" if not fails else "FAILURES"))
    for f in fails:
        print("    - %s" % f)
    n.destroy_node()
    rclpy.try_shutdown()
    return 1 if fails else 0


if __name__ == '__main__':
    raise SystemExit(guarded(main))
