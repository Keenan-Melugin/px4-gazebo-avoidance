#!/usr/bin/env python3
"""The whole regression gate: brake mode, then plan mode.

    python3 test/gate.py                 # the walls world
    python3 test/gate.py --world pillars # another, started with PX4_GZ_WORLD=pillars

Needs PX4, Gazebo and nav2.launch.py running, the aircraft somewhere sensible
(see README.md in this directory) and nobody else touching it. Takes about
eight minutes. The exit code is the number of halves that failed.

The halves are separate scripts because each is useful alone: regression.py
after a change to the pilot, the frames or the obstacle node; nav2_flight.py
after a change to the Nav2 configuration. This runs them in order and keeps
score, so "run the gate" means one command.
"""
import argparse
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
HALVES = (
    ("brake mode: heading hold and collision-prevention standoff", "regression.py"),
    ("plan mode: Nav2 routes round the wall", "nav2_flight.py"),
)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--world', default='walls',
                    help='passed to both halves: the world PX4 was started with')
    a = ap.parse_args()
    failed = 0
    t0 = time.time()
    print("  world: %s" % a.world)
    for label, script in HALVES:
        print("=" * 72)
        print("  %s  (%s)" % (label, script))
        print("=" * 72, flush=True)
        # -u, or the half's output sits in a pipe buffer until it exits and
        # a ten-minute run looks dead.
        rc = subprocess.call([sys.executable, '-u', os.path.join(HERE, script),
                              '--world', a.world])
        print("  -> %s exit %d" % (script, rc))
        print(flush=True)
        if rc != 0:
            failed += 1
    print("=" * 72)
    verdict = "PASS" if failed == 0 else "%d of %d FAILED" % (failed, len(HALVES))
    print("  GATE: %s, %.0f s" % (verdict, time.time() - t0))
    return failed


if __name__ == "__main__":
    raise SystemExit(main())
