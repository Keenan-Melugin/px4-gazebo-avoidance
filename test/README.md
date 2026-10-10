# Measurement scripts

These are the scripts that produced the numbers in the repository README. They
are not unit tests: each one but `histogram_selftest.py` needs PX4, Gazebo and
the stack running, and flies the aircraft.

## The gate

```bash
python3 test/gate.py        # about four minutes; needs nav2.launch.py up
```

Runs `regression.py` (brake mode) and then `nav2_flight.py` (plan mode) and
keeps score. Run it after any change to the pilot, the frames, the obstacle
node or the Nav2 configuration. If a number moves, the change did something
it was not meant to. Each half also runs on its own.

| Script | What it measures | Mode |
|---|---|---|
| `regression.py` | Puts the pilot in brake mode, commands four cardinal headings and reports the settled error, then backs off 8 m from a wall read from the world file, flies at it and reports the closest approach against `CP_DIST`, stopping once the aircraft stands still. Exit code counts failures | brake |
| `nav2_flight.py` | Switches to plan mode, confirms PX4 is in Offboard, sends a Nav2 goal behind the 10 m wall and reports whether the aircraft got past it and how far sideways it went, then switches back | plan |

## The individual measurements

| Script | What it measures |
|---|---|
| `yaw_threshold.py` | Sweeps the yaw stick to find the dead band and the stick-to-rate slope. The yaw table in the README comes from here |
| `xy_threshold.py` | The same sweep for the XY stick: where motion starts and how much velocity a unit of stick commands |
| `yaw_test.py` | Four cardinal headings, settled error. Also checks that a position-only goal leaves the heading alone |
| `hold_test.py` | Separates "does it turn" from "does it stay turned": holds zero stick and watches for drift |
| `velmode_ab.py` | Commanded against achieved speed in velocity mode, with collision prevention on and off. This is the A/B that showed collision prevention and a planner cannot share an axis |
| `mode_test.py` | The brake/plan toggle in both directions, and that plan mode is not subject to collision prevention |
| `ned_check.py` | The body-frame (FLU) to NED velocity conversion in plan mode, by commanding a direction and reading back the velocity PX4 reports. The lateral axis is still unverified, because pure pursuit never commands it |
| `twist_check.py` | That `/odom` carries its twist in base_link FLU, as nav_msgs requires, rather than in the world frame |
| `template_measure.py` | Not a measurement: the skeleton to copy for a new one, with the five-step shape new scripts should follow. Its placeholder measures drift when the aircraft is left alone |
| `histogram_selftest.py` | The obstacle node against made-up clouds and scans, bin by bin: the single camera as measured, two sensors merged, stale and dead sensors, a yawed sensor. The only script here that needs no simulator; two seconds |

## Last run, 2026-10-10

Every script was run on the development machine, on fresh stacks, three times
in sequence while the scripts were being repaired. What each one did:

| Script | Result |
|---|---|
| `histogram_selftest.py` | Pass, every run |
| `regression.py`, `nav2_flight.py`, `gate.py` | Pass on a fresh stack in every run; standoff 2.00 to 2.02 m. Late in a long back-to-back session they failed twice, once after a crash caused by an older test and once with no cause found yet |
| `yaw_test.py`, `mode_test.py`, `twist_check.py`, `template_measure.py`, `velmode_ab.py` | Pass. `yaw_test.py` lost its first command until the connection guard was added |
| `hold_test.py` | Runs; coast past the cut point measured at 2.5, 3.5 and 27 degrees in three runs, so treat one run as indicative only |
| `yaw_threshold.py`, `xy_threshold.py` | Run since the pilot gained its external mode. The yaw sweep repeated (turns from 0.15, rates matching the table in how-it-works.md). The XY slope did not: 5.0 and 2.4 m/s per unit in two runs. Not yet trusted |
| `ned_check.py` | Passed twice, failed once late in a long session with a heading swing during a leg. Run it on a fresh stack |

The rule that follows: run a measurement on a freshly started stack, and
restart PX4, Gazebo and the launch between long sessions. Twenty-plus minutes
of back-to-back flying produced failures that moved between scripts from run
to run, with no failsafe, estimator reset or stick loss in PX4's log to
explain them. That is open.

Run them with the stack up and the workspace sourced, for example:

```bash
python3 test/yaw_test.py
```

Every script that looks for a wall takes `--world NAME` (default `walls`) and
reads the wall positions from that world's file, through the same parser
RViz's markers use; `gate.py` passes it to both halves. `nav2_flight.py` also
takes `--start E N`, `--goal E N` and `--alt`.

Two things to know before trusting a result:

* **Each script takes off itself if it has to.** The ones that measure in
  flight call `ensure_airborne()` from `regression.py`, and `twist_check.py`
  also flies to open air from the world file first, because near a wall
  collision prevention deflects its legs. The standoff is measured by the
  gate's brake half; the older `avoid_test.py` was retired on 2026-10-10 after
  its straight-line reposition with avoidance off flew through a wall.
* **Nothing else should be driving the aircraft.** If someone is clicking in
  RViz while a script runs, the two fight over the same goal topic and the
  numbers are meaningless.
