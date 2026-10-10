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
| `regression.py` | Puts the pilot in brake mode, commands four cardinal headings and reports the settled error, then picks a wall from the world file with a clear line 4 m inside its ends, flies a route round any other box to the 8 m run-up point, flies at the wall and reports the closest approach against `CP_DIST` (pass from 0.25 m inside it to 0.7 m outside), stopping once the aircraft stands still. It aborts rather than measure from the wrong place. Exit code counts failures | brake |
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
| `ned_check.py` | The body-frame (FLU) to NED velocity conversion in plan mode, by commanding forward, right and left in turn and reading back how far the aircraft moved in the world |
| `twist_check.py` | That `/odom` carries its twist in base_link FLU, as nav_msgs requires, rather than in the world frame |
| `template_measure.py` | Not a measurement: the skeleton to copy for a new one, with the five-step shape new scripts should follow. Its placeholder measures drift when the aircraft is left alone |
| `histogram_selftest.py` | The obstacle node against made-up clouds and scans, bin by bin: the single camera as measured, two sensors merged, stale and dead sensors, a yawed sensor. The only script here that needs no simulator; two seconds |

## Last run, 2026-10-10

Every script in sequence on one freshly started stack, then the order that
used to fail (`nav2_flight.py` straight into `regression.py`), then the gate,
about forty minutes of flying in all:

| Script | Result |
|---|---|
| `histogram_selftest.py` | Pass |
| `regression.py`, `nav2_flight.py`, `gate.py` | Pass. Standoffs 2.00 and 2.02 m; Nav2 ended 0.4 to 0.8 m from its goal. The regression run straight after `nav2_flight.py` routed round box2 to its line and passed |
| `yaw_test.py`, `twist_check.py`, `template_measure.py`, `velmode_ab.py`, `ned_check.py` | Pass. In open air `ned_check.py` moved 10.5 m on each leg in 11 s at 1 m/s with the heading steady; `velmode_ab.py` achieved 0.92 m/s with collision prevention on and off |
| `hold_test.py` | Holds: no drift in 30 s, 6.7 degrees of coast past the cut point. Earlier runs measured 2.5, 3.5 and 27 degrees, from before the test took over the stick stream |
| `mode_test.py` | Pass: 1.02 m/s forward and 1.02 m/s right for 1.00 commanded. Its first run of the day read 0.34 m/s right: a trace showed the leg hitting box1 at 1 m/s in plan mode and the aircraft falling to the ground, from a start the script called "well clear of the walls". It had always exited 0, so nobody saw. It now starts in open air found from the world file |
| `yaw_threshold.py` | Turns from stick 0.15, as before |
| `xy_threshold.py` | Repeats in open air: dead band 0.115 and 0.114, slope 5.05 and 5.03 m/s per unit in two runs from different starts, matching the pilot's 0.114 and 5.02. The earlier spread (5.0, 2.4, 1.2) was walls: the sweep flew forward from wherever it was, and collision prevention braked it |

The late-session failures recorded before this run are explained. The plan
half leaves the aircraft at north 10, beyond box2. The brake half then tried
to move straight south to its line, box2 stopped the move at north 7.8, and
the test flew its brake from there, 2.1 m inside box1's end, where collision
prevention steered it round the end of the wall to east 100. It appeared late
in a session because only the gate, or a run after `nav2_flight.py`, starts
there. `regression.py` now routes round obstacles and refuses to measure from
a line it did not reach.

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
  flight call `ensure_airborne()` from `regression.py`. `twist_check.py` also
  flies to open air from the world file first, because near a wall collision
  prevention deflects its legs. The standoff is measured by the
  gate's brake half; the older `avoid_test.py` was retired on 2026-10-10 after
  its straight-line reposition with avoidance off flew through a wall.
* **Each script that flies fixed legs finds open air first.** `ensure_airborne(legs=...)`
  takes the legs the script will fly facing north, finds the nearest start in
  the world file where they stay 5 m from every box, flies there by a clear
  route, and faces north. `mode_test.py`, `ned_check.py`, `velmode_ab.py` and
  `xy_threshold.py` use it; a new script that flies set distances should too.
* **Each script leaves a known state.** Every flying script runs inside
  `guarded()` from `regression.py`: before it starts and after it ends,
  whatever the exit (an abort, an exception, Ctrl-C), any Nav2 goal is
  cancelled, the pilot goes back to brake mode and a STOP goal is sent.
  `velmode_ab.py` also restores `CP_DIST` in its own `finally`, because it
  switches collision prevention off for its second trial.
* **Nothing else should be driving the aircraft.** If someone is clicking in
  RViz while a script runs, the two fight over the same goal topic and the
  numbers are meaningless.
