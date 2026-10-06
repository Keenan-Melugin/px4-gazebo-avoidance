# Measurement scripts

These are the scripts that produced the numbers in the repository README. They
are not unit tests: each one but `histogram_selftest.py` needs PX4, Gazebo and
the stack running, and flies the aircraft.

## The gate

```bash
python3 test/gate.py        # about eight minutes; needs nav2.launch.py up
```

Runs `regression.py` (brake mode) and then `nav2_flight.py` (plan mode) and
keeps score. Run it after any change to the pilot, the frames, the obstacle
node or the Nav2 configuration. If a number moves, the change did something
it was not meant to. Each half also runs on its own.

| Script | What it measures | Mode |
|---|---|---|
| `regression.py` | Puts the pilot in brake mode, commands four cardinal headings and reports the settled error, then flies at a known wall and reports the collision-prevention standoff against `CP_DIST`. Exit code counts failures | brake |
| `nav2_flight.py` | Switches to plan mode, confirms PX4 is in Offboard, sends a Nav2 goal behind the 10 m wall and reports whether the aircraft got past it and how far sideways it went, then switches back | plan |

## The individual measurements

| Script | What it measures |
|---|---|
| `yaw_threshold.py` | Sweeps the yaw stick to find the dead band and the stick-to-rate slope. The yaw table in the README comes from here |
| `xy_threshold.py` | The same sweep for the XY stick: where motion starts and how much velocity a unit of stick commands |
| `yaw_test.py` | Four cardinal headings, settled error. Also checks that a position-only goal leaves the heading alone |
| `hold_test.py` | Separates "does it turn" from "does it stay turned": holds zero stick and watches for drift |
| `avoid_test.py` | Repositions with avoidance off, then flies at a known wall with it on and reports the standoff |
| `velmode_ab.py` | Commanded against achieved speed in velocity mode, with collision prevention on and off. This is the A/B that showed collision prevention and a planner cannot share an axis |
| `mode_test.py` | The brake/plan toggle in both directions, and that plan mode is not subject to collision prevention |
| `ned_check.py` | The body-frame (FLU) to NED velocity conversion in plan mode, by commanding a direction and reading back the velocity PX4 reports. The lateral axis is still unverified, because pure pursuit never commands it |
| `twist_check.py` | That `/odom` carries its twist in base_link FLU, as nav_msgs requires, rather than in the world frame |
| `histogram_selftest.py` | The obstacle node against made-up clouds and scans, bin by bin: the single camera as measured, two sensors merged, stale and dead sensors, a yawed sensor. The only script here that needs no simulator; two seconds |

Run them with the stack up and the workspace sourced, for example:

```bash
python3 test/avoid_test.py
```

Every script that looks for a wall takes `--world NAME` (default `walls`) and
reads the wall positions from that world's file, through the same parser
RViz's markers use; `gate.py` passes it to both halves. `nav2_flight.py` also
takes `--start E N`, `--goal E N` and `--alt`.

Two things to know before trusting a result:

* **Start position matters.** `avoid_test.py` repositions itself; the others
  assume the aircraft is somewhere sensible. A run that starts 100 m outside
  the world measures nothing, which is a mistake these scripts have made.
* **Nothing else should be driving the aircraft.** If someone is clicking in
  RViz while a script runs, the two fight over the same goal topic and the
  numbers are meaningless.
