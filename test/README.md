# Measurement scripts

These are the scripts that produced the numbers in the repository README. They
are not unit tests: each one needs PX4, Gazebo and the stack running, and each
flies the aircraft.

| Script | What it measures |
|---|---|
| `yaw_threshold.py` | Sweeps the yaw stick to find the dead band and the stick-to-rate slope. This is where the table in the README comes from. |
| `yaw_test.py` | Commands four cardinal headings and reports the settled error. Also checks a position-only goal leaves the heading alone. |
| `hold_test.py` | Separates "does it turn" from "does it stay turned", by holding zero stick and watching for drift. |
| `avoid_test.py` | Repositions with avoidance off, then flies at a known wall with it on and reports the standoff. |
| `regression.py` | Heading and avoidance together, to check a refactor changed nothing. |

Run them with the stack up and the workspace sourced, for example:

```bash
python3 test/avoid_test.py
```

Two things to know before trusting a result:

* **Start position matters.** `avoid_test.py` repositions itself; the others
  assume the aircraft is somewhere sensible. A run that starts 100 m outside
  the world measures nothing, which is a mistake these scripts have made.
* **Nothing else should be driving the aircraft.** If someone is clicking in
  RViz while a script runs, the two fight over the same goal topic and the
  numbers are meaningless. `regression.py` prints whether the RViz marker was
  touched during the run for this reason.
