# How it works

Two data paths, one aircraft. This page is what you need to know to read the
code, in the order the data flows, with the measurements that shaped each
decision. Every number is from this stack unless it says otherwise. Terms
from [pieces.md](pieces.md) are not redefined here; new ones are defined
where they first appear.

## The two paths

Brake mode:

```
Gazebo depth camera --> /depth_camera/points --> obstacle_distance node
    --> /fmu/in/obstacle_distance --> PX4 collision prevention
RViz goal --> software pilot --> /fmu/in/manual_control_input (sticks) --> PX4 Position mode
```

Plan mode:

```
/depth_camera/points --> pointcloud_to_laserscan --> /scan --> Nav2 costmaps
RViz goal --> Nav2 planner --> pure pursuit --> /cmd_vel --> software pilot
    --> /fmu/in/trajectory_setpoint (velocity) --> PX4 Offboard mode
```

PX4 and ROS 2 talk through the Micro XRCE-DDS Agent. PX4 v1.17.0 publishes
27 topics under `/fmu/out/` and subscribes to 38 under `/fmu/in/` (counted on
the running stack, 2026-10-10); `ros2 topic list` shows them. The ROS side is one
launch file: the agent, a `ros_gz_bridge` for the clock and the cloud, the
obstacle node, and one process holding the six RViz-side nodes.

## Why sticks

PX4's collision prevention lives in one place: the stick-to-acceleration
mapper of its manual Position-mode flight task. That mapper, the class
`StickAccelerationXY` in PX4's source, turns horizontal stick deflection into
an acceleration demand, and collision prevention trims that demand. Every
automatic mode, reposition, mission and Offboard alike, bypasses it. So the
only way to fly to a goal with avoidance active is to fly on sticks, and the
pilot does exactly that: it runs the position controller itself and emits
`ManualControlSetpoint`, PX4's stick message, at 50 Hz. PX4 cannot tell this
from a ground station's virtual joystick, because both arrive on the same
topic. The source is checked once, against `COM_RC_IN_MODE`, the parameter
that says which stick inputs to accept; SITL sets it to 1, joystick only.

Fifty hertz is not arbitrary. PX4 declares RC loss after `COM_RC_LOSS_T`,
0.5 s, so the stream must never pause. Plan mode streams zero sticks for the
same reason, even though Offboard ignores them for control.

The pilot asks PX4 for Position mode (or Offboard, in plan mode) only after
something wanted it: start-up, a goal, a mode switch, Nav2 starting to
drive. It repeats the request once a second until PX4 enters the mode, and
never again after PX4 has left it. The first version re-asked whenever PX4
was in any other mode, which pulled the aircraft out of a Land, a Return
or PX4's own Loiter when the obstacle data stopped. Leaving a mode is now
PX4's or a person's decision; sending the mode again takes it back.

Plan mode has no collision prevention underneath it, so the pilot watches
the obstacle histogram itself. If none has arrived for 0.5 s it zeroes
Nav2's velocity and holds altitude, rather than let the planner drive on a
costmap that has stopped updating.

Collision prevention is horizontal only. Its interface takes a 2D vector and
the library contains no 3D version, so the climb and descent of a goal are
unprotected. This is 3D waypoints with 2D avoidance.

## The histogram

`ObstacleDistance` is 72 bins of 5 degrees, each a range in centimetres. The
obstacle node bins the cloud by bearing in the body frame, keeps the nearest
point per bin within a 1 m height band around the aircraft, and publishes at
10 Hz. The camera's 73 degree field fills 15 bins; those are 75 degrees wide
together, so about a degree at each edge is reported as clear without having
been seen.

That describes the stock stack, one camera. The node takes any number of
sensors, point clouds and laser scans, each with its own mount and arc, and
merges them before PX4 sees anything: nearest range per bin, observed arcs
added together. The merging has to happen here because PX4 overwrites
rather than merges two histogram publishers. [extend/sensors.md](extend/sensors.md)
has the sensor list and the measured lidar case.

The message has three bin states, and conflating two of them is the bug to
avoid:

| Bin value | Meaning |
|---|---|
| a range in cm | obstacle seen at that range |
| `max_distance + 1` | observed, nothing in range |
| `UINT16_MAX` | not observed |

Unobserved bins must be `UINT16_MAX`. PX4 then applies `CP_GO_NO_DATA`. At
its default of 0 it refuses to accelerate into any unobserved direction,
which with a 73 degree camera means only forwards. At 1 it moves into them.
This stack sets 1, which is why the aircraft must face where it goes.
The `frame` field must be `MAV_FRAME_BODY_FRD` (the body frame: x forward,
y right, z down), or bin zero means north
rather than forward.

A 640x480 cloud, the camera before the patch in install step 4, is 307,200 points, and processing them all took about 400 ms
per frame, which starved the node to 2 Hz. Every eighth point is plenty for
72 bins. The cloud from Gazebo's bridge is FLU (forward, left, up), x forward, not the ROS
optical convention; measured by reading the extents while facing a wall.

Standoff is a brake, not a hold: at `CP_DIST 2.0` the closest approach over
eight runs ranged from 1.97 to 2.60 m, a 32% spread (the README's status
table has each run). PX4 measures from the sensor, not the propeller tips.

## Frames, and the two that fail silently

PX4 speaks NED for the world (x north, y east, z down) and FRD for the body
(x forward, y right, z down). ROS speaks ENU (east, north, up) and FLU
(forward, left, up). Positions swap axes. Attitude is held as a quaternion,
four numbers that encode a 3D rotation without the singularities of roll,
pitch and yaw. Converting it needs a rotation applied on both sides of the
quaternion, or the aircraft renders upside down in RViz while its heading
still reads correctly. `frames.py` holds both constant rotations as plain
quaternion products, checked against SciPy, the scientific Python library:
they agree to within 1e-7, the precision of the constants written into
`frames.py`.

Stick XY is in the heading frame, not NED: PX4 rotates stick input by the
current yaw before using it. So the pilot rotates its position error by the
heading every tick, or the aircraft flies off at an angle that changes as it
turns.

An identity quaternion in ENU is yaw zero, which is due east, so "hold this
heading" cannot be told from "no preference" by looking at the orientation.
A goal that wants its heading held says so with the frame id `odom+yaw`.

## The pilot's numbers

The pilot is a PD controller on position, not a P controller. A P
(proportional) controller commands stick in proportion to the distance from
the goal; PD adds a derivative term that pushes back in proportion to speed,
which brakes the approach. It is needed because zero stick only brakes
through PX4's drag model, and P alone overshot a 5 m goal by about 3 m.

The yaw stick has a dead band. Measured by sweeping it:

| stick | 0.05 | 0.08 | 0.10 | 0.12 | 0.15 | 0.20 | 0.30 | 0.50 |
|---|---|---|---|---|---|---|---|---|
| deg/s | 0.0 | 0.0 | 0.0 | 1.1 | 2.9 | 5.7 | 12.4 | 28.8 |

Above 0.10 the response is `(stick - 0.10) * 72 deg/s`. Proportional heading
control cannot work through that: the command fades into the dead band as the
error shrinks, and the aircraft froze 14.3 degrees short of every heading,
because that error times the old gain landed exactly on 0.10. The pilot now
commands a capped, damped rate and inverts the measured model. Result: worst
heading error 5.6 degrees over the four cardinal headings.

The XY stick has the same shape: no motion to 0.114, then 5.02 m/s per unit of
stick, so full stick is about 4.4 m/s even though `MPC_VEL_MANUAL` says 10.
Scaling off the parameter would under-command by more than double.

## Why two modes

The obvious design was defence in depth: let Nav2 plan, feed its velocity
through the pilot's sticks, and keep PX4 braking underneath. It does not work,
and the reason is structural. A planner approaches an obstacle in order to go
round it; collision prevention exists to veto motion toward obstacles.
Measured with one A/B comparison, the same goal flown twice with one thing
changed:

| `CP_DIST` | Nav2 commanded | achieved | outcome |
|---|---|---|---|
| 1.0 | 1.50 m/s | 0.00 m/s | deadlocked in front of the wall |
| -1 (off) | 1.50 m/s | 1.24 m/s | rounded the end of the wall |

So the two are exclusive, and the switch is PX4's flight mode rather than a
parameter. Brake mode is Position on sticks with collision prevention live.
Plan mode is Offboard, where PX4 structurally has no collision prevention, so
`CP_DIST` can stay at 2.0 throughout. Verified: plan mode tracked 1.00 m/s
commanded to 1.00 m/s achieved with `CP_DIST 2.0`, the value that had
deadlocked the stick path.

The mode is published on `/avoidance_sim/mode` with `TRANSIENT_LOCAL`
durability, so a late subscriber gets the current value and a switch cannot
be lost to discovery timing. In plan mode the planner owns the heading: an
earlier version slaved it to the direction of travel, which fed the heading
back into its own input and swung it through 120 degrees while the aircraft
barely moved.

## Nav2 for a hovering camera

Four settings are decisions, each from a measured failure:

- No map, no localisation. PX4's EKF already gives metric odometry, so the
  global frame is `odom`, rolling, with no static layer. Nav2 is 2D; altitude
  stays with the pilot.
- `max_obstacle_height` is compared in the `odom` frame, so Nav2's default of
  2.0 m silently discarded every observation from an aircraft at 6 m. Measured:
  135 scan beams on a wall at 9.9 m, lethal cell count frozen for 18 s. Both
  bounds are now wide open at the layer and at the source; the height
  selection happens in `pointcloud_to_laserscan`, relative to the aircraft.
- The global costmap never clears. With clearing on, Nav2 raytraces each
  scan beam, marking every cell the beam passes through as free. With a 73
  degree arc, every cell the scan stops covering is cleared, so turning the
  nose away forgets the wall and the planner draws a line through it.
  Measured: a 7.0 m plan for a 6.9 m straight-line goal with a wall in
  between.
- Regulated pure pursuit, not MPPI. MPPI (model predictive path integral)
  samples thousands of candidate trajectories and blends the cheapest. It is
  the better theoretical fit for a holonomic vehicle, one that can move in
  any horizontal direction without turning first, as a quadrotor can. It
  collapsed to commanding nothing three times, each after a round of tuning,
  with a valid 17 m plan sitting unused. Pure pursuit geometrically chases a
  point on the path and cannot fail that way.

Pure pursuit aborts the whole goal on `detected collision ahead!`, which fires
when the camera marks a wall cell under a path planned a moment earlier. Nav2's
stock trees either fail the goal outright or call Spin and BackUp, which a
flying aircraft with a 73 degree camera must not do. The bundled tree
(`config/avoidance_bt.xml`) clears the local costmap, waits and replans, never
clears the global one, and replans at 2 Hz to halve the race. Its path is set
at the top level of the `bt_navigator` parameters; Jazzy also accepts the
plugin-scoped name and silently ignores it.

Geometry sets the standoff: the camera covers 1.48 times its range in width,
so planning round a 10 m wall needs about 10 m of observation distance.

## PX4's parameters, and why the launch sets them

Four parameters are not at their airframe defaults, and the first was found
by installing on a clean machine, where the aircraft never armed:

| Parameter | Default | Set to | Because |
|---|---|---|---|
| `NAV_DLL_ACT` | 2 | 0 | 2 means wait for a ground station before arming; there is none |
| `NAV_RCL_ACT` | 2 | 0 | The RC-loss failsafe would otherwise fly off to return-to-launch if the sticks paused |
| `CP_DIST` | -1 | 2.0 | Collision prevention is off until set |
| `CP_GO_NO_DATA` | 0 | 1 | Move into unobserved directions; see the histogram above |

> These values are for the simulator only. Three of them switch off a
> protection a real aircraft needs; [hardware.md](hardware.md) lists what changes before
> any real flight.

The launch sets them through PX4's own `px4-param` client once PX4 answers.
`PX4_PARAM_<NAME>` environment variables on the PX4 command would be neater,
and they fail for the one that matters. PX4's startup applies them before the
airframe file and records a value equal to the compiled default (0) as "still
default", so the airframe's later `set-default 2` wins. Measured on a
clean machine: three of four applied, `NAV_DLL_ACT` stayed 2.

## Two more traps

PX4 topic names carry a `_v1` suffix only when the message declares a non-zero
`MESSAGE_VERSION`: `/fmu/out/vehicle_local_position_v1`, but
`/fmu/in/obstacle_distance`. The wrong name fails silently. PX4's publishers
are `BEST_EFFORT` and `VOLATILE`; a subscriber asking for `RELIABLE` or
`TRANSIENT_LOCAL` matches nothing and never receives.

## The bridge process

The six RViz-side nodes share one process and one executor, the loop that
waits for messages and timers and runs their callbacks. It cost 102% of a
core on a `MultiThreadedExecutor`, with the callbacks accounting for a third
of that. The rest was rclpy, ROS 2's Python client library, rebuilding its
wait set (the list of things it is waiting on) over about 75 entities on each
of 300 wake-ups a second. Measured in steps: single-threaded 65%, TF and
odometry throttled from 100 to 30 Hz 60%, parameter services removed 57%,
`rclpy.experimental.EventsExecutor` 18%. The last is the default, with the
single-threaded executor as fallback and `AVOIDANCE_SIM_EXECUTOR=single` to
force it. Both passed the regression gate.
