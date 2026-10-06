# How it works

Two data paths, one aircraft. This page is what you need to know to read the
code, in the order the data flows, with the measurements that shaped each
decision. Every number is from this stack unless it says otherwise.

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

PX4 and ROS 2 talk through the Micro XRCE-DDS Agent. PX4 publishes about 27
topics under `/fmu/out/` and subscribes under `/fmu/in/`. The ROS side is one
launch file: the agent, a `ros_gz_bridge` for the clock and the cloud, the
obstacle node, and one process holding the six RViz-side nodes.

## Why sticks

PX4's collision prevention lives in one place: the stick-to-acceleration
mapper of its manual Position-mode flight task (`StickAccelerationXY`). Every
automatic mode, reposition, mission and Offboard alike, bypasses it. So the
only way to fly to a goal with avoidance active is to fly on sticks, and the
pilot does exactly that: it runs the position controller itself and emits
`ManualControlSetpoint` at 50 Hz. PX4 cannot tell this from a ground
station's virtual joystick, because both arrive on the same topic and the
source is checked once, against `COM_RC_IN_MODE`, which SITL sets to 1.

Fifty hertz is not arbitrary. PX4 declares RC loss after `COM_RC_LOSS_T`,
0.5 s, so the stream must never pause. Plan mode streams zero sticks for the
same reason, even though Offboard ignores them for control.

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
The `frame` field must be `MAV_FRAME_BODY_FRD`, or bin zero means north
rather than forward.

A 640x480 cloud is 307,200 points, and processing them all took about 400 ms
per frame, which starved the node to 2 Hz. Every eighth point is plenty for
72 bins. The cloud from Gazebo's bridge is FLU, x forward, not the ROS
optical convention; measured by reading the extents while facing a wall.

Standoff is a brake, not a hold: at `CP_DIST 2.0` three runs stopped 1.98,
2.05 and 2.60 m out, a 31% spread. PX4 measures from the sensor, not the
propeller tips.

## Frames, and the two that fail silently

PX4 speaks NED for the world and FRD for the body. ROS speaks ENU and FLU.
Positions swap axes; attitude needs a rotation applied on both sides of the
quaternion, or the aircraft renders upside down in RViz while its heading
still reads correctly. `frames.py` holds both constant rotations as plain
quaternion products, checked against scipy to 1e-15.

Stick XY is in the heading frame, not NED: PX4 rotates stick input by the
current yaw before using it. So the pilot rotates its position error by the
heading every tick, or the aircraft flies off at an angle that changes as it
turns.

An identity quaternion in ENU is yaw zero, which is due east, so "hold this
heading" cannot be told from "no preference" by looking at the orientation.
A goal that wants its heading held says so with the frame id `odom+yaw`.

## The pilot's numbers

PD on position, not P: zero stick only brakes through PX4's drag model, and
P alone overshot a 5 m goal by about 3 m.

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
round it; collision prevention exists to veto motion toward obstacles. Measured with one A/B, same goal:

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
- The global costmap never clears. With a 73 degree arc and clearing on,
  every cell the scan stops covering is raytraced free, so turning the nose
  away forgets the wall and the planner draws a line through it. Measured: a
  7.0 m plan for a 6.9 m straight-line goal with a wall in between.
- Regulated pure pursuit, not MPPI. MPPI is the better theoretical fit for a
  holonomic vehicle and it collapsed to commanding nothing three times, each
  after a round of tuning, with a valid 17 m plan sitting unused. Pure pursuit
  geometrically chases a point on the path and cannot fail that way.

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

The six RViz-side nodes share one process and one executor. It cost 102% of a
core on a `MultiThreadedExecutor`, with the callbacks accounting for a third
of that; the rest was rclpy rebuilding its wait set over about 75 entities on
each of 300 wake-ups a second. Measured in steps: single-threaded 65%, TF and
odometry throttled from 100 to 30 Hz 60%, parameter services removed 57%,
`rclpy.experimental.EventsExecutor` 18%. The last is the default, with the
single-threaded executor as fallback and `AVOIDANCE_SIM_EXECUTOR=single` to
force it. Both passed the regression gate.
