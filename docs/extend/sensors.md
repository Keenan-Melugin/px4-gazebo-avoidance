# Add a sensor

What is here: the obstacle node takes any number of sensors, each a point
cloud or a laser scan, and merges them. `models/x500_depth_lidar` is the
stock aircraft with a 360 degree 2D lidar (a laser scanner that measures
range in a flat ring around itself) added. `test/histogram_selftest.py`
exercises the merge with made-up sensors and no simulator.

## Why the node merges and PX4 does not

PX4's collision prevention reads one `obstacle_distance` stream and fuses it
with any `distance_sensor` streams (single-beam rangefinders) bin by bin. The
rule is `CollisionPrevention::_enterData` (v1.17.0,
`src/lib/collision_prevention/CollisionPrevention.cpp`): of two sensors that
see a bin, the one with the shorter range wins, and a clear reading replaces
a wall only from the longest-range sensor. It has no rule for two
`obstacle_distance` publishers. Each message overwrites the bins it covers,
so two nodes on the topic alternate and the histogram flickers between them.

A stream that stops is treated as no data after 0.5 s
(`RANGE_STREAM_TIMEOUT_US`). A bin that was once in a sensor's field of view
and is now unknown blocks motion in that direction even with
`CP_GO_NO_DATA 1` (`_checkSetpointDirectionFeasability`, through
`_data_fov`). So the merging is done here, before PX4. The nearest range per
bin wins across the sensors that are alive, and the observed arc is the union
of theirs. A sensor that goes stale, or sends nothing but NaN (not a number,
what a dead renderer produces), drops out of the union.

## Describe a sensor

The `sources` parameter is a space-separated list of names. Each name has
its own parameters:

- `type`: `cloud` or `scan`
- `topic`
- `frame`, cloud only: `flu` as Gazebo publishes, `optical` as real drivers do
  (x right, y down, z forward)
- `hfov_deg`, cloud only (horizontal field of view); a scan carries its arc
- `mount_xyz_frd`, and `yaw_deg`: sensor forward relative to body forward,
  clockwise positive like the histogram's bearings
- `min_distance_cm`, `max_distance_cm`

With `sources` empty the node is the single camera it always was, and a
source named `camera` takes the legacy parameters as defaults, so a second
sensor is described alone. The launch's `lidar:=true` is exactly that:
`config/sensors_lidar.yaml`, a standard ROS parameter file with
`sources: "camera lidar"` and five `lidar` values. `sensors:=/path/to/yours.yaml`
loads any other set, with `config/sensors_example.yaml` as the template, and
`bridge_extra:=` bridges the Gazebo topics it needs ([../data.md](../data.md)
has the full input list).

```bash
python3 test/histogram_selftest.py     # 25 checks, bin by bin, in two seconds
```

Run that before the gate after touching the node. It pins four things. The
single-camera histogram the README was measured with: 15 observed bins, and
a wall 3 m from the camera reported at 312 cm because the camera sits 0.12 m
ahead of the body. The merge: nearest wins, arcs union, stale and dead
sensors leave. A camera yawed 180 degrees, whose left is the body's right.
And a 270 degree scan yawed 90.

## Add a lidar

Three facts decide the model. Gazebo's `gpu_lidar` publishes
`gz.msgs.LaserScan` on its `<topic>` and a `PointCloudPacked` on
`<topic>/points` (gz-sensors 8, `GpuLidarSensor.cc`). Without a `<topic>` the
name is the scoped one, `/world/<w>/model/<m>/link/<l>/sensor/<s>/scan`.

It is a rendering sensor, so it needs the same GPU the depth camera does and
fails the same way in a VM. And PX4's own Gazebo bridge subscribes to a lidar
at one fixed scoped name, `.../link/link/sensor/lidar_2d_v2/scan`, and
publishes `obstacle_distance` from it itself (`GZBridge.cpp`,
`subscribeLaserScan`, enabled by `SIM_GZ_EN_LIDAR`, default 1). That is how
PX4's `x500_lidar_2d` works with no ROS at all, and it would make the second
publisher described above.

`models/x500_depth_lidar/model.sdf` is therefore the stock `x500_depth` plus a
sensor named `lidar` on `lidar_link` with `<topic>lidar</topic>`: not that
name, so the node here stays the only publisher. The other way round is
`PX4_PARAM_SIM_GZ_EN_LIDAR=0` in PX4's environment, which its startup script
applies before the airframe file.

The lidar sits 0.10 m behind and 0.30 m above `base_link`, above the camera
housing and the rotors, with a 0.3 m minimum range that excludes the arms:
360 rays at one degree, 30 m, 10 Hz. The same three numbers appear in
`config/sensors_lidar.yaml` (as the FRD mount), in `frames.py` as `LIDAR_XYZ`
for the static transform RViz needs to place the scan, and in the model file.
Change one, change all three.

## Fly it with the lidar

PX4's build makes a `make px4_sitl gz_<model>` target only for models that
have an airframe file in PX4's tree. An airframe file is the startup script
that sets a vehicle's parameters; `gz_bridge/CMakeLists.txt` (the build
recipe) globs, meaning lists by wildcard, `ROMFS/.../airframes/*_gz_*`. A
sensor variant does not need its own airframe. PX4's startup script takes
`PX4_SYS_AUTOSTART`, the airframe number to load, ahead of the model-name
lookup (`rcS`, line 41), and the `4002_gz_x500_depth` airframe only sets
`PX4_SIM_MODEL` if it is unset. So the binary is run directly, which is also
how PX4 documents multi-vehicle simulation:

```bash
cd ~/PX4-Autopilot && PX4_SYS_AUTOSTART=4002 PX4_SIM_MODEL=gz_x500_depth_lidar \
    PX4_GZ_WORLD=walls HEADLESS=1 ./build/px4_sitl_default/bin/px4
ros2 launch avoidance_sim nav2.launch.py lidar:=true
```

The binary defaults to the build's `rootfs` working directory and to
`etc/init.d-posix/rcS`, so no further arguments. The launch bridges `/lidar`,
gives the obstacle node both sources. In RViz, tick the  display,
which ships switched off, to see the scan in orange.

## Measured with the lidar

The dev machine, 2026-10-07, the walls world, two bring-ups. PX4 attached to
the custom model, Gazebo published `/lidar` at 9.6 Hz (10 set), the node
reported 72 of 72 bins observed, and the merged histogram reached PX4 at
9.6 Hz with no unknown bin. The gate passed in 230 s: headings within 5.5
degrees, standoff 2.24 m (2.57 m on the first bring-up), Nav2 round the wall
with a 6.5 m excursion, ending 0.5 m from the goal.

Then `CP_GO_NO_DATA` was set to 0, the aircraft faced east, and the pilot
asked for a point 5 m to its left. It moved 1.85 m and stopped: 2.6 m short
of a wall the camera could not see, which is collision prevention braking on
the lidar alone. PX4 never printed its refusal. With the camera alone the
same request is refused outright, by the rule in
`_checkSetpointDirectionFeasability`; that refusal is why `CP_GO_NO_DATA 1`
is set in the first place ([../how-it-works.md](../how-it-works.md)).

One finding against. On the first bring-up Nav2's controller hugged the
wall's face at 0.1 m while routing round it, the rotors touched, and the
aircraft tumbled 78 m. The test printed PASS because the tumbling aircraft
crossed the wall line, so it now also requires ending within 4 m of the goal.
It was not seen again in two further plan-half runs with this model (6.5 and
6.4 m excursions). The lidar is not in Nav2's costmap, so the planner was
flying on the camera as before; the lesson is about the controller's margin,
not the sensor. Bandwidth, for the second-machine page: the depth cloud is
21.7 MB/s on the ROS side at this resolution, the lidar scan 29 kB/s, the
stick stream 50.5 Hz.

## A second camera instead

Same mechanism, no new message type. Include `model://OakD-Lite` a second
time in a model, with a `<topic>` of its own and a pose facing backwards.
Bridge that topic with `bridge_extra:=`, and describe it in a sensors file as
`config/sensors_example.yaml` does: `rear`, a cloud, yaw 180, the mount from
its pose. The self-test's third section is that sensor, and
`models/README.md` orders the steps.

## The hook still open: the lidar in Nav2

Collision prevention uses the lidar; Nav2 does not yet, and the README lists
this under what the repo is not. The hook is `config/nav2.yaml`: a second
entry in each costmap's `observation_sources` with `data_type: "LaserScan"`
and `topic: /lidar` (Nav2 Jazzy obstacle layer; both costmaps). What to
measure first is whether the global costmap's never-clear rule, which exists
because the camera forgets, can be relaxed once a sensor sees all round.

## Sources

- PX4-Autopilot v1.17.0: `src/lib/collision_prevention/CollisionPrevention.cpp`
  lines 109 to 150, 228 to 345 and 355 to 367 (`_updateObstacleMap`,
  `_addObstacleSensorData`, `_enterData`, `_checkSetpointDirectionFeasability`)
  and `CollisionPrevention.hpp` line 189 (`RANGE_STREAM_TIMEOUT_US` 500 ms);
  `src/modules/simulation/gz_bridge/GZBridge.cpp` lines 254 to 262 and 853 to
  931 (`subscribeLaserScan`, `laserScanCallback`) and `parameters.c`
  (`SIM_GZ_EN_LIDAR`); `CMakeLists.txt` in the same directory (`gz_<model>`
  targets from the airframe glob); `rcS` lines 41 to 62 (`PX4_SYS_AUTOSTART`)
  and 134 (`PX4_PARAM_` before the airframe file);
  `platforms/posix/src/px4/common/main.cpp` lines 263 to 286 (binary
  defaults); `msg/ObstacleDistance.msg`.
- PX4 user guide v1.17: Collision Prevention (inputs, fusion, timeouts),
  Multi-Vehicle Simulation with Gazebo (the direct binary form).
  https://docs.px4.io/v1.17/en/
- PX4-gazebo-models at `b6127f4`: `x500_depth`, `x500_lidar_2d`, `lidar_2d_v2`.
  https://github.com/PX4/PX4-gazebo-models
- Gazebo Harmonic: gz-sensors 8 `GpuLidarSensor.cc` and `Lidar.cc` (topics,
  frame id), gz-sim 8 `RenderUtil.cc` (default scoped topic).
  https://gazebosim.org/docs/harmonic/sensors/
- ros_gz, jazzy branch: `ros_gz_bridge` README (type table) and
  `parameter_bridge.cpp` (argument syntax). https://github.com/gazebosim/ros_gz
- Nav2 Jazzy: costmap obstacle layer configuration (`observation_sources`,
  `data_type`). https://docs.nav2.org/jazzy/
