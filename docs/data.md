# Data in, data out, and the conditions of a run

What a run writes on its own, and what you can record on request. How to
look while it flies, and how to drive the aircraft from a script or the
shell. How to configure the stack without editing it, and how to change the
world it flies in. Every command here was run on the development machine on
2026-10-07; the two marked untested were not.

## What a run writes on its own: PX4's flight log

PX4 in simulation logs exactly as it does on an aircraft. Its SITL startup
script sets `SDLOG_MODE 1` (log from boot until the first disarm),
`SDLOG_PROFILE 131` (the default set plus estimator replay plus the vision
and avoidance set) and keeps the last seven sessions (`SDLOG_DIRS_MAX 7`;
PX4 v1.17.0, `ROMFS/px4fmu_common/init.d-posix/rcS`). The files are ordinary
ULog, under the build tree:

```
~/PX4-Autopilot/build/px4_sitl_default/rootfs/log/<date>/<time>.ulg
```

### The size trap, measured

Logging runs until disarm, and in this stack
the aircraft stays armed for as long as you fly it. A six-minute flight is
119 MB; one session of about five and a half hours wrote 6.1 GB; two days of
work left 42 GB in the build tree. `px4-logger off` at any time stops the current log without disarming
(`px4-logger on` starts one), lowering `SDLOG_PROFILE` to 1 drops the replay
and avoidance sets, and `rootfs/log` can be deleted with PX4 stopped. Nothing
else references those files.

### Reading one

`pip install pyulog` gives the command-line tools
(`ulog_info`, `ulog_messages`, `ulog_params`, `ulog2csv`), and
`ulog2ros2bag` turns a log into a ROS 2 bag:

```bash
ulog_info  file.ulg                                  # duration, message types, counts
ulog_params file.ulg | grep CP_                       # the parameters the run flew with
ulog2csv -m obstacle_distance,vehicle_local_position file.ulg   # one CSV per message
```

The histogram PX4 received is in the log as `obstacle_distance`, what it
fused as `obstacle_distance_fused`, the velocity limits it derived as
`collision_constraints`, and the sticks as `manual_control_setpoint` (all
four confirmed in a log from the walls world with `ulog_info`). The
parameters the run flew with are in the log too, which is how a number is
tied to its `CP_DIST` a month later.
PX4's Flight Review (logs.px4.io, or self-hosted from PX4/flight_review)
plots a log in the browser. A log from the simulator is the same format as
one from the aircraft, so any pipeline that ingests flight logs ingests
these; mark the source as simulation.

## Recording the ROS side

PX4's log holds what PX4 saw. The point cloud, the scan, the costmap's view
and the pilot's goals exist only in ROS, and rosbag2 records them. The
default storage on Jazzy is mcap. `scripts/record.sh` records the light
topics:

```bash
scripts/record.sh NAME             # position, status, histogram, sticks, mode, goals, TF, scan, Nav2
scripts/record.sh NAME --cloud     # plus the depth cloud
scripts/record.sh NAME --lidar     # plus the lidar scan
ros2 bag info NAME
```

The cloud is 21.7 MB/s at 320x240 (`ros2 topic bw /depth_camera/points`),
so a minute of it is 1.3 GB; the histogram, position and sticks together
are a few kB/s. `/clock` is recorded on purpose: a bag played back then
drives any node started with `use_sim_time`, which is how the next section
works. Measured during a brake run on the development machine: 13.4 s with
the cloud made a 342 MB bag holding 194 clouds, 3,123 clock ticks, 109
histograms and 666 positions. The recorder reported 83 messages lost on the
transport layer, so the cloud is at the edge of what this disk takes;
without it the same recording is a few megabytes.

### Replaying into the obstacle node with no simulator

The node does not
know where a cloud comes from, so a recorded cloud is a test fixture. With
nothing else running:

```bash
ros2 run avoidance_sim obstacle_distance --ros-args -p use_sim_time:=true
ros2 bag play NAME --topics /clock /depth_camera/points     # another terminal
ros2 topic hz /fmu/in/obstacle_distance                      # a third
```

Change the node, replay the same bag, compare histograms: an iteration loop
of seconds rather than minutes, with the input held constant. Measured: with
PX4 and Gazebo stopped, the node fed from the bag above published the
histogram at 10.0 Hz. Its bins matched what the aircraft had faced when the
bag was recorded: all fifteen forward bins clear, facing west with nothing
within 19 m.

### Reading a run back from PX4's log

`scripts/ulog_timeline.py file.ulg` prints the run every few seconds from
PX4's own log: position, mode, the nearest range in the fifteen bins ahead,
and any gap in the stick and histogram streams. It needs only `pyulog`. It
is how a brake test that ended 50 m from where it should have was
explained. The aircraft braked at 2.0 m, exactly as it should. Then, while
the stick kept pushing, it crept sideways along the face toward free space,
through a gap between two walls and round the end. The test's own output
said "settled at east 56"; the log said what happened in between, and the
test now measures the closest approach instead.

## Looking while it flies

```bash
ros2 topic list                                      # everything the ROS side has
ros2 topic hz  /fmu/out/vehicle_local_position_v1    # about 50 Hz
ros2 topic bw  /depth_camera/points                  # what a link would carry
ros2 topic echo /fmu/in/obstacle_distance --once     # the histogram, as sent
px4-listener obstacle_distance                       # the histogram, as PX4 received it
px4-listener obstacle_distance_fused                 # after PX4 merged its sensors
px4-param show CP_DIST                               # one PX4 parameter
gz topic -l                                          # Gazebo's side
gz model -m x500_depth_0 -p                          # Gazebo's own pose for the aircraft
gz sim -g                                            # attach a Gazebo window to a headless run
scripts/report.sh                                    # machine, versions, running stack
```

`px4-listener` and `px4-param` are PX4's own clients, in
`~/PX4-Autopilot/build/px4_sitl_default/bin`. The pair `gz model -p` and
`ros2 topic echo /fmu/out/vehicle_local_position_v1` is the check for a
diverged estimator: when PX4's position and Gazebo's disagree, every later
number is suspect. Today they agreed to 10 cm after a flight through a wall
that was first blamed on the estimator.

## Driving the aircraft from a script or the shell

The pilot and the planner take their inputs on topics, which is what the
measurement scripts use and what RViz's menus publish. From a script,
`test/regression.py`'s node class `R` already subscribes to position and
status and publishes all of these; reuse it, as `test/template_measure.py`
does.

| Input | Type | Meaning |
|---|---|---|
| `/avoidance_sim/pilot_goal` | `geometry_msgs/PoseStamped` | Fly here in brake mode. `x` east, `y` north, `z` up, metres, in `odom`. `frame_id` `odom` holds position only; `odom+yaw` also holds the heading in `orientation` (ENU yaw); `STOP` drops the goal and centres the sticks |
| `/avoidance_sim/mode` | `std_msgs/String`, reliable, transient local | `brake` or `plan`. A volatile publisher never reaches the pilot's subscriber |
| `/goal_pose` | `geometry_msgs/PoseStamped` | RViz's 2D Goal Pose, brake mode only; becomes a PX4 reposition |
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | A planned route, plan mode only |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2's output into the pilot; publishing it by hand drives velocity mode |
| `/fmu/in/vehicle_command` | `px4_msgs/VehicleCommand` | Arm (command 400, `param1` 1, `param2` 21196 to force), takeoff (22), land (21); `target_system` 1 |

From the shell, the same things in YAML:

```bash
ros2 topic pub --once /avoidance_sim/mode std_msgs/msg/String "{data: brake}" \
    --qos-reliability reliable --qos-durability transient_local
ros2 topic pub --once /avoidance_sim/pilot_goal geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: odom}, pose: {position: {x: 2.0, y: 0.0, z: 7.0}}}"
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: odom}, pose: {position: {x: -2.0, y: 10.0}, orientation: {w: 1.0}}}}" --feedback
```

What a script reads back: `/fmu/out/vehicle_local_position_v1` (`x` north,
`y` east, `z` down, metres; the `_v1` is PX4's message versioning),
`/fmu/out/vehicle_status_v1` (`nav_state` 2 is Position, 14 is Offboard;
`arming_state` 2 is armed), `/odom` (ENU, for Nav2), `/fmu/in/obstacle_distance`,
`/plan`. The list of PX4 topics the bridge carries is
`src/modules/uxrce_dds_client/dds_topics.yaml` in the PX4 tree.

## Configuring without editing

### The launch

Every knob is an argument; `ros2 launch avoidance_sim
sim.launch.py --show-args` lists them. `world:=` and `world_sdf:=` pick the
walls RViz draws; `lidar:=true` adds the lidar; `sensors:=file.yaml` and
`bridge_extra:="..."` describe any sensor set (next paragraph);
`px4_params:="..."` sets PX4 parameters at start; `rviz:=false` and
`agent:=false` leave those out.

### Sensors, in a file

The obstacle node reads a standard ROS parameter
file, keyed by its node name with `ros__parameters` underneath, nested maps
becoming dotted names. `config/sensors_lidar.yaml` is the one `lidar:=true`
loads; `config/sensors_example.yaml` is the commented template with a rear
camera and a side scanner. A sensor on a new Gazebo topic also needs that
topic bridged, in `bridge_extra`, in the bridge's own syntax
(`/topic@ros_type[gz_type`). Check a file without a simulator by adapting
`test/histogram_selftest.py`, then run the gate.

### PX4 parameters

Three ways, in order of persistence. At start,
`px4_params:="NAV_DLL_ACT=0 NAV_RCL_ACT=0 CP_DIST=3.0 CP_GO_NO_DATA=1"` on the
launch (the launch applies them through `px4-param` once PX4 answers, which
is the only route that reaches every parameter; `scripts/px4_params.sh`
says why). While running, `px4-param set CP_DIST 3.0`, or `param set` at the
`pxh>` prompt. To keep a value across restarts, `param save` at `pxh>`; it
writes `build/px4_sitl_default/rootfs/parameters.bson`, which the next start
loads, and which is how the development machine came to differ from a fresh
install for months. `PX4_PARAM_NAME=value` in PX4's environment is applied
before the airframe file, so it cannot set a value the airframe file sets.

### Node parameters

The obstacle node answers `ros2 param set
/obstacle_distance_publisher stale_s 2.0` and `ros2 param dump`. The six
nodes inside `rviz_bridge` do not: they start without parameter services, a
measured CPU saving ([how-it-works.md](how-it-works.md)), so their values
are launch-time only.

## Changing the conditions

### Where it flies

`PX4_GZ_WORLD=name` for PX4 and `world:=name` for the
launch ([extend.md](extend.md) section 1 for making one).
`PX4_GZ_MODEL_POSE="x,y,z,roll,pitch,yaw"` spawns the aircraft elsewhere
(metres and radians, missing values zero); `PX4_HOME_LAT`, `PX4_HOME_LON` and
`PX4_HOME_ALT`, all three together, move the world's origin on the globe.

### How fast

`PX4_SIM_SPEED_FACTOR=2 make px4_sitl gz_x500_depth` asks
Gazebo for twice real time, and PX4's startup script scales its link-loss
timeouts to match. PX4 and Gazebo run in lockstep, so the factor is a
target the machine may not reach, and the script only applies it when it
spawns the model, not when it attaches to a world already running. Untested
here. The thing to check first is the stick stream: the pilot runs on
simulation time, so at factor 2 it must deliver 100 messages per wall-clock
second or PX4 sees RC loss.

### Wind

Gazebo applies wind only to links that opt in with
`<enable_wind>true</enable_wind>`, and only when the world loads the
`WindEffects` system; the stock x500 opts nothing in, so PX4's `windy` world
(5 m/s east, 2 m/s north) leaves it untouched. The recipe is a copy of
`x500_base/model.sdf` with that element on `base_link`, the plugin block from
gz-sim's `examples/worlds/wind.sdf` in the world, and `<wind>` in the world;
wind can then be changed while running by publishing a `gz.msgs.Wind` on
`/world/<name>/wind/`. Untested here.

### What the sensors see

The depth camera's resolution and rate are the
patch in `patches/`, applied to the OakD-Lite model in PX4's tree. Its field
of view (`horizontal_fov`) and range (`<clip>`) are in the same file. A
`<noise type="gaussian"><stddev>` element inside `<camera>` or `<lidar>`
adds measurement noise. The obstacle node's own filters are its parameters:
`height_band_m`, `decimate`, `stale_s`, and per sensor the ranges and arc.

### A sensor that stops

`kill -STOP $(pgrep -f parameter_bridge)` freezes
the bridge; after `stale_s` the node drops the camera, publishes nothing,
and PX4 holds on its own 0.5 s timeout (`kill -CONT` resumes). That is the
dead-camera behaviour the VM run forced, available on demand.

### Battery

PX4 simulates a battery that drains to `SIM_BAT_MIN_PCT` (50%)
over `SIM_BAT_DRAIN` seconds (60) and stays there, so the low-battery action
(`COM_LOW_BAT_ACT 3`, return at critical and land at emergency, set by the
SITL startup script) never fires. Set `SIM_BAT_MIN_PCT` below `BAT_CRIT_THR`
to watch it fire, in flight if you like; PX4's own failsafe-testing page
suggests exactly that.

### Failures

PX4's failure injection is on in SITL (`SYS_FAILURE_EN 1`):

```bash
px4-failure gps off          # also gyro accel mag baro optical_flow vio distance_sensor
px4-failure gps ok           #      airspeed battery motor servo avoidance
px4-failure motor off -i 1   # one instance; 0 means all
```

The types are `ok`, `off`, `stuck`, `garbage`, `wrong`, `slow`, `delayed`,
`intermittent`, and `rc_signal` and `mavlink_signal` are components too.
Sensor failures act inside PX4, so they work in any simulator. Motor and
servo failures need the simulator's cooperation, which PX4's documentation
lists for the old Gazebo only; the x500 model PX4 v1.17.0 pins carries no
motor-failure plugin. Measured here: `failure gps off`, from the client and
from the `pxh>` prompt, printed "inject failure unit: gps" and then "Timeout
waiting for ack", and the position stayed valid. So at this version with
Gazebo Harmonic, failure injection is not a tool this stack has. The ROS-side
substitutes above (freeze the bridge, stop a node) are.

### Light and time of day

The world's `<light>` is a directional sun;
the depth camera and the lidar are range sensors and do not care. Change it
for pictures, not for measurements.

## Sources

- PX4-Autopilot v1.17.0: `ROMFS/px4fmu_common/init.d-posix/rcS` (logging,
  battery, failure and low-battery defaults, `parameters.bson`, `PX4_PARAM_`
  overrides, speed-factor scaling), `px4-rc.gzsim` (model pose, home
  position, `set_physics`), `src/modules/logger/module.yaml` (the
  `SDLOG_PROFILE` bits), `src/modules/simulation/battery_simulator/battery_simulator_params.c`,
  `src/modules/uxrce_dds_client/dds_topics.yaml`; the `failure` command's
  own usage text (`px4-failure help`).
- PX4 user guide v1.17: Logging (https://docs.px4.io/v1.17/en/dev_log/logging.html:
  ULog, `SDLOG_MODE`, `SDLOG_PROFILE`, `logger on` and `off`), ULog File
  Format, Flight Log Analysis (Flight Review, PlotJuggler, pyulog), System
  Failure Injection (https://docs.px4.io/v1.17/en/debug/failure_injection.html:
  components, types, the simulator caveat), Simulate Failsafes
  (https://docs.px4.io/v1.17/en/simulation/failsafes.html: the battery that
  never runs out, `SIM_BAT_MIN_PCT` in flight), Gazebo Simulation
  (https://docs.px4.io/v1.17/en/sim_gazebo_gz/: `PX4_SIM_SPEED_FACTOR` and
  lockstep, `PX4_GZ_MODEL_POSE`, `PX4_HOME_*`), ROS 2 User Guide and the
  message translation node page (topic version suffixes).
- pyulog 1.2.4 (https://github.com/PX4/pyulog): `ulog_info`, `ulog_params`,
  `ulog2csv`, `ulog2ros2bag`. Flight Review: https://github.com/PX4/flight_review
- rosbag2, jazzy branch: README (mcap is the default storage), the record
  and play option help in `ros2bag/verb/record.py` and `play.py`
  (`--clock` excludes a recorded `/clock`; `--topics`, `-r`, `-l`).
  https://github.com/ros2/rosbag2/tree/jazzy; the Jazzy tutorial Recording
  And Playing Back Data, https://docs.ros.org/en/jazzy/
- ROS 2 Jazzy tutorials: Understanding Topics (`ros2 topic pub --once`),
  Understanding Actions (`ros2 action send_goal`), Understanding Parameters
  (`ros2 param set`, `dump`, `load`), the Node arguments guide (parameter
  file layout, nested maps as dotted names, `--params-file`), Using ROS 2
  launch for large projects (`parameters=[path]`). https://docs.ros.org/en/jazzy/
- Nav2 jazzy: `nav2_msgs/action/NavigateToPose.action`. https://api.nav2.org/actions/jazzy/navigatetopose.html
- Gazebo Harmonic (gz-sim 8, SDFormat 1.11): `world.sdf` `<wind>`, `link.sdf`
  `enable_wind`, `examples/worlds/wind.sdf` and the `WindEffects` system
  (https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1WindEffects.html);
  `camera.sdf` and `lidar.sdf` (`horizontal_fov`, `<clip>`, `<noise>`);
  `physics.sdf` and the `/world/<name>/set_physics` service (UserCommands);
  the `gz topic`, `gz service` and `gz model` tools; https://gazebosim.org/docs/harmonic/
- ros_gz jazzy: `ros_gz_bridge` README (type table, including `gz.msgs.Image`),
  `ros_gz_image` for images through image_transport. https://github.com/gazebosim/ros_gz
