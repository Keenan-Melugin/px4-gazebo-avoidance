# Extend it

[change-it.md](change-it.md) is for changing a number and proving the change.
This page is for adding a thing: a world, a sensor, an airframe, a second
machine. Each section says what PX4 and Gazebo actually do (read in the
source, version given), which files change, how to run it, and what to
measure. The first two have been built and measured. The last two are the
path, with the hooks that still need writing named as such.

## Where PX4 looks, and why there is a link script

PX4 starts Gazebo itself and builds the world path as
`${PX4_GZ_WORLDS}/${PX4_GZ_WORLD}.sdf`, and spawns the aircraft from
`${PX4_GZ_MODELS}/<name>/model.sdf` (PX4 v1.17.0,
`ROMFS/px4fmu_common/init.d-posix/px4-rc.gzsim`). Both variables come from
`gz_env.sh`, which PX4's build generates and the startup script sources, and
it exports them unconditionally to `Tools/simulation/gz/{worlds,models}`
(`src/modules/simulation/gz_bridge/gz_env.sh.in`). Setting them in your shell
therefore changes nothing. `GZ_SIM_RESOURCE_PATH` is appended to rather than
replaced, but it only resolves `model://` URIs inside files; it does not
decide what PX4 starts. PX4's own documentation says the same in fewer words:
worlds and models go in those two directories.

So `scripts/link_assets.sh` symlinks this repository's `worlds/*.sdf` and
`models/*/` into them. `install.sh` runs it. Run it again after adding a
world or model, and after anything that cleans the PX4 tree: `git clean` in
the submodule removes the links, and PX4 then reports a world it cannot find
for a file that is still here. `git status` inside `Tools/simulation/gz`
lists the links as untracked, which is expected.

The other route, for a world that lives nowhere near either repository, is
to start Gazebo yourself and let PX4 attach. The startup script looks for a
running world (`gz topic -l`, a `/world/*/clock` topic) before launching one,
and uses it if it finds one. Source `build/px4_sitl_default/rootfs/gz_env.sh`
first, because that is where the server configuration comes from: the stock
worlds carry no `<plugin>` elements since PX4-gazebo-models #84, and PX4's
`server.config` supplies physics, sensors, IMU, GPS and the rest. Then
`gz sim -r -s /any/path/world.sdf`, then `make px4_sitl gz_x500_depth` as
usual. `world_sdf:=/any/path/world.sdf` on the launch draws its walls.

## 1. A world

What is here: `worlds/pillars.sdf`, the second world and the template, and
`avoidance_sim/world_geometry.py`, which reads a world's boxes for RViz's wall
markers and for the measurement scripts, so a new world needs no numbers
typed anywhere.

### Make one

Copy `pillars.sdf`. Keep everything above the first obstacle: the physics
step, gravity and magnetic field PX4's simulated sensors expect, the ground
plane, the sun and the spherical coordinates that give PX4 its home. Set
`<world name>` to the file's basename. That name is what `PX4_GZ_WORLD` and
`world:=` refer to, and the startup script spawns the aircraft into
`/world/<name>/create`, so a mismatch is a world with no aircraft. Obstacles
are static models with a model-level `<pose>` and a `<box><size>`; that is
the one shape the parser reads. A yawed box is drawn but skipped by the face
queries, and anything that is not a box is drawn by nothing. Then:

```bash
bash scripts/link_assets.sh
python3 -m avoidance_sim.world_geometry pillars --from 0 0 --alt 7 --dir east
```

The second line prints the boxes as the tests will see them and the faces
ahead of a point, which is the check to do before flying anything.

### Fly it

The world name goes to PX4 and to the launch, so the walls RViz draws are
the walls PX4 loaded:

```bash
cd ~/PX4-Autopilot && PX4_GZ_WORLD=pillars HEADLESS=1 make px4_sitl gz_x500_depth
ros2 launch avoidance_sim nav2.launch.py world:=pillars
python3 test/gate.py --world pillars
```

`PX4_GZ_MODEL_POSE="x,y,z,roll,pitch,yaw"` spawns the aircraft somewhere
other than the origin, in metres and radians, missing values zero.

### What the gate assumes about a world

Its brake half flies east from wherever the aircraft is and looks for the
first box face on that line, at that altitude. So a world needs a wall east
of the working area, long enough to be found from anywhere the plan half
parks the aircraft; that is why `wall_east` is 26 m. If there is none the
test says so and does not fly. It backs off to 8 m from the face for a
run-up, pushes, and reads the closest approach, stopping the push once the
aircraft has stood still for 4 s. Pushed on after braking, the aircraft
creeps along the face toward free space, 3 m in 14 s measured, which is why
the end of a fixed push is the wrong thing to read. It also moves at least 4 m inside that wall's span first, because near a wall's
end collision prevention does something else that is also correct. `CP_GUIDE_ANG` (30 degrees) steers the setpoint
toward free space, and the aircraft slides round the end instead of
stopping. Measured once, from 0.5 m inside the walls world's box1 end: it
went round that wall and the next and reached east 100 with nothing left to
brake at.

Its plan half positions at `--start` (east -2, north -9 by default) and asks
for `--goal` beyond the first box north of there. It needs that box's ends
within the camera's view from the start; from 13.5 m back a 73 degree arc is
20 m wide. `python3 test/nav2_flight.py --world x --start E N --goal E N`
moves the scenario.

### Measured on pillars

The dev machine, 2026-10-07: the whole gate passed in 220 s with nothing
changed but `--world pillars`. Brake half: flying east from the origin at
full stick, the aircraft braked 2.04 m from `wall_east`. On the way, the two
pillars 4 m either side of the line pushed it 3.9 m north (`CP_GUIDE_ANG`
again, toward the freer side). Plan half: the 6 m `wall_north` from 13.5 m
back, reached with a 4.2 m sideways excursion and 40 plans, against 6.2 to
7.6 m round the walls world's 10 m wall. Headings within 5.6 degrees, as on
walls.

### What it is for

The walls world taught the numbers in the README: the standoff and its
spread, the 10 m of observation distance a 10 m wall needs. Pillars at the
edge of the arc teach a different thing: a one-metre post fills one bin,
enters the histogram late and off-axis, and collision prevention limits the
velocity component toward it rather than stopping. A world with a person
walking through it is the next one to build, because it is the first that
breaks the never-clearing global costmap ([how-it-works.md](how-it-works.md));
the fix is a measured change, not a setting.

## 2. A sensor

What is here: the obstacle node takes any number of sensors, each a point
cloud or a laser scan, and merges them. `models/x500_depth_lidar` is the
stock aircraft with a 360 degree 2D lidar added. `test/histogram_selftest.py`
exercises the merge with made-up sensors and no simulator.

### Why the node merges and PX4 does not

PX4's collision prevention reads one `obstacle_distance` stream and fuses it
with any `distance_sensor` streams bin by bin. The rule is
`CollisionPrevention::_enterData` (v1.17.0,
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
bin wins across the sensors that are alive, the observed arc is the union of
theirs, and a sensor that goes stale or sends nothing but NaN drops out of
the union.

### Describe a sensor

The `sources` parameter is a space-separated list of names. Each name has
its own parameters:

- `type`: `cloud` or `scan`
- `topic`
- `frame`, cloud only: `flu` as Gazebo publishes, `optical` as real drivers do
- `hfov_deg`, cloud only; a scan carries its arc
- `mount_xyz_frd`, and `yaw_deg`: sensor forward relative to body forward,
  clockwise positive like the histogram's bearings
- `min_distance_cm`, `max_distance_cm`

With `sources` empty the node is the
single camera it always was, and a source named `camera` takes the legacy
parameters as defaults, so a second sensor is described alone. The launch's
`lidar:=true` is exactly that: `config/sensors_lidar.yaml`, a standard ROS
parameter file with `sources: "camera lidar"` and five `lidar` values.
`sensors:=/path/to/yours.yaml` loads any other set, with
`config/sensors_example.yaml` as the template, and `bridge_extra:=` bridges
the Gazebo topics it needs ([data.md](data.md) has the full input list).

```bash
python3 test/histogram_selftest.py     # 25 checks, bin by bin, in two seconds
```

Run that before the gate after touching the node. It pins four things. The
single-camera histogram the README was measured with: 15 observed bins, and
a wall 3 m from the camera reported at 312 cm because the camera sits 0.12 m
ahead of the body. The merge: nearest wins, arcs union, stale and dead
sensors leave. A camera yawed 180 degrees, whose left is the body's right.
And a 270 degree scan yawed 90.

### Add a lidar

Three facts decide the model. Gazebo's `gpu_lidar` publishes
`gz.msgs.LaserScan` on its `<topic>` and a `PointCloudPacked` on
`<topic>/points` (gz-sensors 8, `GpuLidarSensor.cc`). Without a `<topic>` the
name is the scoped one, `/world/<w>/model/<m>/link/<l>/sensor/<s>/scan`. It
is a rendering sensor, so it needs the same GPU the depth camera does and
fails the same way in a VM. And PX4's own Gazebo bridge subscribes to a lidar
at one fixed scoped name, `.../link/link/sensor/lidar_2d_v2/scan`, and
publishes `obstacle_distance` from it itself (`GZBridge.cpp`,
`subscribeLaserScan`, enabled by `SIM_GZ_EN_LIDAR`, default 1). That is how
PX4's `x500_lidar_2d` works with no ROS at all, and it would make the second
publisher described above. `models/x500_depth_lidar/model.sdf` is therefore
the stock `x500_depth` plus a sensor named `lidar` on `lidar_link` with
`<topic>lidar</topic>`: not that name, so the node here stays the only
publisher. The other way round is `PX4_PARAM_SIM_GZ_EN_LIDAR=0` in PX4's
environment, which its startup script applies before the airframe file.

The lidar sits 0.10 m behind and 0.30 m above `base_link`, above the camera
housing and the rotors, with a 0.3 m minimum range that excludes the arms:
360 rays at one degree, 30 m, 10 Hz. The same three numbers appear in
`config/sensors_lidar.yaml` (as the FRD mount), in `frames.py` as `LIDAR_XYZ`
for the static transform RViz needs to place the scan, and in the model file.
Change one, change all three.

### Fly it with the lidar

`make px4_sitl gz_<model>` targets exist only for models with an airframe
file in PX4's tree (`gz_bridge/CMakeLists.txt` globs
`ROMFS/.../airframes/*_gz_*`). A sensor variant does not need its own
airframe. PX4's startup script takes `PX4_SYS_AUTOSTART` ahead of the
model-name lookup (`rcS`, line 41), and the `4002_gz_x500_depth` airframe
only sets `PX4_SIM_MODEL` if it is unset. So the binary is run directly,
which is also how PX4 documents multi-vehicle simulation:

```bash
cd ~/PX4-Autopilot && PX4_SYS_AUTOSTART=4002 PX4_SIM_MODEL=gz_x500_depth_lidar \
    PX4_GZ_WORLD=walls HEADLESS=1 ./build/px4_sitl_default/bin/px4
ros2 launch avoidance_sim nav2.launch.py lidar:=true
```

The binary defaults to the build's `rootfs` working directory and to
`etc/init.d-posix/rcS`, so no further arguments. The launch bridges `/lidar`,
gives the obstacle node both sources, and RViz draws the scan in orange.

### Measured with the lidar

The dev machine, 2026-10-07, the walls world, two bring-ups. PX4 attached to
the custom model, Gazebo published `/lidar` at 9.6 Hz (10 set), the node
reported 72 of 72 bins observed, and the merged histogram reached PX4 at
9.6 Hz with no unknown bin. The gate passed in 230 s: headings within 5.5
degrees, standoff 2.24 m (2.57 m on the first bring-up), Nav2 round the wall
with a 6.5 m excursion, ending 0.5 m from the goal.

Then `CP_GO_NO_DATA` was set to 0, the aircraft faced east, and the pilot
asked for a point 5 m to its left. It moved 1.85 m and stopped: 2.6 m short
of a wall the camera could not see, which is collision prevention braking
on the lidar alone. PX4 never printed its refusal. With the camera alone the
same request is refused outright, by the rule in
`_checkSetpointDirectionFeasability`; that refusal is why `CP_GO_NO_DATA 1`
is set in the first place ([how-it-works.md](how-it-works.md)).

One finding against. On the first bring-up Nav2's controller hugged the
wall's face at 0.1 m while routing round it, the rotors touched, and the
aircraft tumbled 78 m. The test printed PASS because the tumbling aircraft
crossed the wall line, so it now also requires ending within 4 m of the
goal. Not seen again in two further plan-half runs with this model (6.5 and
6.4 m excursions). The lidar is not in Nav2's costmap, so the planner was
flying on the camera as before; the lesson is about the controller's
margin, not the sensor. Bandwidth, for section 4: the depth cloud is
21.7 MB/s on the ROS side at this resolution, the lidar scan 29 kB/s, the
stick stream 50.5 Hz.

### A second camera instead

Same mechanism, no new message type: include `model://OakD-Lite` a second
time in a model, with a `<topic>` of its own and a pose facing backwards,
bridge that topic with `bridge_extra:=`, and describe it in a sensors file
as `config/sensors_example.yaml` does (`rear`, a cloud, yaw 180, the mount
from its pose). The self-test's third section is that sensor, and
`models/README.md` orders the steps.

### Nav2 and the lidar

Not wired yet. The planner still sees only the camera, through
`pointcloud_to_laserscan` and the costmaps' single `scan` source. The hook is
`config/nav2.yaml`: a second entry in each costmap's `observation_sources`
with `data_type: "LaserScan"` and `topic: /lidar` (Nav2 Jazzy obstacle layer;
both costmaps). What to measure first is whether the global costmap's
never-clear rule, which exists because the camera forgets, can be relaxed
once a sensor sees all round.

## 3. An airframe

Not built. What a reader faces today, and the path.

### Where the x500 is assumed

In five places with no list of them:

- the run command names `gz_x500_depth`;
- `patches/px4-camera-res.patch` targets the OakD-Lite model;
- the camera's mount is the obstacle node's default `mount_xyz_frd` and
  `frames.py`'s `CAM_XYZ`;
- `config/nav2.yaml` sets `robot_radius: 0.3`;
- the pilot's gains and dead bands in `software_pilot.py` were measured on
  it (`test/yaw_threshold.py`, `test/xy_threshold.py`).

The hook to write is an
airframe profile: one YAML with the model name, the sensors (in the `sources`
schema above), the radius and the inertial facts. The launch passes it to
every node (`parameters=` takes a file) and into Nav2 through
`nav2_common.launch.RewrittenYaml`, which rewrites named keys of `nav2.yaml`
at launch time and is how Nav2's own bring-up overrides values. The pilot's
gains stay in the pilot; the profile names the two scripts that re-measure
them.

### The PX4 side

A model is a directory under `models/`, linked in as above;
`models/README.md` orders the steps and `models/airframe_template/` holds a
commented PX4 airframe file. If the vehicle
is still a quadrotor with different sensors, start it with
`PX4_SYS_AUTOSTART=4002` as in section 2 and no airframe file. If the vehicle
is different, it needs its own airframe file in PX4's tree:
`ROMFS/px4fmu_common/init.d-posix/airframes/NNNN_gz_<model>` with
`PX4_SIMULATOR`, `PX4_GZ_WORLD`, `PX4_SIM_MODEL` and the vehicle's
parameters. Then a line in that directory's `CMakeLists.txt` and a clean
build (PX4's "Adding a New Airframe Configuration" and the Gazebo page). For
a four-motor VTOL with no cruise motor the starting points exist in PX4's
own models and airframes. `quadtailsitter` with `4018_gz_quadtailsitter`
(`CA_AIRFRAME 4`, `VT_TYPE 0`) is the one that matches; `standard_vtol` with
`4004_gz_standard_vtol` applies if a pusher is added; `tiltrotor` is the
third. Copy the airframe, point it at your model, give it a new number.

### What changes with a VTOL

Collision prevention lives in the multicopter position controller and stops
during transition, so the gate's brake half applies to hover and multicopter
flight only, and the plan half's Offboard velocity setpoints likewise. The
measurements to add are the ones a VTOL forces: the standoff at the real
mass, the yaw dead band of the new airframe (it will not be 0.105), and what
collision prevention does in the seconds around a transition.

### Order

The model flies in Gazebo under PX4's stock airframe before any sensor is
added; then the camera, at its measured mount; then the profile; then the
gate, with the README's status table gaining a row per airframe.

## 4. A second machine

Not built. The ROS side of this package is ordinary ROS 2 Jazzy, and Ubuntu
24.04 on arm64 is a Tier 1 platform with binary packages (REP 2000), so it
runs on a Raspberry Pi 5 as it runs here. The question is what crosses the
network.

### Which side runs where

PX4, Gazebo, the DDS agent and the Gazebo bridge stay with the simulator.
The agent is a DDS participant: PX4's topics appear on the domain for every
machine on it, so nothing on the PX4 side changes (`UXRCE_DDS_AG_IP` stays at
localhost). The Pi runs the obstacle node, the pilot and, if wanted, Nav2,
from the same launch files with `agent:=false`. The hook to write is a
`side:=sim|ros|all` argument that starts only one half.

### Discovery

Same `ROS_DOMAIN_ID` on both machines; PX4's startup script copies it into
`UXRCE_DDS_DOM_ID`. Jazzy's default discovery range is `SUBNET`, which
relies on multicast, and WiFi often drops it; then
`ROS_STATIC_PEERS=<other machine's address>` on both sides names the peer
directly. Fast DDS, Jazzy's default middleware, fragments anything over
64 kB and loses the whole message if one fragment is lost. For the point
cloud over WiFi its documentation recommends
`FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA` (TCP for data, UDP for discovery
only) and raising `net.core.rmem_max` and `wmem_max`. Wired Ethernet first;
the Pi 5 has gigabit.

### Time

`use_sim_time` is set on every node here, and `/clock` from the bridge is an
ordinary topic, so the Pi's nodes follow the simulator's clock with no
further setup.

### What to measure, in this order

1. The cloud's bandwidth, `ros2 topic bw /depth_camera/points`, before moving
   anything. Here it is 21.7 MB/s at 320x240, which is a gigabit link's
   business and not WiFi's; the lidar scan is 29 kB/s.
2. The rate of the stick stream on the simulator side,
   `ros2 topic hz /fmu/in/manual_control_input`, with the pilot on the Pi and
   the Pi loaded. PX4 declares RC loss after `COM_RC_LOSS_T` (0.5 s), and a
   stream that pauses drops the aircraft out of Position mode. That is the
   one safety property this design has.
3. The Pi's CPU for the obstacle node and the pilot; here they cost a sixth
   of a core together, on x86.
4. The camera-to-histogram latency.

### The real aircraft later

The agent moves to the companion computer and talks to the flight controller
over a serial port (`MicroXRCEAgent serial --dev ... -b ...`, PX4's
`UXRCE_DDS_CFG` on TELEM2 with `MAV_1_CONFIG 0`). A human on a transmitter
and this package's synthetic sticks cannot both be the only input.
`COM_RC_IN_MODE` has modes for each and for both with one taking priority;
which one, and how the human takes over, is decided and tested on the bench
before any flight.

## Sources

- PX4-Autopilot v1.17.0: `ROMFS/px4fmu_common/init.d-posix/px4-rc.gzsim`
  (world and model paths, attach to a running world, `PX4_GZ_MODEL_POSE`),
  `rcS` (lines 41 to 62, `PX4_SYS_AUTOSTART` before the model-name lookup;
  line 134, `PX4_PARAM_` applied before the airframe file),
  `src/modules/simulation/gz_bridge/gz_env.sh.in` and `CMakeLists.txt`
  (unconditional exports; `gz_<model>` targets from the airframe glob),
  `src/modules/simulation/gz_bridge/GZBridge.cpp` lines 254 to 262 and 853
  to 931 (`subscribeLaserScan`, `laserScanCallback`) and `parameters.c`
  (`SIM_GZ_EN_LIDAR`), `src/lib/collision_prevention/CollisionPrevention.cpp`
  lines 109 to 150, 228 to 345 and 355 to 367 (`_updateObstacleMap`,
  `_addObstacleSensorData`, `_enterData`, `_checkSetpointDirectionFeasability`)
  and `CollisionPrevention.hpp` line 189 (`RANGE_STREAM_TIMEOUT_US` 500 ms),
  `msg/ObstacleDistance.msg`, `platforms/posix/src/px4/common/main.cpp`
  lines 263 to 286 (binary defaults).
- PX4 user guide v1.17: Gazebo Simulation (environment variables, "Adding
  New Worlds and Models"), Multi-Vehicle Simulation with Gazebo (the direct
  binary form), Collision Prevention (inputs, fusion, timeouts), uXRCE-DDS
  (client start, agent, TELEM2), Adding a New Airframe Configuration.
  https://docs.px4.io/v1.17/en/
- PX4-gazebo-models at the commit v1.17.0 pins (`b6127f4`): `walls.sdf`
  (#52, 2024-08; plugins removed in #84, 2025-03), `x500_depth`,
  `x500_lidar_2d`, `lidar_2d_v2`, `quadtailsitter`, `standard_vtol`,
  `tiltrotor`. https://github.com/PX4/PX4-gazebo-models
- Gazebo Harmonic: gz-sensors 8 `GpuLidarSensor.cc` and `Lidar.cc` (topics,
  frame id), gz-sim 8 `RenderUtil.cc` (default scoped topic, `/scan`),
  `Sensors.cc` (render engine), `Util.cc` (world file resolution);
  https://gazebosim.org/docs/harmonic/sensors/
- ros_gz, jazzy branch: `ros_gz_bridge/README.md` (type table, `/clock`),
  `parameter_bridge.cpp` (argument syntax), `convert/std_msgs.cpp` and
  `utils.cpp` (frame id mapping). https://github.com/gazebosim/ros_gz
- ROS 2 Jazzy: Improved Dynamic Discovery (`ROS_AUTOMATIC_DISCOVERY_RANGE`,
  `ROS_STATIC_PEERS`), About Domain ID, DDS tuning; REP 2000 (Jazzy target
  platforms). https://docs.ros.org/en/jazzy/
- Fast DDS 2.14 documentation: environment variables
  (`FASTDDS_BUILTIN_TRANSPORTS`), "Large Data mode and Fast DDS over TCP",
  "Large Data Rates". https://fast-dds.docs.eprosima.com/en/v2.14.4/
- Nav2 Jazzy: `nav2_common/launch/rewritten_yaml.py`; costmap obstacle layer
  configuration (`observation_sources`, `data_type`). https://docs.nav2.org/jazzy/
