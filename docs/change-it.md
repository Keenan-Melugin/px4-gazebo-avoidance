# Change it

The repository is built to be changed and then measured. This page says where
each thing lives, how to rebuild, and how to prove a change did what you meant
rather than something else.

## Where things live

```
README.md               the front door: what it is, the Start-here table, the measured status and hardware
COMMANDS.md             every command in the repository, with what goes wrong with each
docs/
  pieces.md             what ROS 2, Gazebo, PX4, the bridge, RViz and Nav2 each are and do here, with a ROS 2 learning path
  install.md            from no operating system to a flying aircraft, every step with a check
  fly.md                flying from RViz in brake mode and plan mode, and what looks like a fault but is not
  how-it-works.md       the two data paths and the measurements behind each design decision
  change-it.md          this page
  extend.md             adding a world, a sensor, an airframe or a second machine
  data.md               flight logs, bags, replay, the input topics, configuration by file, the conditions of a run
  img/                  the three screenshots
avoidance_sim/
  frames.py             frame conventions, the two shared QoS profiles, quaternion helpers
  obstacle_distance.py  point clouds and laser scans -> one 72-bin histogram. Sensors as parameters
  software_pilot.py     the pilot: gains, dead bands, the two modes
  goal_3d.py            the green ball: arrows, ring, right-click menu
  command_marker.py     the orange ball: arm, take off, land, disarm, mode
  world_markers.py      wall outlines, read from the Gazebo world file
  world_geometry.py     the world file's boxes, shared by the markers and the measurement scripts
  goal_bridge.py        RViz's flat 2D Goal Pose -> PX4 reposition (brake mode only)
  tf_publisher.py       PX4 odometry -> TF and /odom at 30 Hz
  rviz_bridge.py        the process that hosts the six nodes above, and its executor
launch/sim.launch.py    the base stack. Arguments: rviz, agent, agent_cmd, world, world_sdf, lidar, sensors, bridge_extra, px4_bin, px4_params
launch/nav2.launch.py   the base stack plus Nav2 and pointcloud_to_laserscan
config/nav2.yaml        costmaps, planner, controller, tree. Every non-default is commented with its measurement
config/avoidance_bt.xml the behaviour tree Nav2 runs
config/avoidance.rviz   the RViz layout
config/sensors_lidar.yaml    the lidar described to the obstacle node; what lidar:=true loads
config/sensors_example.yaml  the template for describing any sensor set (sensors:=)
scripts/px4_params.sh   the PX4 parameters the launch sets
scripts/report.sh       what this machine is and what state the stack is in, for problem reports
scripts/record.sh       a rosbag2 of a run's light topics, the cloud and the lidar on request
scripts/ulog_timeline.py  a run printed from PX4's own flight log: position, mode, histogram, stream gaps
scripts/prereqs.sh      ROS 2, Gazebo and build tools
scripts/install.sh      agent, px4_msgs, this package, Nav2, and the link step below
scripts/link_assets.sh  symlinks worlds/ and models/ into PX4's Gazebo tree, where PX4 insists they live
patches/                the depth camera resolution change
worlds/                 extra worlds: pillars, and the template for the next one; its README is the checklist
models/                 extra aircraft: x500_depth_lidar, plus a PX4 airframe-file template and the README that orders the steps
test/                   the measurement scripts, the gate, histogram_selftest.py (no simulator) and template_measure.py (copy it); its README lists them
package.xml, setup.py, setup.cfg, resource/   the ROS 2 package description and build files colcon reads
```

## The build loop

```bash
cd ~/av_ws
colcon build --packages-select avoidance_sim
source install/setup.bash          # again, after every build
```

`colcon build` exits 0 when it matched no packages, so read the summary line:
it must say `1 package finished`. Then restart the launch; running nodes do
not pick up new code.

## The gate

```bash
python3 test/gate.py                  # about 4 minutes, needs nav2.launch.py running
python3 test/gate.py --world pillars  # the same, in the second world
```

Run it after any change to the pilot, the frames, the obstacle node or the
Nav2 configuration. It flies the aircraft through the two things this
repository claims. In brake mode: four cardinal headings held, then the
closest approach to a wall at `CP_DIST 2.0`. The brake half finds the wall
east of the aircraft from the world file, moves well inside its span, backs
off to 8 m for a run-up, pushes, and stops once the aircraft has stood still.
In plan mode: a route round a 10 m wall to a goal behind it, ending within 4 m
of the goal. The exit code counts failures, and each half prints what it
measured, so a regression shows up as a number that moved, not as an
opinion. Measured: 205 to 255 s for the whole gate on the development machine.

One condition for a result that means anything: nobody else may be driving
the aircraft. A goal clicked in RViz during a run fights the test for the
same topic. Each half positions the aircraft itself, so the start position
no longer matters. `test/README.md` lists the other scripts, each measuring
one thing.

After a change to the obstacle node, before the gate:
`python3 test/histogram_selftest.py`. It needs no simulator, takes two seconds,
and names the bin that went wrong rather than the wall the aircraft hit.

## Recipes

**Change the standoff.** `CP_DIST` is a PX4 parameter the launch sets. Pass
`px4_params:="NAV_DLL_ACT=0 NAV_RCL_ACT=0 CP_DIST=3.0 CP_GO_NO_DATA=1"` to the
launch, or `param set CP_DIST 3.0` at the `pxh>` prompt on a running PX4.
Measure with `test/avoid_test.py`, which repositions, flies at a known wall and
prints the gap. Expect a spread of a third of the setpoint.

**Change the camera.** Resolution and rate are in
`patches/px4-camera-res.patch`, applied to the OakD-Lite model in the PX4
tree. What the obstacle node assumes about the camera (field of view, range,
mount position) goes in a sensors file: copy `config/sensors_example.yaml`,
change the `camera` entry, and launch with `sensors:=/path/to/yours.yaml`.
Height band and decimation are node-wide parameters in the same file. If the
field of view changes, so does the number of observed bins and the
`1.48 * range` standoff rule for planning. Run `test/histogram_selftest.py`
before flying.

**Tune the pilot.** The gains and dead bands are constants at the top of
`software_pilot.py`, each with the measurement that set it. If you change the
yaw model, re-run `test/yaw_threshold.py` (the stick sweep) and
`test/yaw_test.py` (the four headings). For the XY model,
`test/xy_threshold.py`. For anything else, the gate.

**Tune Nav2.** `config/nav2.yaml`, where every non-default value carries the
failure that set it. Then `test/nav2_flight.py`, which prints plans produced,
velocity commands sent, the final position and the sideways excursion, so you
can tell "went round" from "went through" from "sat still". If you change the
behaviour tree, the launch passes its installed path to `bt_navigator`; a tree
that names a behaviour the behaviour server does not load fails to load at
all, and the stack then has no navigator.

**Use another world.** `PX4_GZ_WORLD=name` for PX4, `world:=name` on the
launch and `--world name` for the tests. The wall positions come from the
world file in all three places, through `world_geometry.py`, so nothing is
typed twice. Making a world is in [extend.md](extend.md).

**Add a node to the bridge.** Write it as a plain `rclpy` node, add the class
to `NODE_TYPES` in `rviz_bridge.py`, and use `PX4_QOS` from `frames.py` for
anything that talks to PX4. Keep callbacks short: the six nodes share one
executor, `rclpy.experimental.EventsExecutor` by default with the
single-threaded one as fallback, which is why the process costs a sixth of a
core rather than a whole one. Pass `start_parameter_services=False` to the node
constructor as the others do; nothing calls those services and each one is a
waitable the executor pays for.

**Add a mode.** The pilot's modes are strings on `/avoidance_sim/mode`,
published and subscribed with `MODE_QOS`, which is retained. Brake and plan
differ in which PX4 flight mode they request and what they stream; a third
mode would follow the same shape. Add a menu entry in `command_marker.py` and
teach `goal_bridge.py` whether to act on `/goal_pose` in it.

**Add or replace a sensor.** The obstacle node takes a list of sensors
(`sources`), point clouds or laser scans, and merges them before PX4 sees
any of them, because PX4 cannot merge two histogram publishers. The schema,
the bundled lidar model and the reasons are in [extend.md](extend.md). The
three bin states are the contract whatever the sensor: a range,
`max_distance + 1` for observed-and-clear, `UINT16_MAX` for unobserved.
Getting the last two the wrong way round makes PX4 refuse to move, or move
into things.

## Measuring a change

The pattern used throughout: one variable, same goal, same start, before and
after, and the numbers into a table. The README's status table and the Nav2
A/B in [how-it-works.md](how-it-works.md) are the shape to copy. A change that
cannot be measured this way is a change you cannot tell apart from the
simulation degrading, which it does after an hour or two of flying; restart
PX4 and Gazebo before concluding anything.

## The executor

`AVOIDANCE_SIM_EXECUTOR=single` before the launch forces the bridge onto the
`SingleThreadedExecutor` instead of `rclpy.experimental.EventsExecutor`. Both
passed the gate; the events executor costs a third of the CPU. If the
experimental one misbehaves on a newer rclpy, this is the switch.
