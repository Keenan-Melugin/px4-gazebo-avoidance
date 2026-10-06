# Change it

The repository is built to be changed and then measured. This page says where
each thing lives, how to rebuild, and how to prove a change did what you meant
rather than something else.

## Where things live

```
avoidance_sim/
  frames.py             frame conventions, the two shared QoS profiles, quaternion helpers
  obstacle_distance.py  point cloud -> 72-bin histogram. Camera geometry as parameters
  software_pilot.py     the pilot: gains, dead bands, the two modes
  goal_3d.py            the green ball: arrows, ring, right-click menu
  command_marker.py     the orange ball: arm, take off, land, disarm, mode
  world_markers.py      wall outlines, read from the Gazebo world file
  goal_bridge.py        RViz's flat 2D Goal Pose -> PX4 reposition (brake mode only)
  tf_publisher.py       PX4 odometry -> TF and /odom at 30 Hz
  rviz_bridge.py        the process that hosts the six nodes above, and its executor
launch/sim.launch.py    the base stack. Arguments: rviz, agent, agent_cmd, world_sdf, px4_bin, px4_params
launch/nav2.launch.py   the base stack plus Nav2 and pointcloud_to_laserscan
config/nav2.yaml        costmaps, planner, controller, tree. Every non-default is commented with its measurement
config/avoidance_bt.xml the behaviour tree Nav2 runs
config/avoidance.rviz   the RViz layout
scripts/px4_params.sh   the PX4 parameters the launch sets
scripts/prereqs.sh      ROS 2, Gazebo and build tools
scripts/install.sh      agent, px4_msgs, this package, Nav2
patches/                the depth camera resolution change
test/                   the measurement scripts and the gate
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
python3 test/gate.py        # about 6 minutes, needs nav2.launch.py running
```

Run it after any change to the pilot, the frames, the obstacle node or the
Nav2 configuration. It flies the aircraft through the two things this
repository claims. In brake mode: four cardinal headings held, then the
standoff from a wall at `CP_DIST 2.0`. In plan mode: a route round a 10 m wall
to a goal behind it. The exit code counts failures, and each half prints what it
measured, so a regression shows up as a number that moved, not as an opinion.

Two conditions for a result that means anything. Nobody else may be driving
the aircraft; a goal clicked in RViz during a run fights the test for the same
topic. And the start position matters: the gate assumes the aircraft is
somewhere sensible near the origin, facing nothing. `test/README.md` lists the
other nine scripts, each measuring one thing.

## Recipes

**Change the standoff.** `CP_DIST` is a PX4 parameter the launch sets. Pass
`px4_params:="NAV_DLL_ACT=0 NAV_RCL_ACT=0 CP_DIST=3.0 CP_GO_NO_DATA=1"` to the
launch, or `param set CP_DIST 3.0` at the `pxh>` prompt on a running PX4.
Measure with `test/avoid_test.py`, which repositions, flies at a known wall and
prints the gap. Expect a spread of a third of the setpoint.

**Change the camera.** Resolution and rate are in
`patches/px4-camera-res.patch`, applied to the OakD-Lite model in the PX4
tree. Field of view, clip distances, height band and decimation are parameters
of the obstacle node, with the model's values as defaults; pass them with
`-p` or in the launch. If the field of view changes, so does the number of
observed bins and the `1.48 * range` standoff rule for planning.

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

**Use another world.** Pass `world_sdf:=/path/to/world.sdf` so the wall
outlines in RViz match, and set `PX4_GZ_WORLD` to the world's name in the PX4
command. The regression script knows the `walls` world's geometry
(`WALL_FACES_EAST`); a different world needs its own numbers or the standoff
test will report `no wall ahead`.

**Add a node to the bridge.** Write it as a plain `rclpy` node, add the class
to `NODE_TYPES` in `rviz_bridge.py`, and use `PX4_QOS` from `frames.py` for
anything that talks to PX4. Keep callbacks short: the six nodes share one
single-threaded executor, which is why the process costs a sixth of a core
rather than a whole one. Pass `start_parameter_services=False` to the node
constructor as the others do; nothing calls those services and each one is a
waitable the executor pays for.

**Add a mode.** The pilot's modes are strings on `/avoidance_sim/mode`,
published and subscribed with `MODE_QOS`, which is retained. Brake and plan
differ in which PX4 flight mode they request and what they stream; a third
mode would follow the same shape. Add a menu entry in `command_marker.py` and
teach `goal_bridge.py` whether to act on `/goal_pose` in it.

**Replace the sensor.** The obstacle node is the only consumer of the cloud
and the only producer of the histogram. A different sensor means a different
subscriber and the same output, with the three bin states respected: a range,
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
