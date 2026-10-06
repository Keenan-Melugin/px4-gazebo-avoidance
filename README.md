# px4-gazebo-avoidance

Depth-camera obstacle avoidance for PX4 in Gazebo, flown from RViz.

A ROS 2 node turns the simulated depth camera's point cloud into the polar
obstacle histogram PX4 expects, and a software pilot flies the aircraft to
clicked waypoints by streaming synthetic manual control, which is what keeps
it in the one flight mode where PX4 applies collision prevention.

Fly it at a wall and it brakes. Switch avoidance off and it hits the wall.

## What was measured

All of this was measured on one machine: Windows 11 with WSL2 Ubuntu 24.04, an
AMD RX 7800 XT, 12 cores. Where a number is a single run, it says so.

| Thing | Result |
|---|---|
| Collision prevention standoff | Stopped **1.98 m**, **2.05 m** and **2.60 m** short of a wall at `CP_DIST 2.0`, in three runs |
| With avoidance off (`CP_DIST -1`) | Reached the wall face and collided |
| Depth camera correctness | Reported **2.900 m** against a wall placed at 2.90 m, on both Gazebo render backends |
| Heading control | Worst error **5.5 degrees** over one pass through 0, 90, 180 and -90 degrees. The pilot accepts arrival within 8 degrees, so expect up to that |
| Path straightness while turning | 0.32 m lateral drift on one 10 m leg while rotating 90 degrees |
| Simulation speed | Real-time factor **1.00** headless (five samples, 0.9987 to 1.0004), **0.55 to 0.89** with the Gazebo GUI open |
| Software rendering, for contrast | Real-time factor **0.033** |
| Nav2 route around a 10 m wall, plan mode | Goal reached in **43 s**, 6.2 m detour; earlier run 70 s, 11.2 m. Two runs |

It brakes rather than holding a set distance: the standoff spread 0.6 m
across three runs at the same `CP_DIST`, which is 31% of the setpoint, so
treat `CP_DIST` as approximate. All three are measured from the body origin,
and PX4's own documentation notes that `CP_DIST` is the distance to the
sensor, not to the propeller disc.

The obstacle histogram's geometry is arithmetic rather than a measurement: 15
of 72 bins fall inside the camera's 73 degree field of view. Those 15 bins are
5 degrees wide and centred, so they span 75 degrees, meaning about a degree at
each edge is reported as clear without having been seen.

## What this is not

**Avoidance here is a hover capability.** It brakes, it does not plan around
things. PX4's collision prevention is horizontal only, runs in Position mode
only (strictly, Position and Position Slow), and stops during VTOL transition.

**The camera sees 73 degrees ahead, and nothing else.** The other 57 bins are
honestly reported as unknown, and PX4 will not accelerate into a direction it
cannot see unless you set `CP_GO_NO_DATA 1`. So the aircraft has to be pointed
roughly where it is going.

**This is simulation.** Nothing here has run on real aircraft hardware.

**The airframe is PX4's `x500_depth`**, an x500 quadrotor with a simulated
OAK-D Lite. Plain `x500` has no camera and will not work.

A separate claim worth marking as not measured here: for a transitioning or
fixed-wing aircraft at cruise speed, a stereo sensing-range argument says the
detection distance does not support avoidance at all. That is an argument from
sensor geometry, not a result from this stack.

## Requirements

- **Ubuntu 24.04**. Not Raspberry Pi OS: the ROS index for Debian bookworm has
  63 packages and zero `ros-jazzy-*`.
- **ROS 2 Jazzy**, **Gazebo Harmonic** (8.x)
- **PX4 v1.17.0** and **px4_msgs release/1.17**
- **Micro XRCE-DDS Agent v2.4.3** from eProsima. Not `micro-ros-agent`, which
  is a different thing and will not work.
- Hardware-accelerated OpenGL. See the real-time factor numbers above.

`scripts/prereqs.sh` installs the ROS 2 and Gazebo line on a clean Ubuntu
24.04. PX4 and the agent are Install steps 2 and 3.

## Install

### 1. ROS 2 Jazzy and Gazebo Harmonic

```bash
git clone https://github.com/Keenan-Melugin/px4-gazebo-avoidance.git ~/px4-gazebo-avoidance
~/px4-gazebo-avoidance/scripts/prereqs.sh
```

On a clean Ubuntu 24.04 this adds the ROS 2 and Gazebo apt repositories and
installs `ros-jazzy-desktop`, `ros-dev-tools`, `gz-harmonic` and
`ros-jazzy-ros-gz`, following the two projects' own install pages command for
command; the links are at the top of the script. Each step is skipped when
its result is already there, so it is safe on a machine that has some of
this. It runs a full `apt upgrade` first because the ROS 2 page asks for one.
Afterwards, every new terminal needs `source /opt/ros/jazzy/setup.bash`.

### 2. PX4

`install.sh` deliberately does not do this, because it is a firmware build
with its own setup script.

```bash
git clone -b v1.17.0 --recursive https://github.com/PX4/PX4-Autopilot.git ~/PX4-Autopilot
cd ~/PX4-Autopilot && bash Tools/setup/ubuntu.sh    # then log out and back in
make px4_sitl                                        # slow the first time
```

`--recursive` is load-bearing. The `walls` world used below lives in the
`Tools/simulation/gz` submodule, and a non-recursive clone will not have it.

On arm64, `Tools/setup/ubuntu.sh` needs `--no-nuttx`, because `gcc-multilib`
does not exist for that architecture.

### 3. This package

```bash
cd ~/px4-gazebo-avoidance
./scripts/install.sh
```

It prints what it will write and where, asks for sudo up front, and stops at
the first failure. It builds the agent and `px4_msgs` into `~/av_ws` (override
with `WS=...`), runs `rosdep` and builds this package.

`px4_msgs` has no binary package on any architecture, confirmed by its absence
from a `release:` block in `rosdistro`, so it is always a source build: 235
messages and one service. Minutes on a desktop. The script drops to two
compiler jobs automatically under 8 GB of RAM; add swap if it is still killed.

### 4. Optional: the depth camera patch

`patches/px4-camera-res.patch` drops the simulated camera from 640x480 at
30 Hz to 320x240 at 15 Hz. **The measurements above were taken with it
applied.** It is worth applying on a slower machine and especially on arm64.

```bash
cd ~/PX4-Autopilot/Tools/simulation/gz
git apply ~/px4-gazebo-avoidance/patches/px4-camera-res.patch
```

Note the path: it applies inside the `Tools/simulation/gz` submodule, not at
the PX4 root, and it will leave that submodule dirty.

## Run

Every command in this repository, with what it does and what goes wrong, is in
[COMMANDS.md](COMMANDS.md). The short version follows.

Two terminals. Source ROS and the workspace in both.

```bash
source /opt/ros/jazzy/setup.bash
source ~/av_ws/install/setup.bash
```

Terminal 1, PX4 and Gazebo:

```bash
cd ~/PX4-Autopilot
PX4_PARAM_NAV_DLL_ACT=0 PX4_PARAM_NAV_RCL_ACT=0 PX4_PARAM_CP_DIST=2.0 PX4_PARAM_CP_GO_NO_DATA=1 \
PX4_GZ_WORLD=walls HEADLESS=1 make px4_sitl gz_x500_depth
```

`HEADLESS=1` suppresses the Gazebo GUI and buys back 10 to 45% of real-time
factor. Drop it if you want to watch in Gazebo as well as RViz.

Terminal 2, the ROS 2 side:

```bash
ros2 launch avoidance_sim sim.launch.py
```

This also starts the Micro XRCE-DDS Agent on UDP 8888. If you already run one,
pass `agent:=false`, or you will get a port collision. `rviz:=false` skips
RViz.

PX4 is not in the launch file on purpose: starting it means running a make
target from the PX4 source tree, and burying a build inside a launch file
makes failures hard to read.

The four `PX4_PARAM_` variables are PX4 parameters. PX4's own SITL startup
script applies any `PX4_PARAM_<NAME>` it finds in the environment with
`param set`, so this is the same as typing them at the `pxh>` prompt, without
the typing. All four are needed:

| Parameter | Why it is in the command |
|---|---|
| `NAV_DLL_ACT=0` | The x500 airframe defaults this to 2: refuse to arm until a ground station connects. There is no ground station here, so without it PX4 repeats `Preflight Fail: No connection to the GCS` and nothing you do in RViz will fly. Found on the first clean-machine install; the development machine had it saved from months before |
| `NAV_RCL_ACT=0` | The RC-loss failsafe. The pilot's synthetic sticks are the RC link, and if they ever pause this stops PX4 flying off to return-to-launch |
| `CP_DIST=2.0` | The collision-prevention standoff in metres. Avoidance is off until it is set; `-1` disables it |
| `CP_GO_NO_DATA=1` | The camera sees 73 degrees, so 57 of the 72 obstacle bins are honestly unknown. At the default of 0 PX4 refuses to accelerate in any direction it cannot see, and sideways or backwards goals are silently ignored |

They persist in `~/PX4-Autopilot/build/px4_sitl_default/rootfs/parameters.bson`,
so after the first run they stay set; keeping the variables in the command is
harmless and makes the command self-contained.

### Is it working?

Terminal 1 prints `Ready for takeoff!` within about 30 s. If it repeats
`Preflight Fail: No connection to the GCS` instead, `NAV_DLL_ACT` did not take.

```bash
ros2 topic hz /fmu/out/vehicle_local_position_v1   # about 50 Hz
```

The obstacle node logs `camera sees 15 of 72 bins` at startup. If it does not
appear, the depth cloud is not arriving.

## Fly it

In RViz, on the green goal ball:

- **Drag the three arrows** for east, north and altitude.
- **Drag the ring** to set the heading it will hold. The yellow arrow shows it.
- **Right-click the ball** for `FLY HERE (avoidance ON)`. There is also
  `FLY HERE (direct, NO avoidance)`, which uses PX4's own reposition and turns
  off the thing this repository is about, plus `STOP` and `snap to aircraft`.
- **Right-click the orange ball** above the aircraft for `ARM`, `TAKEOFF`,
  `LAND`, `DISARM` and `DISARM (FORCE, in air)`.

Arm first, then fly, and in that order for a concrete reason: PX4 refuses to
arm while the throttle stick is above center, and a goal the aircraft has not
reached holds it there. If arming is denied with `throttle above center`, use
`STOP` on the goal menu to release the pilot, then arm.

Give PX4 time before the first arm, too. For tens of seconds after it starts
it reports `Preflight Fail: No valid data from Baro 0` and `ekf2 missing
data`, and arming is denied with `Resolve system health failures first`. Wait
for `Ready for takeoff` in the PX4 console.

Do not use `TAKEOFF` while the pilot is running either: it streams sticks
continuously to hold Position mode, and that overrides the automatic
takeoff.

One surprise worth knowing: with `CP_DIST` set, PX4 forces Loiter if the
obstacle stream stops for five seconds. Stopping the ROS stack mid-flight will
change the aircraft's mode.

## Path planning with Nav2

The base stack brakes for obstacles. A second, opt-in layer plans around them:

```bash
ros2 launch avoidance_sim nav2.launch.py
```

**Status: reaches goals.** Measured twice on the walls world, both from
13.5 m south of a 10 m wall to a goal 5.5 m north of it:

| Run | Result | Time from acceptance | Detour |
|---|---|---|---|
| Recovery tree, costmap still height-filtered | goal reached, (−2.25, 9.67) | ~70 s | 11.2 m, west end |
| Recovery tree, costmap fixed | goal reached, (−1.51, 10.06) | **43 s** | **6.2 m, east end** |

The second run chose the shorter way round because it could see both ends of
the wall from the start. Two runs on one obstacle layout is evidence, not a
guarantee; the limits below still apply.

Four things had to be fixed for that, and every one was a measurement rather
than a guess. They are written up in `config/nav2.yaml`,
`config/avoidance_bt.xml` and the `nav2.launch.py` docstring:

- **The costmap could not see at altitude.** `max_obstacle_height` is
  compared in the `odom` frame, so Nav2's default of 2.0 m discarded every
  observation from an aircraft at 6 m. Measured: 135 scan beams on a wall at
  9.9 m, lethal count frozen for 18 s. Every earlier run was planning against
  marks made during moments below 2 m.
- **Pure pursuit aborts the goal on `detected collision ahead!`**, which fires
  when the camera marks a wall cell under a path planned a moment earlier. The
  bundled behaviour tree clears the local costmap, waits, and replans instead
  of failing, and never clears the global costmap, which is the only memory of
  walls the camera is not facing.
- **Plan mode must stream zero sticks.** Stopping them made PX4 declare RC
  loss in 0.5 s, before the 1.2 s Offboard warm-up finished.
- **Mode switches are retained** (`TRANSIENT_LOCAL`), a stick goal in plan
  mode switches back to brake rather than being silently ignored, and brake
  mode keeps asking for Position mode until it gets it.

**One finding from it is solid and matters more than the feature.** PX4
collision prevention and a path planner cannot both own the same axis. A
planner approaches obstacles deliberately in order to get around them, and
collision prevention exists to veto motion toward obstacles, so the lower
layer vetoes the plan. Measured with one A/B, identical goal:

| `CP_DIST` | Nav2 commanded | aircraft achieved | outcome |
|---|---|---|---|
| 1.0 | 1.50 m/s | **0.00 m/s** | deadlocked in front of the wall |
| -1 (off) | 1.50 m/s | **1.24 m/s** | rounded the end of the wall |

Rather than making you reconfigure PX4, that choice is a runtime mode. Publish
on `/avoidance_sim/mode`, or use the RViz right-click menu:

| Mode | PX4 flight mode | Who avoids |
|---|---|---|
| `brake` | Position | PX4 collision prevention, the whole mechanism |
| `plan` | Offboard | Nav2. PX4 has no collision prevention in Offboard |

Switching the flight mode rather than `CP_DIST` is what makes this clean:
nothing needs reconfiguring, because collision prevention does not apply in
Offboard at all. Verified by leaving `CP_DIST` at 2.0 for a whole test, the
value that previously deadlocked the planner, and watching plan mode track
1.00 m/s commanded to 1.00 m/s achieved.

```bash
ros2 topic pub --once /avoidance_sim/mode std_msgs/msg/String "{data: plan}"
```

Collision prevention is a manual-flight assist, not a composable safety
layer.

Two more things measured here, both consequences of the sensor rather than the
software:

- **Standoff is geometry.** The camera sees a 73 degree arc, so at range R it
  covers 1.48*R of width. Planning around a 10 m wall needs roughly 10 m of
  observation distance. From 3.5 m it sees 5 m of wall, neither end, and can
  never find a route it has not observed.
- **The global costmap must not clear.** With a 73 degree arc and clearing on,
  every cell the scan stops covering is raytraced clear, so turning the nose
  away forgets the wall and the planner draws a straight line through it.
  Before the fix: a 7.0 m plan for a 6.9 m straight-line goal with a wall in
  between.

## Platform support

| Platform | Status |
|---|---|
| **Windows + WSL2, Ubuntu 24.04** | **Tested.** Everything above was measured here. Needs `GALLIUM_DRIVER=d3d12`, which is WSL-only |
| Linux x86_64, Ubuntu 24.04 | Untested, but nothing here is WSL-specific. Set no renderer variables: Mesa picks the right driver itself |
| Raspberry Pi 5, Ubuntu 24.04 arm64 | **Untested.** See below |
| Raspberry Pi OS | No. No ROS 2 Jazzy binary packages exist for Debian bookworm |
| macOS | Not attempted. Jazzy lists macOS as Tier 3, source build only, no binaries. An Ubuntu 24.04 arm64 VM is the practical route |

**The Pi case is untested.** Nothing here has run on Pi hardware. What is
known: the Pi's Mesa V3D driver is reported to cap desktop OpenGL at 3.1,
while Gazebo's default `ogre2` backend requires 3.3, so `ogre2` is not
expected to start. Three third-party reports have `--render-engine ogre`
working on a Pi 5. Measured here on x86, not on a Pi: the `ogre` backend
aborts with `Unable to open display` when no X server is present, while
`ogre2` works headless through EGL. Together that suggests a Pi needs a real
display, but nobody has confirmed it.

If you try it, PX4 has the switch built in:

```bash
PX4_GZ_SIM_RENDER_ENGINE=ogre \
PX4_PARAM_NAV_DLL_ACT=0 PX4_PARAM_NAV_RCL_ACT=0 PX4_PARAM_CP_DIST=2.0 PX4_PARAM_CP_GO_NO_DATA=1 \
PX4_GZ_WORLD=walls make px4_sitl gz_x500_depth
```

Expect the `px4_msgs` build to take far longer than on a desktop and to risk
the out-of-memory killer. A Pi is a better companion computer than a
simulation host: Ubuntu 24.04 arm64 is a Tier 1 ROS 2 Jazzy platform, so the
nodes here should run on one against a simulation hosted elsewhere.

## Traps

Each of these cost real debugging time, and none produces a useful error.

**The frame conversion fails silently.** PX4 is NED and FRD, ROS is ENU and
FLU. Swapping the position axes alone looks right in a plot and is still
wrong: attitude needs a rotation applied on *both* sides, or the aircraft
renders upside down while its heading still reads correctly.

**Stick XY is in the heading frame, not NED.** PX4 rotates stick input by the
current heading before using it, so a position controller has to rotate its
error the same way every cycle, or the aircraft flies off at an angle that
changes as it turns.

**The yaw stick has a dead band.** Measured by sweeping it:

| stick | 0.05 | 0.08 | 0.10 | 0.12 | 0.15 | 0.20 | 0.30 | 0.50 |
|---|---|---|---|---|---|---|---|---|
| deg/s | 0.0 | 0.0 | 0.0 | 1.1 | 2.9 | 5.7 | 12.4 | 28.8 |

The dead band runs to about 0.11, and above it the response is near-linear at
roughly 73 deg/s per unit of stick. Proportional heading control cannot work
through that: the command fades into the dead band as the error shrinks. Ours
froze **14.3 degrees** short of every commanded heading, repeatably, until it
was changed to command a rate through this table instead.

**`ObstacleDistance` has three bin states, not two.** A range in centimetres,
`max_distance + 1` meaning seen and clear, and `UINT16_MAX` meaning unknown.
Using `UINT16_MAX` for bins that are clear is the bug to avoid. Using it for
bins the sensor genuinely cannot see is correct, which is what this node does,
and the consequence is the `CP_GO_NO_DATA` setting above. The `frame` field
also has to be `MAV_FRAME_BODY_FRD`, or bin zero means north, not forward.

**PX4 topic names carry a version suffix only when `MESSAGE_VERSION` is
declared and non-zero.** `VehicleLocalPosition` is
`/fmu/out/vehicle_local_position_v1`; `ObstacleDistance` has no suffix.
Subscribing to the wrong name fails silently.

**PX4's publishers are `BEST_EFFORT` and `VOLATILE`.** A subscriber asking for
`RELIABLE` or `TRANSIENT_LOCAL` matches nothing and never receives.

**Two different meanings of headless.** `HEADLESS=1` only suppresses the
Gazebo GUI; the server still renders the depth camera. Gazebo's separate
`--headless-rendering` EGL path is ogre2-only, and PX4 never passes it. So
`ogre` with no display cannot work either way.

## Layout

```
avoidance_sim/      the ROS 2 package
  frames.py           frame conventions, shared QoS, helpers
  obstacle_distance.py  point cloud -> 72-bin polar histogram
  software_pilot.py     flies to a goal on synthetic sticks
  goal_3d.py            draggable 3D waypoint with its heading ring
  command_marker.py     the right-click menu: brake/plan mode, arm, takeoff, land
  world_markers.py      draws the world's obstacles in RViz
  goal_bridge.py        RViz's flat Goal Pose tool -> PX4 reposition
  tf_publisher.py       PX4 odometry -> the TF tree
  rviz_bridge.py        composes the six RViz-side nodes in one process
launch/sim.launch.py  brake mode: PX4 collision prevention on synthetic sticks
launch/nav2.launch.py plan mode: the above plus Nav2 and pointcloud_to_laserscan
config/avoidance.rviz the RViz layout, with the Nav2 panel and overlays
config/nav2.yaml      costmaps, planner, controller and tree parameters
config/avoidance_bt.xml  the recovery behaviour tree Nav2 runs
patches/              the optional depth camera resolution change
scripts/prereqs.sh    ROS 2 Jazzy, Gazebo Harmonic and the build tools
scripts/install.sh    the agent, px4_msgs and this package
test/                 the measurement scripts behind the numbers above
COMMANDS.md           the one-page command reference
```

`world_markers.py` reads the Gazebo world from `~/PX4-Autopilot/Tools/...` by
default. If PX4 lives elsewhere, pass `world_sdf`, or the wall outlines simply
will not appear.

## License

BSD-3-Clause, the same licence as PX4 and `px4_msgs`. See `LICENSE`.
