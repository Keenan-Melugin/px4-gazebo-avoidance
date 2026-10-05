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

## Install

### 1. PX4

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

### 2. This package

```bash
git clone https://github.com/Keenan-Melugin/px4-gazebo-avoidance.git
cd px4-gazebo-avoidance
./scripts/install.sh
```

It prints what it will write and where, asks for sudo up front, and stops at
the first failure. It builds the agent and `px4_msgs` into `~/av_ws` (override
with `WS=...`), runs `rosdep` and builds this package.

`px4_msgs` has no binary package on any architecture, confirmed by its absence
from a `release:` block in `rosdistro`, so it is always a source build: 235
messages and one service. Minutes on a desktop. The script drops to two
compiler jobs automatically under 8 GB of RAM; add swap if it is still killed.

### 3. Optional: the depth camera patch

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

Two terminals. Source ROS and the workspace in both.

```bash
source /opt/ros/jazzy/setup.bash
source ~/av_ws/install/setup.bash
```

Terminal 1, PX4 and Gazebo:

```bash
cd ~/PX4-Autopilot && PX4_GZ_WORLD=walls HEADLESS=1 make px4_sitl gz_x500_depth
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

Then, **at the `pxh>` prompt in terminal 1**:

```
param set CP_DIST 2.0
param set CP_GO_NO_DATA 1
```

Both matter. Without the first there is no avoidance. Without the second, PX4
refuses to accelerate in any direction the camera cannot see, so the aircraft
will appear to ignore sideways and backwards goals with no error.

### Is it working?

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

Do not use `TAKEOFF` while the pilot is running either: it streams sticks
continuously to hold Position mode, and that overrides the automatic
takeoff.

One surprise worth knowing: with `CP_DIST` set, PX4 forces Loiter if the
obstacle stream stops for five seconds. Stopping the ROS stack mid-flight will
change the aircraft's mode.

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
PX4_GZ_SIM_RENDER_ENGINE=ogre make px4_sitl gz_x500_depth
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
  command_marker.py     the right-click arm/land menu
  world_markers.py      draws the world's obstacles in RViz
  goal_bridge.py        RViz's flat Goal Pose tool -> PX4 reposition
  tf_publisher.py       PX4 odometry -> the TF tree
  rviz_bridge.py        composes the six RViz-side nodes in one process
launch/sim.launch.py
config/avoidance.rviz
patches/              the optional depth camera resolution change
scripts/install.sh
```

`world_markers.py` reads the Gazebo world from `~/PX4-Autopilot/Tools/...` by
default. If PX4 lives elsewhere, pass `world_sdf`, or the wall outlines simply
will not appear.

## License

**Not yet chosen, which means all rights reserved.** Until a license is added,
nobody can legally copy, modify or redistribute this, which is at odds with
publishing an install script. `package.xml` and `setup.py` both carry a
placeholder that is not a valid SPDX identifier, so ROS release tooling will
reject it. BSD-3-Clause would match PX4 and `px4_msgs`.
