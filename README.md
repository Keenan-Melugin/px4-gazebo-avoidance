# px4-gazebo-avoidance

Depth-camera obstacle avoidance for PX4 in Gazebo, flown from RViz.

A ROS 2 node turns the simulated depth camera's point cloud into the polar
obstacle histogram PX4 expects, and a software pilot flies the aircraft to
clicked waypoints by streaming synthetic manual control, which is what keeps
the aircraft in the one flight mode where PX4 applies collision prevention.

Fly it at a wall and it stops. With avoidance switched off, it hits the wall.

## What is actually verified

Measured on this stack, not inferred:

| Thing | Result |
|---|---|
| Collision prevention standoff | Holds **1.98 m** off a wall at `CP_DIST 2.0` |
| With avoidance off | Reaches the wall face and collides |
| Obstacle histogram geometry | 15 of 72 bins inside the camera's 73 degree arc, **zero outside** |
| Heading control | Within **5.5 degrees** on all four cardinal headings |
| Path straightness while turning | 0.32 m lateral drift |
| Simulation speed | Real-time factor **1.00** with GPU rendering |

## What this is not

Three limits worth stating plainly, because each is easy to assume away.

**Avoidance here is a hover capability.** It brakes, it does not plan around
things. PX4's collision prevention is horizontal only, runs in Position mode
only, and stops during VTOL transition. For a fixed-wing or transitioning
aircraft at cruise speed, stereo sensing range does not support avoidance at
all, which is a sensing limit rather than a software one.

**This is simulation.** Nothing here has flown on hardware.

**The airframe is PX4's x500 quadrotor**, not any particular vehicle.

## Requirements

- **Ubuntu 24.04**, x86_64 or arm64. Not Raspberry Pi OS: the ROS package
  index for Debian bookworm contains zero ROS 2 Jazzy packages.
- **ROS 2 Jazzy**
- **Gazebo Harmonic**
- **PX4 v1.17.0** and **px4_msgs release/1.17**
- **Micro XRCE-DDS Agent v2.4.3** from eProsima. Not `micro-ros-agent`, which
  is a different thing and will not work.
- Hardware-accelerated OpenGL. Software rendering gives a real-time factor
  around 0.033 against 1.00, so it is unusable rather than merely slow.

## Install

```bash
./scripts/install.sh
```

It stops at the first failure and says what failed. Two steps are slow and
worth knowing about before you start:

- **`px4_msgs` has no binary package on any architecture**, so it compiles
  everywhere: 235 messages and one service. Minutes on a desktop, an evening
  on a Raspberry Pi, where it can also be killed for memory. Limit the build
  with `MAKEFLAGS="-j2"` and add swap if you have under 8 GB.
- **PX4 itself** is a full firmware build the first time.

## Run

Two commands, in two terminals.

```bash
cd ~/PX4-Autopilot && PX4_GZ_WORLD=walls make px4_sitl gz_x500_depth
```

```bash
ros2 launch avoidance_sim sim.launch.py
```

PX4 is deliberately not in the launch file: starting it means running a make
target from the PX4 source tree, and burying a build inside a launch file makes
failures hard to read.

Then switch avoidance on:

```bash
px4-param set CP_DIST 2.0
```

## Fly it

In RViz, on the green goal ball:

- **Drag the three arrows** for east, north and altitude.
- **Drag the ring** to set the heading it will hold. The yellow arrow shows it.
- **Right-click the ball** and choose FLY HERE to go there with avoidance on.
- **Right-click the orange ball** above the aircraft to ARM, LAND or RETURN.

Arm first, then fly. Do not use TAKEOFF while the pilot is running: it streams
sticks continuously to hold Position mode, and that overrides the automatic
takeoff.

## Platform support

| Platform | Hosts the simulation? | Notes |
|---|---|---|
| Linux x86_64 | Yes | Reference platform |
| Windows via WSL2 | Yes | Needs `GALLIUM_DRIVER=d3d12`, which is WSL-only |
| Raspberry Pi 5, Ubuntu 24.04 | **Only with a display attached** | See below |
| Raspberry Pi OS | No | No ROS 2 Jazzy packages exist for it |
| macOS | No | Not a ROS 2 target in any distribution. Use a Ubuntu arm64 VM |

**The Pi case is specific.** Gazebo's default renderer needs OpenGL 3.3 and the
Pi's driver caps desktop GL at 3.1, so it will not start. The older renderer
runs on 3.1 but cannot render without an X server, so headless does not work:

```bash
PX4_GZ_SIM_RENDER_ENGINE=ogre make px4_sitl gz_x500_depth
```

A Pi makes a better companion computer than a simulation host. Ubuntu 24.04
arm64 is a fully supported ROS 2 Jazzy platform, so the nodes here run on it
against a simulation hosted elsewhere.

## Traps

Each of these cost real debugging time and none produces a useful error.

**The frame conversion fails silently.** PX4 is NED and FRD, ROS is ENU and
FLU. Swapping the position axes alone looks right in a plot and is still
wrong: attitude needs a rotation applied on *both* sides, or the aircraft
renders upside down while its heading still reads correctly.

**The yaw stick has a dead band.** Below about 0.10 it does nothing, and above
it the rate is roughly `(stick - 0.10) * 72 deg/s`. Proportional heading
control therefore cannot work: the command fades into the dead band as the
error shrinks, and the aircraft stalls a fixed distance short of every target.
Command a rate through the measured model instead.

**Stick XY is in the heading frame, not NED.** Rotate the position error by the
current heading every cycle, or the aircraft flies off at an angle that changes
as it turns.

**`ObstacleDistance` has three bin states, not two.** A range in centimetres,
`max_distance + 1` meaning observed and clear, and `UINT16_MAX` meaning
unknown. Filling unobserved bins with `UINT16_MAX` and calling that clear
makes PX4 refuse to move, because `CP_GO_NO_DATA` defaults to treating unknown
as blocked. The `frame` field also has to be set to `MAV_FRAME_BODY_FRD`, or
bin zero means north instead of forward.

**PX4 topic names carry a version suffix only when `MESSAGE_VERSION` is not
zero.** `VehicleLocalPosition` is `/fmu/out/vehicle_local_position_v1`;
`ObstacleDistance` has no suffix. Subscribing to the wrong name fails silently.

**PX4's publishers are `BEST_EFFORT` and `VOLATILE`.** A subscriber asking for
`RELIABLE` or `TRANSIENT_LOCAL` matches nothing and never receives.

**`--headless-rendering` is ogre2 only.** The older renderer aborts with
`Unable to open display`, so a guide telling a Pi owner to use that renderer
*and* run headless is self-contradicting.

## Layout

```
avoidance_sim/      the ROS 2 package
  frames.py           frame conventions, shared QoS, helpers
  obstacle_distance.py  point cloud -> 72-bin polar histogram
  software_pilot.py     flies to a goal on synthetic sticks
  goal_3d.py            the draggable 3D waypoint with its heading ring
  command_marker.py     the right-click arm/land menu
  world_markers.py      draws the world's obstacles in RViz
  goal_bridge.py        RViz's flat Goal Pose tool -> PX4 reposition
  tf_publisher.py       PX4 odometry -> the TF tree
  rviz_bridge.py        composes the six RViz-side nodes in one process
launch/sim.launch.py
config/avoidance.rviz
patches/              depth camera resolution, applied to the PX4 tree
scripts/install.sh
```

## License

Not yet chosen. `package.xml` says so rather than implying a license this does
not have.
