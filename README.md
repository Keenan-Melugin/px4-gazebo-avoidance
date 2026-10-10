# px4-gazebo-avoidance

Depth-camera obstacle avoidance for PX4 in Gazebo, flown from RViz, with an
optional Nav2 layer that plans a route around what the camera has seen.

A ROS 2 node turns the simulated depth camera's point cloud into the polar
obstacle histogram PX4 expects. A software pilot flies the aircraft to clicked
waypoints on synthetic manual control, which keeps PX4 in the one flight
mode where its collision prevention applies. Fly it at a wall and it brakes;
switch avoidance off and it hits the wall.

It is a learning and test bench for the eVTOL project's autonomy work: a
place to learn the stack, try a world, a sensor or an airframe, and measure
the result before anything flies. It is not a flight-ready system.

![RViz: the aircraft facing a wall, the depth cloud painting it, the green goal ball and the orange command ball](docs/img/rviz-overview.png)

## Start here

| You want to | Read |
|---|---|
| Learn what each piece is: ROS 2, Gazebo, PX4, the bridge, RViz, Nav2, and where their data goes. Read this first if those names are new | [docs/pieces.md](docs/pieces.md) |
| Install it, from an empty Ubuntu 24.04 to a flying aircraft | [docs/install.md](docs/install.md) |
| Fly it from RViz, in brake mode and in plan mode | [docs/fly.md](docs/fly.md) |
| Understand how the pieces talk, and where they bite | [docs/how-it-works.md](docs/how-it-works.md) |
| Change it, and prove the change did what you meant | [docs/change-it.md](docs/change-it.md) |
| Add to it: a world, a sensor, an airframe, a second machine | [docs/extend.md](docs/extend.md) |
| Get data out, put inputs in, change the conditions of a run | [docs/data.md](docs/data.md) |
| Take it toward a real aircraft: what to change before anything flies | [docs/hardware.md](docs/hardware.md) |
| Look a command up | [COMMANDS.md](COMMANDS.md) |

## Status

Everything below was measured, on Windows 11 with WSL2 Ubuntu 24.04, 12
threads and an AMD RX 7800 XT unless the row says otherwise.

| | Result |
|---|---|
| Brake mode: closest approach to a wall at `CP_DIST 2.0` | 1.98, 2.05, 2.60, 2.09, 2.02 and 1.97 m in six runs here; 2.02 and 2.07 m on a fresh install. With avoidance off it collides. Pushed on after braking it creeps along the face toward free space (`CP_GUIDE_ANG`), 3 m in 14 s, and from 0.5 m inside the wall's end it slides round the end; the test now reads the closest approach and stops pushing once the aircraft stands still |
| Plan mode: a goal 5.5 m behind a 10 m wall, from 13.5 m out | Reached in 14 of 16 runs, two on a fresh install. Best 20 s round the near end; the reference run 43 s with a 6.2 m detour. The misses: one stalled short of the far end, one touched the wall's face. One pass detoured 25 m on the global costmap's memory of earlier runs |
| Heading hold | Worst error 5.6 degrees over 0, 90, 180 and -90 |
| A second world, `pillars`, `--world pillars` | Passes: 2.04 m standoff from its east wall after two pillars pushed the aircraft 3.9 m sideways; Nav2 round its 6 m wall with a 4.2 m excursion. Its walls are read from its file |
| Two sensors merged: the camera plus a 360 degree lidar (`models/x500_depth_lidar`, `lidar:=true`) | All 72 bins observed. Gate passes: standoff 2.24, 2.57 and 1.97 m, Nav2 round the wall. With `CP_GO_NO_DATA 0` it flies sideways and brakes 2.6 m from a wall the camera cannot see |
| Depth camera | 2.900 m reported against a wall placed at 2.90 m |
| Simulation speed | Real-time factor 1.00 headless, 0.55 to 0.89 with the Gazebo GUI open, 0.033 under software rendering |
| Clean install | The install guide followed verbatim on a fresh Ubuntu 24.04, 4 cores, 8 GB, spinning disk: every step passed and both modes flew |

What it is not: avoidance here is a hover capability. PX4's collision
prevention is horizontal only, runs in Position mode only, and stops during
VTOL transition. The camera sees a 73 degree arc ahead, so the aircraft has
to point roughly where it is going. Plan mode has no PX4 collision
prevention at all: in Offboard the planner's costmap is the only thing that
sees obstacles, and the pilot holds position if the obstacle data stops.
Nothing here has run on real hardware, and the parameters it sets are for
the simulator only: [docs/hardware.md](docs/hardware.md) lists what changes
before anything flies. Nav2 plans on the camera only; the lidar feeds collision
prevention but not the planner yet. The airframe is PX4's `x500_depth`; plain `x500` has no camera,
and `models/x500_depth_lidar` here adds a 360 degree 2D lidar to it.

## Hardware

| | Measured |
|---|---|
| Disk | 18 GB for everything, 4.2 GB of it the PX4 tree after one SITL build |
| RAM | Peak 4.4 GB, during the PX4 build. 8 GB passes with margin. 4 GB untested, and the PX4 build is where it would fail first |
| CPU | 4 cores installs and runs it, with degraded flight dynamics under aggressive manoeuvres. 6 or more for the behaviour in the table above. Below 4 untested |
| GPU | Hardware OpenGL 3.3 or better is a requirement, not a preference: software rendering runs at 0.033 real time. On WSL, `GALLIUM_DRIVER=d3d12` |
| Time | About 1.5 hours on 4 cores and a spinning disk, nearly all waiting on installs and builds |
| OS | Ubuntu 24.04: under WSL2 on Windows (tested), natively (untested, nothing here is WSL-specific), or in a VMware VM (installs and flies, avoidance not validated: 2 Hz camera). Raspberry Pi 5 untested; Raspberry Pi OS cannot (no Jazzy packages for Debian). The install guide starts from the operating system |

## Versions

PX4 v1.17.0, px4_msgs release/1.17, ROS 2 Jazzy, Gazebo Harmonic 8.x,
eProsima Micro XRCE-DDS Agent v2.4.3. The last one is not `micro-ros-agent`,
which is a different program and will not work.
