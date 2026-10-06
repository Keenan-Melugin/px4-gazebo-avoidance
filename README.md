# px4-gazebo-avoidance

Depth-camera obstacle avoidance for PX4 in Gazebo, flown from RViz, with an
optional Nav2 layer that plans a route around what the camera has seen.

A ROS 2 node turns the simulated depth camera's point cloud into the polar
obstacle histogram PX4 expects. A software pilot flies the aircraft to clicked
waypoints by streaming synthetic manual control, which keeps PX4 in the one
flight mode where its collision prevention applies. Fly it at a wall and it
brakes. Switch avoidance off and it hits the wall.

![RViz: the aircraft facing a wall, the depth cloud painting it, the green goal ball and the orange command ball](docs/img/rviz-overview.png)

## Start here

| You want to | Read |
|---|---|
| Install it, from an empty Ubuntu 24.04 to a flying aircraft | [docs/install.md](docs/install.md) |
| Fly it from RViz, in brake mode and in plan mode | [docs/fly.md](docs/fly.md) |
| Understand how the pieces talk, and where they bite | [docs/how-it-works.md](docs/how-it-works.md) |
| Change it, and prove the change did what you meant | [docs/change-it.md](docs/change-it.md) |
| Look a command up | [COMMANDS.md](COMMANDS.md) |

## Status

Everything below was measured, on Windows 11 with WSL2 Ubuntu 24.04, 12
threads and an AMD RX 7800 XT unless the row says otherwise.

| | Result |
|---|---|
| Brake mode: standoff from a wall at `CP_DIST 2.0` | 1.98, 2.05 and 2.60 m in three runs here; 2.02 and 2.07 m in two runs on a fresh install. With avoidance off it reaches the wall and collides |
| Plan mode: a goal 5.5 m behind a 10 m wall, from 13.5 m out | Reached in every run since the costmap fix, seven so far, two of them on a fresh install. Best 28 s; the measured reference run 43 s with a 6.2 m detour round the near end |
| Heading hold | Worst error 5.6 degrees over 0, 90, 180 and -90 |
| Depth camera | 2.900 m reported against a wall placed at 2.90 m |
| Simulation speed | Real-time factor 1.00 headless, 0.55 to 0.89 with the Gazebo GUI open, 0.033 under software rendering |
| Clean install | The install guide followed verbatim on a fresh Ubuntu 24.04 with 4 cores, 8 GB and a spinning disk: every step passed and both modes flew |

What it is not: avoidance here is a hover capability. PX4's collision
prevention is horizontal only, runs in Position mode only, and stops during
VTOL transition. The camera sees a 73 degree arc ahead and nothing else, so the
aircraft has to be pointed roughly where it is going. Nothing here has run on
real hardware. The airframe is PX4's `x500_depth`; plain `x500` has no camera.

## Hardware

| | Measured |
|---|---|
| Disk | 18 GB for everything, 4.2 GB of it the PX4 tree after one SITL build |
| RAM | Peak 4.4 GB, during the PX4 build. 8 GB passes with margin. 4 GB untested, and the PX4 build is where it would fail first |
| CPU | 4 cores installs and runs it, with degraded flight dynamics under aggressive manoeuvres. 6 or more for the behaviour in the table above. Below 4 untested |
| GPU | Hardware OpenGL 3.3 or better is a requirement, not a preference: software rendering runs at 0.033 real time. On WSL, `GALLIUM_DRIVER=d3d12` |
| Time | About 1.5 hours on 4 cores and a spinning disk, nearly all of it waiting on package installs and builds |
| OS | Ubuntu 24.04. WSL2 tested. Native Ubuntu untested, but nothing here is WSL-specific. Raspberry Pi 5 untested, see the install guide. Raspberry Pi OS cannot: no ROS 2 Jazzy packages exist for Debian. VMware VM on Windows: installs and flies, avoidance not validated (2 Hz camera), see the install guide. Mac: through an Ubuntu 24.04 arm64 VM, untested |

## Versions

PX4 v1.17.0, px4_msgs release/1.17, ROS 2 Jazzy, Gazebo Harmonic 8.x,
eProsima Micro XRCE-DDS Agent v2.4.3. The last one is not `micro-ros-agent`,
which is a different program and will not work.

## License

BSD-3-Clause, the same licence as PX4 and `px4_msgs`. See `LICENSE`.
