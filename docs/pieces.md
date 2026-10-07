# The pieces

What each part of this stack is, what it is doing here, where its data goes
and why, how to get that data out, and where to learn the part properly.
Read it before [how-it-works.md](how-it-works.md), which assumes these
names mean something, and beside [data.md](data.md), which has the
commands. Versions throughout: ROS 2 Jazzy, Gazebo Harmonic (gz-sim 8),
PX4 v1.17.0, Nav2 for Jazzy.

The one-paragraph version. Gazebo simulates the world, the aircraft's body
and its sensors. PX4 is the flight controller, the same software that runs
on the real aircraft, flying that simulated body. ROS 2 is the message
system that everything on the companion-computer side is written in: the
node that turns the camera's cloud into PX4's obstacle histogram, the pilot
that flies the aircraft on synthetic sticks, Nav2's planner, and RViz's
displays. The Micro XRCE-DDS Agent is the bridge that lets PX4 and ROS 2
exchange messages. Three things save data: PX4 writes a flight log on its
own, ROS 2 records a bag when asked, and the measurement scripts print
numbers that go in the README. Gazebo saves nothing; its input is the world
file.

## ROS 2

### What it is

A set of libraries and tools for writing robot software as
many small programs, called nodes, that exchange typed messages over named
topics without knowing about each other. A node publishes to a topic;
any number of nodes subscribe to it. Underneath, DDS (Fast DDS by default
on Jazzy) carries the messages and discovers who is on the network. Three
other patterns ride on the same transport: services (one request, one
reply), actions (a long task with feedback and a result, like "navigate to
this pose") and parameters (named values a node reads at start and can
expose for change). A launch file starts a set of nodes with their
parameters in one command. TF is the library that keeps the tree of
coordinate frames (where the camera is relative to the body, where the body
is in the world) so any node can ask "where is X in frame Y right now".
Quality of service (QoS) settings on each publisher and subscriber decide
whether messages are retried, kept for late joiners, or dropped when late;
mismatched settings produce silence, not an error. Time can come from the
system clock or from a `/clock` topic, which is how a simulation drives
everything at its own pace.

### What it does here

Everything on the ROS side of this repo is a node:
the obstacle node (one subscriber, one publisher, ten lines of maths in
the middle), the software pilot (a timer that publishes sticks at 50 Hz),
the six small nodes behind RViz's menus and markers, the TF publisher, and
Nav2's servers. `sim.launch.py` starts them with `use_sim_time` so they
follow Gazebo's clock. The PX4 topics are all under `/fmu/out/` and
`/fmu/in/`, and they use the sensor-data QoS profile, which is why every
subscriber to them in this repo is best-effort.

### Where its data goes, and why

Nowhere, by default. A topic is a live
stream; when nobody records it, it is gone. That is deliberate: a robot's
nodes should not fill a disk. When you want a record, rosbag2 subscribes to
the topics you name and writes every message with its timestamp to a bag
(an `.mcap` file on Jazzy), which `ros2 bag play` can publish again later,
at the original timing, to nodes that never know the difference. That is
how a recorded point cloud becomes a test fixture for the obstacle node
with no simulator running ([data.md](data.md)). The repo's `scripts/record.sh`
chooses the topics worth keeping; the depth cloud is left out unless asked
for because it is 21.7 MB/s.

### Getting it out

`ros2 bag info` for what a bag holds; `ros2 bag play`
to replay; `ros2 topic echo` to see messages as text; the `rosbag2_py`
module or the `mcap` Python package to read a bag into a script and plot
it; Foxglove Studio or PlotJuggler to browse one. `ulog2ros2bag` from
pyulog converts a PX4 log into a bag, so both records can be read with one
set of tools.

### Learn it here

The official tutorials are the right order:
https://docs.ros.org/en/jazzy/Tutorials.html (the beginner CLI and client
library sections, then Launch, tf2, and Recording and playing back data).
This repo's examples for each step:

1. Topics: `ros2 topic list`, then `ros2 topic echo /fmu/in/obstacle_distance --once`.
   The 72 numbers are the histogram, in centimetres.
2. A node that subscribes and publishes: `avoidance_sim/obstacle_distance.py`.
   Exercise: write a node that subscribes to the histogram and prints the
   bearing and range of the nearest obstacle once a second.
3. A node driven by a timer: `avoidance_sim/software_pilot.py`, 50 Hz.
   Exercise: change the rate and watch what PX4 does at 5 Hz (`COM_RC_LOSS_T`
   is 0.5 s). Put it back.
4. QoS: `MODE_QOS` in `avoidance_sim/frames.py`. Exercise: publish the mode
   with `ros2 topic pub` without the transient-local flag and see that the
   pilot never hears it.
5. Parameters and launch arguments: `sim.launch.py`, and `sensors:=` with
   `config/sensors_example.yaml`. Exercise: give the camera a 50 degree field
   of view in a file and watch the observed bins fall from 15 to 11.
6. TF: `avoidance_sim/tf_publisher.py`, and `ros2 run tf2_ros tf2_echo odom base_link`.
7. Actions: Nav2's `/navigate_to_pose`, sent from the shell as in data.md,
   or from `test/nav2_flight.py`.
8. Bags: record a brake run with `scripts/record.sh`, replay the cloud into
   the obstacle node, change `decimate`, replay again, compare.

## Gazebo

### What it is

A physics simulator with a rendering engine. A world file
(SDF, an XML dialect) describes the ground, the lights, the physics step
and the obstacles; a model file describes a body as links, joints, visuals,
collision shapes and sensors. Systems, loaded as plugins, do the work:
physics (DART here), the sensors system, the scene broadcaster, user
commands. Rendering sensors (cameras, depth cameras, GPU lidars) are drawn
by the render engine, which is why they need a real GPU and fail in a VM
with a weak one. Gazebo has its own message transport (`gz topic`), separate
from ROS; `ros_gz_bridge` copies chosen topics across.

### What it does here

It simulates the x500 quadrotor's body, its four
motors, an IMU, a barometer, a magnetometer, a GPS and the depth camera, in
the walls or pillars world, at a 4 ms physics step. PX4's bridge reads the
IMU and the other flight sensors straight from Gazebo; the depth cloud and
the clock cross into ROS through `ros_gz_bridge`. The lidar on
`x500_depth_lidar` is a Gazebo GPU lidar on its own topic.

### Where its data goes, and why

Gazebo keeps no record of a run. Its
state exists only while it runs, and that is by design: the simulation's
"data in" is the world file, the model file and the spawn pose, all of
which are text under version control, so a run is reproducible from the
files rather than from a recording. Anything you want kept crosses into
ROS and goes in a bag, or into PX4 and goes in its log. The exception is
the camera's own point of view: `gz sim -g` attaches a window to a running
server to look, and PX4 can also stream the colour camera through the
bridge if you add the topic.

### Getting it out

`gz topic -l` and `gz topic -e -t /lidar` to watch a
Gazebo topic; `gz model -m x500_depth_0 -p` for the aircraft's true pose,
which is the check against PX4's estimate; `ros_gz_bridge` to bring any
topic into ROS; `ros_gz_image` for images.

### Learn it here

https://gazebosim.org/docs/harmonic/ (Getting started,
then SDF worlds, Sensors, and the ros_gz tutorial). This repo's examples:
`worlds/pillars.sdf` with its header comment, `models/x500_depth_lidar/model.sdf`
for a sensor on a link, and the "Where PX4 looks" section of
[extend.md](extend.md) for how PX4 finds both.

## PX4

### What it is

An open-source flight controller: the software that reads
the sensors, estimates where the aircraft is (EKF2), runs the attitude and
position controllers, mixes motor outputs, and decides what to do when
something is lost (the failsafes). On a real aircraft it runs on a Pixhawk;
in SITL (software in the loop) the same code runs as a Linux process and
talks to a simulator instead of hardware. Inside, modules exchange data
over uORB, PX4's own publish-subscribe bus, and the configuration is a flat
list of parameters (`CP_DIST`, `NAV_DLL_ACT`, ...). Flight modes decide who
is in charge: Position mode follows the sticks with position hold, Offboard
mode follows setpoints from a companion computer, and so on.

### What it does here

It flies the simulated x500. Its collision
prevention, which lives in the multicopter Position-mode controller, reads
the `obstacle_distance` histogram that the ROS node sends and limits the
velocity toward anything within `CP_DIST`. The pilot's synthetic sticks
keep it in Position mode (brake mode); Nav2's velocity setpoints put it in
Offboard mode (plan mode), where collision prevention does not apply.

### Where its data goes, and why

PX4 writes a ULog file on its own, from
boot until disarm in simulation, exactly as it would on the aircraft:
positions, attitude, sensor data, the histogram it received and the fused
one, the velocity limits it derived, the sticks, every parameter, and its
own messages. The purpose on an aircraft is forensics and tuning after a
flight; the purpose here is the same, plus provenance: a number in the
README can be traced to a log with the parameters it flew under. The trap is
size, 6 GB for a long armed session, because logging stops only at disarm.
`px4-logger off` stops it by hand.

### Getting it out

`pyulog` reads a log in Python and ships command-line
tools (`ulog_info`, `ulog2csv`, `ulog2ros2bag`); Flight Review plots one in
a browser; PlotJuggler plots any topic; `scripts/ulog_timeline.py` prints a
run as a table. A log from SITL is the same format as one from the real
aircraft, so a flight-data pipeline that ingests one ingests the other; mark
the source.

### Learn it here

https://docs.px4.io/v1.17/en/ (Flight Modes, Simulation,
the uORB and uXRCE-DDS middleware pages, Logging and Flight Log Analysis,
Collision Prevention). This repo's examples: `scripts/px4_params.sh` for
how parameters are set from outside, `px4-listener obstacle_distance` for
reading a uORB topic live, and the collision-prevention source lines cited
in [extend.md](extend.md).

## The bridge: Micro XRCE-DDS Agent and px4_msgs

### What it is

PX4's uORB and ROS 2's DDS are different buses. PX4 runs a
small client (uXRCE-DDS) that serialises chosen uORB topics and sends them
over UDP or a serial link to an agent process, which is a full DDS
participant and republishes them as ROS 2 topics; the reverse direction
works the same. `px4_msgs` is the ROS 2 package of message definitions
generated from PX4's `.msg` files, built against the matching PX4 branch so
the two sides agree on every field. Since PX4 v1.16 the messages carry a
version, and a versioned topic's name ends in `_v1`.

### What it does here

`sim.launch.py` starts the agent on UDP 8888; PX4
SITL connects to it on start. Every `/fmu/out/` and `/fmu/in/` topic exists
because of this pair. On a real aircraft the agent runs on the companion
computer and the link is a serial port.

### Learn it here

https://docs.px4.io/v1.17/en/middleware/uxrce_dds.html
and the ROS 2 User Guide beside it; https://github.com/PX4/px4_msgs.
`scripts/install.sh` shows the build of both, and the "which side runs
where" section of [extend.md](extend.md) says what moves when the agent
moves.

## RViz

### What it is

ROS 2's 3D visualiser. It subscribes to topics and draws
them: point clouds, laser scans, paths, markers, TF frames, costmaps. It
also sends: the 2D Goal Pose tool publishes a pose, and interactive markers
let a node put clickable, draggable things in the scene. A config file
saves which displays are open and how they look.

### What it does here

It is the cockpit. The depth cloud, the wall
outlines, the aircraft marker, the scan when the lidar is on, Nav2's path
and costmaps, and two interactive markers: the green goal ball with its
menu (brake or plan mode, fly here) and the orange command ball (arm, take
off, land). `config/avoidance.rviz` is the layout. RViz keeps no data; it
is a window.

### Learn it here

https://docs.ros.org/en/jazzy/Tutorials/Intermediate/RViz/RViz-Main.html.
This repo's examples: `avoidance_sim/goal_3d.py` and `command_marker.py`
for interactive markers, `world_markers.py` for drawing geometry.

## Nav2

### What it is

The ROS 2 navigation stack: a planner server that computes a
path through a costmap, a controller server that follows it with velocity
commands, costmaps (a global one for planning, a local rolling one for
following) built from sensor observations with an inflation layer around
obstacles, a behaviour tree that sequences planning, following and
recovery, and lifecycle management that brings the servers up in order.
It is two-dimensional and built for ground robots; it assumes a localised
robot and a map, neither of which an aircraft with a forward camera has by
default.

### What it does here

Plan mode. The depth cloud is flattened into a 2D
scan, the costmaps mark what the camera has seen, the planner finds a
route round the wall, and pure pursuit turns it into `/cmd_vel`, which the
pilot converts into PX4 velocity setpoints in Offboard mode. The
configuration in `config/nav2.yaml` is unusual on purpose (no map, `odom`
as the global frame, a global costmap that never clears), and every
non-default value carries the measurement that set it. The behaviour tree
in `config/avoidance_bt.xml` removes the ground-robot recoveries.

### Where its data goes

Costmaps and paths are topics (`/plan`, the costmap
topics), so they live in a bag if recorded and nowhere otherwise.

### Learn it here

https://docs.nav2.org/ (Concepts, then the configuration
guide for the costmap and the planner and controller servers, then
behaviour trees). This repo's examples: the comments in `config/nav2.yaml`,
and the five defects recorded in `launch/nav2.launch.py`'s docstring, which
are the things the tutorials do not say.

## The pieces this repo adds

**The software pilot** (`avoidance_sim/software_pilot.py`) is the thing
that lets PX4's own collision prevention be used at all: it flies the
aircraft to a goal by publishing synthetic stick positions, which keeps PX4
in Position mode, the one mode where that feature runs. Its gains and dead
bands were measured, and the scripts that measured them are in `test/`.

**The obstacle node** (`avoidance_sim/obstacle_distance.py`) turns any
number of point clouds and laser scans into PX4's 72-bin histogram,
merging them before PX4 because PX4 cannot merge two histogram publishers.
It is the model of a ROS node in this repo: one subscriber per sensor, one
timer, one publisher.

**The world geometry module** (`avoidance_sim/world_geometry.py`) reads a
world file's boxes for RViz and for the tests, so the picture and the
measurement always agree on where the walls are.

**The gate** (`test/gate.py` and the scripts behind it) is how a change is
proved: fly the two things the repo claims, print numbers, exit with the
count of failures. The README's status table is its output over time, and
the measurement scripts are the ROS 2 client-library tutorial applied to
one aircraft.

**The build** uses colcon and ament, ROS 2's build tools: `colcon build`
compiles or installs each package in the workspace, and `source
install/setup.bash` puts the result on the path. A Python package like this
one is installed by copying, so a change needs a rebuild and a relaunch;
https://colcon.readthedocs.io/ has the rest.

## Where the records live, in one table

| Record | Written by | When | Holds | Read with |
|---|---|---|---|---|
| ULog `.ulg` | PX4 | Always, boot to disarm | PX4's view: state, sensors, histogram received and fused, sticks, parameters | pyulog, Flight Review, PlotJuggler, `scripts/ulog_timeline.py`, a flight-data pipeline |
| Bag `.mcap` | rosbag2 | When asked | ROS topics: cloud, scan, histogram sent, goals, TF, Nav2 plans | `ros2 bag`, `rosbag2_py`, the `mcap` package, Foxglove |
| Printed numbers | The measurement scripts | Each run | The one thing each script measures, with its conditions | The README status table, git history |
| World and model files | You | Before a run | The conditions: obstacles, sensors, spawn | Version control; `python3 -m avoidance_sim.world_geometry` |

PX4's log and a bag of the same run are the same moments seen from two
sides of the bridge; `ulog2ros2bag` puts them in one format when that
helps. A simulation log and a real flight's log are the same format seen
from two sides of reality, which is the comparison the project is working
toward.

## Sources

- ROS 2 Jazzy documentation: Concepts (Nodes, Topics, Services, Actions,
  Parameters, Launch, Quality of Service, Time), the tf2 tutorials, and
  Recording and playing back data. https://docs.ros.org/en/jazzy/
- rosbag2 (jazzy branch): mcap as the default storage, `--clock`.
  https://github.com/ros2/rosbag2/tree/jazzy; the MCAP format,
  https://mcap.dev/
- Gazebo Harmonic documentation: Getting started, SDF worlds, Sensors;
  gz-sim 8 API for the Sensors and WindEffects systems; SDFormat 1.11
  specification. https://gazebosim.org/docs/harmonic/
- ros_gz (jazzy): `ros_gz_bridge` and `ros_gz_image` READMEs.
  https://github.com/gazebosim/ros_gz
- PX4 user guide v1.17: Flight Modes, Simulation, uORB Messaging, uXRCE-DDS,
  ROS 2 User Guide, Logging, ULog File Format, Flight Log Analysis,
  Collision Prevention. https://docs.px4.io/v1.17/en/
- PX4 v1.17.0 source as cited in [extend.md](extend.md) and [data.md](data.md).
- px4_msgs, https://github.com/PX4/px4_msgs; eProsima Micro XRCE-DDS,
  https://micro-xrce-dds.docs.eprosima.com/
- pyulog, https://github.com/PX4/pyulog; Flight Review,
  https://github.com/PX4/flight_review; PlotJuggler, https://plotjuggler.io/
- Nav2 (Jazzy) documentation: Concepts, Configuration Guide, Behavior
  Trees. https://docs.nav2.org/
- RViz user guide for Jazzy,
  https://docs.ros.org/en/jazzy/Tutorials/Intermediate/RViz/RViz-Main.html
- colcon, https://colcon.readthedocs.io/
