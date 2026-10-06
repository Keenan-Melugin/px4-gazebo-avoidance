# Command reference

Everything you actually type, and what it does. Verified against this package
rather than written from memory: the launch arguments come from
`ros2 launch ... --show-args`.

Three terminals is the normal shape. PX4 in one, the ROS 2 side in another,
and a third for poking at things.

## Every session starts here

```bash
source /opt/ros/jazzy/setup.bash
source ~/av_ws/install/setup.bash
```

Without both of these, `ros2 launch avoidance_sim ...` fails with
`Package 'avoidance_sim' not found`. `~/av_ws` is the default workspace;
`install.sh` creates it and honours `WS=...` if you want it elsewhere.

To use the `px4-*` commands from a terminal that is not PX4's own console:

```bash
export PATH="$HOME/PX4-Autopilot/build/px4_sitl_default/bin:$PATH"
```

There are 91 of them. They are generated into the build directory, so nothing
puts them on `PATH` for you.

## Run it

### Terminal 1: PX4 and Gazebo

```bash
cd ~/PX4-Autopilot
PX4_GZ_WORLD=walls HEADLESS=1 make px4_sitl gz_x500_depth
```

| Part | What it does |
|---|---|
| `PX4_GZ_WORLD=walls` | Loads the world with the obstacle walls. Omit for an empty one |
| `HEADLESS=1` | Suppresses the Gazebo GUI. Worth 10 to 45% of real-time factor. The server still renders the depth camera |
| `gz_x500_depth` | The airframe. Plain `gz_x500` has no camera and will not work |
| `PX4_GZ_SIM_RENDER_ENGINE=ogre` | Add this on a Raspberry Pi. Its driver caps desktop OpenGL at 3.1 and Gazebo's default renderer needs 3.3 |

This leaves you at a `pxh>` prompt. That prompt is PX4's own shell, not bash.

### Terminal 2: the ROS 2 side

```bash
ros2 launch avoidance_sim sim.launch.py
```

Starts the XRCE agent, the Gazebo-to-ROS bridge, the obstacle node, the six
RViz-side nodes, and RViz. Arguments:

| Argument | Default | Use |
|---|---|---|
| `rviz:=false` | true | Skip RViz, for a headless run or a test |
| `agent:=false` | true | You already have an agent running. Otherwise the second one collides on UDP 8888 |
| `agent_cmd:=/path/to/MicroXRCEAgent` | `MicroXRCEAgent` | The agent is not on `PATH` |
| `world_sdf:=/path/to/world.sdf` | the node default | PX4 lives somewhere other than `~/PX4-Autopilot`, or you want different wall outlines |

For path planning instead, which includes everything above:

```bash
ros2 launch avoidance_sim nav2.launch.py
```

Adds `base:=false` if the base stack is already running.

### At the `pxh>` prompt, before flying

```
param set CP_DIST 2.0
param set CP_GO_NO_DATA 1
```

`CP_DIST` is the standoff in metres and **avoidance is off until you set it**
(`-1` disables it). `CP_GO_NO_DATA 1` matters more than it looks: the camera
sees 73 degrees, so 57 of the 72 obstacle bins are honestly unknown, and at
the default of 0 PX4 refuses to accelerate in any direction it cannot see.
Leave it at 0 and the aircraft will only fly forwards, with no error.

Note these are `param set`, not `px4-param set`. `px4-param` is the external
client and is only reachable from a terminal that has the build `bin` on
`PATH`.

## Flying it

In RViz, on the green ball:

| Action | Effect |
|---|---|
| Drag the three arrows | Move the goal east, north, up |
| Drag the ring | Set the heading it will hold. The yellow arrow shows it |
| Right-click, `FLY HERE (avoidance ON)` | Fly there with collision prevention active |
| Right-click, `FLY HERE (direct, NO avoidance)` | PX4 reposition. Turns off the thing this repo is about |
| Right-click, `STOP` | Release the pilot. Needed before arming |

On the orange ball above the aircraft: `ARM`, `TAKEOFF`, `LAND`, `DISARM`,
`DISARM (FORCE, in air)`, and the two mode entries below.

**Arm before you fly, and in that order.** PX4 refuses to arm while the
throttle stick is above centre, and a goal the aircraft has not reached holds
it there. If arming is denied with `throttle above center`, use `STOP` first.
It also refuses for tens of seconds after boot while the barometer and EKF
settle, reporting `Resolve system health failures first`. Wait for
`Ready for takeoff`.

## The costmap display is off by default

The bundled RViz config has a `local costmap` display, and it ships disabled.
On this machine, enabling it makes RViz log

    rviz/glsl120/indexed_8bit_image.vert
    active samplers with a different type refer to the same texture image unit

and the first time, that killed RViz outright rather than just failing to
draw. It is an OGRE shader problem in RViz's Map display, not something this
package can fix, and it appears to be driver dependent.

Tick it on in the Displays panel if you want the costmap and your driver
copes. The `scan (what Nav2 sees)` and `Nav2 plan` displays have no such
problem and are on by default, and between them they show the same story: the
orange points are what the camera observed, the green line is the route the
planner chose through it.

## Switching who does the avoiding

Avoidance belongs to exactly one layer at a time. From the orange ball's menu,
or on the command line:

```bash
ros2 topic pub --once /avoidance_sim/mode std_msgs/msg/String "{data: brake}"
ros2 topic pub --once /avoidance_sim/mode std_msgs/msg/String "{data: plan}"
```

| Mode | PX4 flight mode | Who avoids |
|---|---|---|
| `brake` | Position (`nav_state` 2) | PX4 collision prevention |
| `plan` | Offboard (`nav_state` 14) | Nav2. PX4 has none in Offboard |

`CP_DIST` does not need changing between them, because collision prevention
does not apply in Offboard at all.

## Planning a route round something

```bash
ros2 launch avoidance_sim nav2.launch.py          # instead of sim.launch.py
```

Then in RViz: right-click the orange ball, **MODE: plan**. Confirm it took:

```bash
ros2 topic echo /fmu/out/vehicle_status_v1 --once | grep nav_state   # 14
```

Set the goal with RViz's **2D Goal Pose** tool (the toolbar button, not the
green ball). In plan mode that goes to Nav2; the green ball is for brake mode.
The green line is the planned path, the orange points are what the camera has
seen. Measured on the walls world: a goal 5.5 m behind a 10 m wall, reached in
43 s round the shorter end.

Two things that matter for a clean run:

- **Observe from far enough back.** The camera covers 1.48 times its range in
  width, so to see both ends of a 10 m wall the aircraft needs about 10 m of
  standoff. Start close and it probes the wall a segment at a time.
- **Expect `detected collision ahead!` in the log.** That is pure pursuit
  objecting to a freshly marked wall cell under its path. The tree recovers
  (`Running wait`, replan); only `Goal failed` is a real failure.

`MODE: brake` to hand control back to the green ball. The pilot also switches
to brake by itself if you give it a green-ball goal while in plan mode.

## Looking at what is happening

```bash
ros2 node list                                   # 7 nodes for the base stack, 8 with the bridge
ros2 topic hz /fmu/out/vehicle_local_position_v1 # about 50 Hz. If silent, the agent is down
ros2 topic hz /depth_camera/points               # the depth cloud, about 12 Hz
ros2 topic hz /scan                              # the 2D scan Nav2 consumes
ros2 topic echo /fmu/out/vehicle_status_v1 --once | grep nav_state
gz topic -e -t /world/walls/stats                # real_time_factor. Want 0.9 or better
```

From a terminal with the PX4 `bin` on `PATH`:

```bash
px4-listener obstacle_distance      # the 72-bin histogram PX4 is actually receiving
px4-uxrce_dds_client status         # "connected" or "disconnected"
px4-commander status                # arming state and flight mode
px4-param show CP_DIST              # read a parameter back
```

`px4-listener obstacle_distance` is the single most useful diagnostic: if it
is empty, perception is not reaching PX4 and nothing downstream can work.

## Measuring it

Each script needs the stack running and flies the aircraft. They are
measurements, not unit tests.

```bash
python3 test/gate.py             # THE GATE: regression.py then nav2_flight.py, about 8 min
python3 test/avoid_test.py       # standoff from a wall. Repositions itself first
python3 test/regression.py       # brake mode: heading hold and the standoff, exit code counts failures
python3 test/mode_test.py        # the brake/plan toggle, both directions
python3 test/yaw_threshold.py    # sweeps the yaw stick to find its dead band
python3 test/xy_threshold.py     # same for the lateral stick, plus the velocity slope
python3 test/hold_test.py        # does the aircraft hold heading when left alone
python3 test/twist_check.py      # is /odom's twist really in base_link FLU
python3 test/nav2_flight.py      # does Nav2 route around a wall
```

Two things to know before trusting any result. **Start position matters**: a
run that begins outside the test area measures nothing, which these scripts
have done. And **nothing else should be driving the aircraft**: if someone is
clicking in RViz while a script runs, the two fight over the same goal topic.

Run the gate after any change to the pilot, the frames, the obstacle node or
the Nav2 configuration. If a number moves, the change did something it was not
meant to. `test/README.md` has the full table.

## When it goes wrong

The simulation degrades after an hour or two of flying, crashes and restarts.
The symptoms are an aircraft that drifts, ignores commands, or reports a
position far from the world. Measurements become inconsistent before they
become obviously wrong, so a clean restart is the first thing to try, not the
last.

```bash
# stop PX4 and Gazebo only, leaving the ROS side up
pkill -f "make px4_sitl"; pkill -f "bin/px4"; pkill -f "gz sim"
sleep 3; pkill -9 -f "gz sim"; pkill -9 -f "bin/px4"
pgrep -cf "gz sim|bin/px4"        # must print 0 before restarting
```

That last check matters. A surviving Gazebo server means the restarted PX4
reattaches to the old degraded world instead of a fresh one, which looks like
the restart did nothing.

Then restart terminal 1, and re-set the parameters, because they do not
survive a restart.

Other specific failures:

| Symptom | Cause |
|---|---|
| `Package 'avoidance_sim' not found` | Workspace not sourced |
| Nothing on any `/fmu/out/` topic | The agent is not running, or died |
| `error while loading shared libraries: libmicroxrcedds_agent.so` | The agent was built but never installed. `sudo make install` in its build directory |
| Aircraft only flies forwards | `CP_GO_NO_DATA` is 0 |
| Real-time factor near 0.03 | Software rendering. Check `glxinfo -B` for `llvmpipe` |
| Real-time factor around 0.5 | The Gazebo GUI is open. Use `HEADLESS=1` |
| Aircraft will not arm | A goal is holding the throttle up (`STOP` first), or PX4 is still booting |

## Building

```bash
cd ~/av_ws
colcon build --packages-select avoidance_sim
source install/setup.bash          # needed again after every build
```

`colcon build` exits 0 even when it matched no packages, so check the summary
says `1 package finished` rather than trusting the exit code.
