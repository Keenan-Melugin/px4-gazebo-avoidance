# Before any real flight

Everything in this repository is set up for the simulator. Several settings
that make the simulation convenient would be dangerous on an aircraft, and
some of the measured numbers belong to a simulated x500, not to your
airframe. This page is the list of what changes first. None of it has been
done here: nothing in this repository has flown on hardware.

Work through it in order. Each item says what the simulator does, why that
is wrong on hardware, and what to do instead. Parameter facts are from the
PX4 v1.17.0 source; file references are to that tree.

## 1. Put PX4's safety parameters back

The launch sets five parameters through `px4-param` (see
[how-it-works.md](how-it-works.md#px4s-parameters-and-why-the-launch-sets-them)). Three of them switch off
something that protects a real aircraft, one more (`COM_RC_OVERRIDE`) is a
decision for item 2, and three that the simulator
leaves at their defaults decide what happens when the companion computer
stops.

| Parameter | Simulator | Why | On hardware |
|---|---|---|---|
| `NAV_RCL_ACT` | 0 | The software pilot's stick stream is the RC link; a pause would otherwise trigger the RC-loss failsafe. 0 is outside the documented range (1 to 6) and works only as "disabled" | Restore a failsafe action (PX4's default is 2, Return). Losing the human's link must do something |
| `NAV_DLL_ACT` | 0 | There is no ground station, and the x500 airframe file sets 2; any value above 0 refuses to arm without one | Set the data-link-loss action your operation needs |
| `CP_GO_NO_DATA` | 1 | The camera sees 73 of 360 degrees, so most bins are unknown; at 0 PX4 will not move sideways or backwards | 0, unless sensors cover every direction you fly. At 1, PX4 flies into space nobody has looked at |
| `CP_DIST` | 2.0 | The standoff | Re-measure it, item 6 |
| `COM_RC_OVERRIDE` | 0 | The pilot is the only stick source, and its own stick changes counted as a takeover and cancelled a Land | Decide in item 2. With a human on a transmitter, override is how they take over from an automatic mode |
| `COM_RC_LOSS_T` | 0.5 s (default) | Any pause over 0.5 s in the pilot's 50 Hz stick stream is RC loss | Know that a companion computer stall is an RC loss |
| `COM_OF_LOSS_T`, `COM_OBL_RC_ACT` | 1.0 s, 0 (defaults) | Plan mode is Offboard; after 1.0 s without setpoints PX4 falls back to Position mode | A dead companion computer loses the Offboard stream and the stick stream at once, and with stick input lost too PX4 falls through to `NAV_RCL_ACT`. With the simulator's 0 that is no action at all |

Parameters persist in PX4's parameter storage once set, so check them on
the flight controller itself, not in this repository's launch file.
`param show -c` at the PX4 shell lists only the changed ones; plain
`param show` lists everything.

Sources: PX4 parameter reference, v1.17
(https://docs.px4.io/v1.17/en/advanced_config/parameter_reference.html);
`src/modules/commander/commander_params.c` (`COM_RC_LOSS_T`,
`COM_OF_LOSS_T`, `COM_OBL_RC_ACT`, `NAV_RCL_ACT`, `NAV_DLL_ACT`);
`src/modules/commander/failsafe/failsafe.cpp` (the fall-through when both
links are lost); PX4 Safety Configuration
(https://docs.px4.io/v1.17/en/config/safety.html).

## 2. Decide who holds the sticks: you cannot have both

This is the decision everything else depends on, and PX4 does not offer the
obvious answer.

PX4 takes stick input from exactly one source at a time
(`ManualControlSelector`, chosen by `COM_RC_IN_MODE`). The software pilot is
a MAVLink stick source. A radio transmitter is the RC source. And PX4 acts
on the transmitter's switches, the kill switch and the mode switch
included, **only while RC is the selected source**
(`src/modules/manual_control/ManualControl.cpp`, `processSwitches`: "Only use
switches if the currently valid source is RC as well"). So:

- With RC priority (`COM_RC_IN_MODE` 5 or 7), PX4 switches to the
  transmitter whenever it is on and valid, and the software pilot's sticks
  are ignored: brake mode cannot fly while the transmitter is on.
- With MAVLink priority (6 or 8), or MAVLink only (1), the software pilot
  flies, and the transmitter's kill and mode switches do **nothing** while
  it streams.
- With 2 (fallback) or 3 (keep the first valid source, PX4's default on
  hardware; the simulator uses 1), which one is selected depends on which
  became valid first and which dropped out, so the switches work or not
  depending on the order things were powered up. Do not fly on that.

There is no setting in which the pilot flies and the transmitter's kill
switch works. Decide which you need for each flight, and test the kill path
in exactly that configuration (item 3). A kill that does not depend on the
stick source, such as a separate hardware cut-off, is worth considering.

Two more facts to plan around:

- **Stick override.** `COM_RC_OVERRIDE` (default 1) lets moving sticks take
  a multicopter out of automatic modes into Position mode; 3 adds Offboard.
  It acts on the selected source, and only on multicopters and VTOLs in
  multicopter mode. The software pilot is a stick source too: its own stick
  changes cancelled a Land, which is why the simulator sets 0 and the pilot
  also freezes its sticks when PX4 leaves its mode (see
  [how-it-works.md](how-it-works.md)). The freeze alone leaves a window of a
  fraction of a second, measured, so with override on and the pilot as the
  selected source, a Land can still be cancelled.
- **What still takes Position mode back.** The pilot does not ask for its
  mode back on its own after PX4 leaves it, and refuses goals while PX4 is
  landing or returning. But publishing `brake` (or `plan`) on
  `/avoidance_sim/mode` is an explicit takeover and does ask. On hardware,
  decide who may send that.

Sources: `src/modules/manual_control/ManualControl.cpp`,
`ManualControlSelector.cpp`; `commander_params.c` (`COM_RC_IN_MODE`,
`COM_RC_OVERRIDE`, `COM_RC_STICK_OV`); `Commander.cpp`
(`manualControlCheck`, "Pilot took over using sticks"); PX4 Offboard mode
(https://docs.px4.io/v1.17/en/flight_modes/offboard.html).

## 3. Test the kill paths on a tether

Before the first free flight, with the aircraft tethered or the propellers
removed as each test allows, and in the stick-source configuration you
chose in item 2:

- the kill switch stops the motors. Map it (`RC_MAP_KILL_SW`), and know that
  flipping it back within `COM_KILL_DISARM` (default 5 s) restarts them;
- disarm works on the ground;
- each failsafe in item 1 fires and does what you set, which means actually
  removing the RC link and the companion computer link in turn;
- removing the obstacle data stops horizontal motion within 0.5 s and
  switches PX4 to Loiter after 5 s (fixed in PX4's collision prevention, no
  parameter);
- the companion computer stopping (unplug it) leaves the aircraft in a
  state the human can fly.

The orange command ball has `DISARM (FORCE, in air)`. In the simulator that
is a convenience. On an aircraft it stops the motors in flight and the
aircraft falls. Remove that menu entry from `avoidance_sim/command_marker.py`
before the stack ever talks to real hardware.

Sources: `commander_params.c` (`COM_KILL_DISARM`);
`src/lib/collision_prevention/CollisionPrevention.cpp` (the 0.5 s
obstacle-stream timeout and the switch to Loiter after 5 s).

## 4. Turn simulated time off

Every node is launched with `use_sim_time` true, so its clock is Gazebo's
`/clock`. On hardware there is no `/clock`, and the nodes' clocks stay at
zero. Timers driven by those clocks do not advance, so the pilot publishes
nothing, and anything that does run compares zero with zero: a velocity
command from minutes ago reads as zero seconds old, and a dead sensor
never goes stale. This follows from how ROS 2 time works with
`use_sim_time` and no `/clock` (ROS 2 design article "Clock and Time",
https://design.ros2.org/articles/clock_and_time.html); it has not been
measured here. Launch with `use_sim_time` false and check, with
`ros2 topic echo`, that message stamps advance.

## 5. Characterise the real sensors

The obstacle node's numbers are the simulated camera's. Measure the real
sensor and set them from the measurements:

- **Range limits and noise**, at the distances you will fly at. A real depth
  camera returns noise, holes and invalid points that the simulated one does
  not.
- **Rate and latency.** The obstacle node drops a sensor after `stale_s`
  (1.0 s) without data. PX4 zeroes horizontal motion after 0.5 s without
  obstacle data. Plan mode holds after 0.5 s without a histogram or 1.0 s
  without Nav2's `/scan`. A real camera over USB on a small computer may
  run slower than that. Set `CP_DELAY` (default 0.4 s, "average delay of
  the range sensor message plus the tracking delay") from the latency you
  measure.
- **Field of view and mount.** The arc and the mount position go in the
  sensor list (see [extend/sensors.md](extend/sensors.md)).
- **Surfaces.** Glass, thin wires, foliage and direct sunlight defeat
  stereo and structured-light cameras. Know what yours cannot see.

Sources: `avoidance_sim/obstacle_distance.py` (`stale_s`);
`CollisionPrevention.hpp` (`RANGE_STREAM_TIMEOUT_US`, 500 ms);
`collisionprevention_params.c` (`CP_DELAY`).

## 6. Re-measure the standoff at real mass and speed

The standoff numbers in the README are a simulated x500 braking at the
speed the brake test reaches. Braking distance grows with speed and
depends on the real airframe's mass and response. `CP_DIST` is measured
from the sensor, not from the propeller tips, so add the distance from the
sensor to the furthest tip. Collision prevention also needs `MPC_POS_MODE`
set to acceleration based (4, the default); check it on the aircraft.
Measure the closest approach at the speeds you will fly, against a soft
obstacle (a foam board or a net), and set `CP_DIST` and the speed limits
from that. `test/regression.py` shows the method: a fixed run-up, the
closest approach, and stopping the push once the aircraft has stood still.

Source: PX4 Collision Prevention
(https://docs.px4.io/v1.17/en/computer_vision/collision_prevention.html),
which also describes `CP_GUIDE_ANG` and the behaviour near obstacle ends.

## 7. Limit where it can be sent

- **Goals.** The pilot flies any pose published on
  `/avoidance_sim/pilot_goal`, at any distance and altitude. Add limits:
  a maximum distance from home and an altitude band.
- **Geofence.** PX4 has none out of the box: `GF_MAX_HOR_DIST` and
  `GF_MAX_VER_DIST` default to 0, which disables them. Set both, and
  `GF_ACTION` (default 2, Hold), so the flight controller enforces a
  boundary even if the companion computer misbehaves.
- **Who can command it.** Any ROS 2 node on the same network and domain can
  publish goals, modes and PX4 commands, and the uXRCE-DDS link carries
  them to the flight controller. Restrict discovery to the companion
  computer (`ROS_AUTOMATIC_DISCOVERY_RANGE` set to `LOCALHOST`, or a
  dedicated `ROS_DOMAIN_ID` on an isolated network). On the PX4 side,
  `UXRCE_DDS_PTCFG` can restrict its topics to localhost and
  `UXRCE_DDS_DOM_ID` sets its domain. SROS2 adds authenticated topics on the
  ROS side; the PX4 uXRCE-DDS page says nothing about security.

Sources: `src/modules/navigator/geofence_params.c`; PX4 Safety
Configuration, "Geofence Failsafe"
(https://docs.px4.io/v1.17/en/config/safety.html); PX4 Geofence
(https://docs.px4.io/v1.17/en/flying/geofence.html); PX4 uXRCE-DDS
(https://docs.px4.io/v1.17/en/middleware/uxrce_dds.html); ROS 2 Jazzy,
Improved Dynamic Discovery
(https://docs.ros.org/en/jazzy/Tutorials/Advanced/Improved-Dynamic-Discovery.html).

## 8. Hover only

Avoidance here is a hover capability. PX4's collision prevention works in
Position mode only, is horizontal only, and stops during VTOL transition;
plan mode has none and relies on the planner's costmap. A VTOL in forward
flight is not protected by anything in this repository. Fly avoidance tests
in multicopter mode, at walking speed.

## 9. Re-measure the pilot's stick model

The software pilot turns velocities into stick deflections through a model
measured on the simulated x500: the dead bands (`XY_DZ` 0.114, `YAW_DZ`
0.105), the slope (`VEL_PER_STICK` 5.02 m/s per unit of stick) and the PD
gains. A different airframe, or PX4 tuned differently, has different
numbers. `test/xy_threshold.py` and `test/yaw_threshold.py` are the method;
on hardware, run the equivalent sweep tethered or in open space, with the
human ready to take over in the configuration chosen in item 2, and
replace the constants in `avoidance_sim/software_pilot.py`.

## 10. Leave the test scripts behind

The scripts in `test/` are simulator tools. `velmode_ab.py` switches
collision prevention off; the sweeps take over the stick stream in external
mode; several arm and take off by themselves. None of them is fit to run
against an aircraft.

## Regulation

Flying is regulated separately from all of this. In Australia that is the
Civil Aviation Safety Authority (CASA, https://www.casa.gov.au/drones);
elsewhere, your national regulator. Weight class, where you fly and whether
the operation is autonomous all decide what is needed.
