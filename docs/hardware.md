# Before any real flight

Everything in this repository is set up for the simulator. Several settings
that make the simulation convenient would be dangerous on an aircraft, and
some of the measured numbers belong to a simulated x500, not to your
airframe. This page is the list of what changes first. None of it has been
done here: nothing in this repository has flown on hardware.

Work through it in order. Each item says what the simulator does, why that
is wrong on hardware, and what to do instead.

## 1. Put PX4's safety parameters back

The launch sets four parameters through `px4-param` (see
[how-it-works.md](how-it-works.md#px4s-parameters-and-why-the-launch-sets-them)). Three of them switch off
something that protects a real aircraft.

| Parameter | Simulator | Why it is set | On hardware |
|---|---|---|---|
| `NAV_RCL_ACT` | 0 | The software pilot's stick stream is the RC link; a pause would otherwise trigger the RC-loss failsafe | Restore a failsafe action (PX4's default is Return). Losing the human's link must do something |
| `NAV_DLL_ACT` | 0 | There is no ground station, and the x500 airframe file sets 2, which refuses to arm without one | Set the data-link-loss action your operation needs. With a ground station in the loop, a lost link must not be ignored |
| `CP_GO_NO_DATA` | 1 | The camera sees 73 of 360 degrees, so most bins are unknown; at 0 PX4 will not move sideways or backwards | 0, unless sensors cover every direction you fly. At 1, PX4 flies into space nobody has looked at |
| `CP_DIST` | 2.0 | The standoff | Re-measure it, item 6 |

Parameters persist in PX4's parameter storage once set, so check them on
the flight controller itself, not in this repository's launch file. Also
check every other parameter you changed while experimenting; `param show`
at the PX4 shell lists the values that differ from the defaults.

Sources: PX4 parameter reference, v1.17
(https://docs.px4.io/v1.17/en/advanced_config/parameter_reference.html),
entries `NAV_RCL_ACT`, `NAV_DLL_ACT`, `CP_GO_NO_DATA`, `CP_DIST`. PX4 Safety
Configuration (https://docs.px4.io/v1.17/en/config/safety.html).

## 2. Decide who holds the sticks

In the simulator the software pilot is the only stick source, and PX4 is
told to accept joystick input only (`COM_RC_IN_MODE` 1). On an aircraft a
person with a radio transmitter must be able to take over at any moment,
and the software pilot then competes with that transmitter for the same
input.

Decide, before anything flies:

- **Which input wins.** `COM_RC_IN_MODE` selects between the RC receiver
  and MAVLink or joystick input. Read its options in the parameter reference
  and pick the one where the human's transmitter has priority.
- **How the human takes over.** A mode switch on the transmitter that the
  software never commands, tested on the ground. `COM_RC_OVERRIDE` controls
  whether stick movement takes over from automatic modes and Offboard.
- **What the pilot does when PX4 leaves its mode.** It no longer asks for
  the mode back once PX4 has left it, so a Land, a Return or a failsafe
  stands (see [how-it-works.md](how-it-works.md)). Verify that on your
  setup; do not take this repository's word for it.

Sources: PX4 parameter reference v1.17, `COM_RC_IN_MODE`,
`COM_RC_OVERRIDE`; PX4 Offboard mode
(https://docs.px4.io/v1.17/en/flight_modes/offboard.html).

## 3. Test the kill paths on a tether

Before the first free flight, with the aircraft tethered or the propellers
removed as each test allows:

- the transmitter's kill switch stops the motors;
- disarm works on the ground;
- each failsafe you configured in item 1 fires and does what you set, which
  means actually removing the RC link, the companion computer link and the
  obstacle data in turn;
- the companion computer stopping (unplug it) leaves the aircraft in a
  state the human can fly.

The orange command ball has `DISARM (FORCE, in air)`. In the simulator that
is a convenience. On an aircraft it stops the motors in flight and the
aircraft falls. Remove that menu entry from `avoidance_sim/command_marker.py`
before the stack ever talks to real hardware.

## 4. Turn simulated time off

Every node is launched with `use_sim_time` true, so its clock is Gazebo's
`/clock`. On hardware there is no `/clock`, and the nodes' clocks stay at
zero. Timers driven by those clocks never fire, so the pilot publishes
nothing, and anything that does run compares zero with zero: a velocity
command from minutes ago reads as zero seconds old, and a dead sensor
never goes stale. Launch with `use_sim_time` false
and check, with `ros2 topic echo`, that message stamps advance.

## 5. Characterise the real sensors

The obstacle node's numbers are the simulated camera's. Measure the real
sensor and set them from the measurements:

- **Range limits and noise**, at the distances you will fly at. A real depth
  camera returns noise, holes and invalid points that the simulated one does
  not.
- **Rate and latency.** The obstacle node drops a sensor after 0.5 s
  without data, and plan mode holds after 0.5 s without a histogram. A real
  camera over USB on a small computer may run slower than that.
- **Field of view and mount.** The arc and the mount position go in the
  sensor list (see [extend/sensors.md](extend/sensors.md)).
- **Surfaces.** Glass, thin wires, foliage and direct sunlight defeat
  stereo and structured-light cameras. Know what yours cannot see.

## 6. Re-measure the standoff at real mass and speed

The standoff numbers in the README are a simulated x500 braking at the
speed the brake test reaches. Braking distance grows with speed and
depends on the real airframe's mass and response. Measure the closest approach at the speeds you will
fly, against a soft obstacle (a foam board or a net), and set `CP_DIST` and
the speed limits from that. `test/regression.py` shows the method: a fixed
run-up, the closest approach, and stopping the push once the aircraft has
stood still.

Source: PX4 Collision Prevention
(https://docs.px4.io/v1.17/en/computer_vision/collision_prevention.html),
which also describes `CP_GUIDE_ANG` and the behaviour near obstacle ends.

## 7. Limit where it can be sent

- **Goals.** The pilot flies any pose published on
  `/avoidance_sim/pilot_goal`, at any distance and altitude. Add limits:
  a maximum distance from home and an altitude band.
- **Geofence.** Set PX4's geofence (`GF_*` parameters) so the flight
  controller enforces a boundary even if the companion computer misbehaves.
- **Who can command it.** Any ROS 2 node on the same network and domain can
  publish goals, modes and PX4 commands, and the uXRCE-DDS link carries
  them to the flight controller. Restrict discovery to the companion
  computer (`ROS_AUTOMATIC_DISCOVERY_RANGE` set to `LOCALHOST`, or a
  dedicated `ROS_DOMAIN_ID` on an isolated network), and consider SROS2 for
  authenticated topics.

Sources: PX4 Geofence (https://docs.px4.io/v1.17/en/flying/geofence.html);
PX4 uXRCE-DDS (https://docs.px4.io/v1.17/en/middleware/uxrce_dds.html);
ROS 2 Jazzy, Improved Dynamic Discovery
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
human on the transmitter, and replace the constants in
`avoidance_sim/software_pilot.py`.

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
