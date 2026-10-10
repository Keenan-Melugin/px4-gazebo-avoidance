# Add a second machine

Not built. The ROS side of this package is ordinary ROS 2 Jazzy, and Ubuntu
24.04 on arm64 (the processor architecture of the Raspberry Pi and most
companion computers) is a Tier 1 platform with binary packages. Tier 1 is
ROS 2's top support level, defined in REP 2000, the ROS document that lists
supported platforms per release. So it should run on a Raspberry Pi 5;
nobody has tried yet. The question is what crosses the network.

## Which side runs where

PX4, Gazebo, the DDS agent and the Gazebo bridge stay with the simulator.
The agent is a DDS participant: PX4's topics appear on the domain for every
machine on it, so nothing on the PX4 side changes (`UXRCE_DDS_AG_IP` stays at
localhost). A DDS domain is a numbered group of machines that see each
other's topics. The Pi runs the obstacle node, the pilot and, if wanted,
Nav2. Today's launch cannot be split that way: besides the agent it starts
the Gazebo bridge and the `px4-param` step, which must stay with the
simulator, so `agent:=false` alone is not enough. The hook to write is a
`side:=sim|ros|all` argument that starts only one half.

## Discovery

Use the same `ROS_DOMAIN_ID` on both machines; PX4's startup script copies it
into `UXRCE_DDS_DOM_ID`. Jazzy's default discovery range is `SUBNET`, which
finds other machines by multicast (one message sent to every machine on the
local network at once), and WiFi often drops multicast. Then
`ROS_STATIC_PEERS=<other machine's address>` on both sides names the peer
directly.

Fast DDS, Jazzy's default middleware, fragments anything over 64 kB and
loses the whole message if one fragment is lost. For the point cloud over
WiFi its documentation recommends `FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA`.
That mode sends data over TCP, which retransmits lost packets, and keeps
UDP, which does not, for discovery only. It also recommends raising the
kernel's socket buffers, `net.core.rmem_max` and `wmem_max`. Use wired
Ethernet first; the Pi 5 has gigabit.

## Time

`use_sim_time` is set on every node here, and `/clock` from the bridge is an
ordinary topic, so the Pi's nodes follow the simulator's clock with no
further setup.

## What to measure, in this order

1. The cloud's bandwidth, `ros2 topic bw /depth_camera/points`, before moving
   anything. Here it is 21.7 MB/s at 320x240, which is a gigabit link's
   business and not WiFi's; the lidar scan is 29 kB/s.
2. The rate of the stick stream on the simulator side,
   `ros2 topic hz /fmu/in/manual_control_input`, with the pilot on the Pi and
   the Pi loaded. PX4 declares RC loss after `COM_RC_LOSS_T` (0.5 s), and a
   stream that pauses drops the aircraft out of Position mode. That is the
   one safety property this design has.
3. The Pi's CPU for the obstacle node and the `rviz_bridge` process (the
   pilot and five helpers); here the whole `rviz_bridge` process costs a
   sixth of a core, on x86, and the obstacle node is measured separately.
4. The camera-to-histogram latency.

## The real aircraft later

The agent moves to the companion computer and talks to the flight controller
over a serial port, a direct wired data link
(`MicroXRCEAgent serial --dev ... -b ...`). On the flight controller that is
TELEM2, the Pixhawk's second telemetry port, set up with PX4's
`UXRCE_DDS_CFG` and with `MAV_1_CONFIG 0` so MAVLink, PX4's ground-station
protocol, does not also claim the port. A human on a transmitter and this
package's synthetic sticks compete for PX4's one stick input, and PX4 honours
the transmitter's kill and mode switches only while the transmitter is the
selected source: [../hardware.md](../hardware.md), item 2, has the details.
Which one, and how the human takes over, is decided and tested on the bench
before any flight.

## Sources

- ROS 2 Jazzy: Improved Dynamic Discovery (`ROS_AUTOMATIC_DISCOVERY_RANGE`,
  `ROS_STATIC_PEERS`), About Domain ID, DDS tuning.
  https://docs.ros.org/en/jazzy/
- REP 2000, "ROS 2 Releases and Target Platforms", Jazzy Jalisco.
  https://www.ros.org/reps/rep-2000.html
- Fast DDS 2.14 documentation: environment variables
  (`FASTDDS_BUILTIN_TRANSPORTS`), "Large Data mode and Fast DDS over TCP",
  "Large Data Rates". https://fast-dds.docs.eprosima.com/en/v2.14.4/
- PX4 user guide v1.17: uXRCE-DDS (client start, agent, `UXRCE_DDS_CFG`,
  TELEM2), parameter reference (`COM_RC_IN_MODE`, `COM_RC_LOSS_T`).
  https://docs.px4.io/v1.17/en/middleware/uxrce_dds.html
- PX4-Autopilot v1.17.0, `ROMFS/px4fmu_common/init.d-posix/rcS`
  (`ROS_DOMAIN_ID` copied into `UXRCE_DDS_DOM_ID`).
- Raspberry Pi 5 product page (Gigabit Ethernet).
  https://www.raspberrypi.com/products/raspberry-pi-5/
