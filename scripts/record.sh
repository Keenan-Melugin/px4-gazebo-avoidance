#!/usr/bin/env bash
# Record a run's light topics to a rosbag2 (mcap). Ctrl-C stops it.
#
#   scripts/record.sh NAME                 # position, status, histogram, sticks, mode, goals, TF, scan, Nav2
#   scripts/record.sh NAME --cloud         # plus the depth cloud: 21.7 MB/s at 320x240, 1.3 GB a minute
#   scripts/record.sh NAME --lidar         # plus the lidar scan (29 kB/s)
#   scripts/record.sh NAME /some/topic ... # plus anything else
#
# /clock is recorded so that `ros2 bag play NAME` drives nodes started with
# use_sim_time, which is how docs/data.md replays a cloud into the obstacle
# node with no simulator running.
set -euo pipefail
name=${1:?usage: scripts/record.sh NAME [--cloud] [--lidar] [topic ...]}
shift
topics=(
  /clock
  /fmu/out/vehicle_local_position_v1
  /fmu/out/vehicle_status_v1
  /fmu/out/vehicle_odometry
  /fmu/in/obstacle_distance
  /fmu/in/manual_control_input
  /avoidance_sim/mode
  /avoidance_sim/pilot_goal
  /odom /tf /tf_static
  /scan /cmd_vel /plan
)
for a in "$@"; do
  case "$a" in
    --cloud) topics+=(/depth_camera/points) ;;
    --lidar) topics+=(/lidar) ;;
    *) topics+=("$a") ;;
  esac
done
echo "recording ${#topics[@]} topics to ./$name (Ctrl-C to stop)"
exec ros2 bag record -s mcap -o "$name" "${topics[@]}"
