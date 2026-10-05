#!/usr/bin/env bash
# Set up everything this package needs on a clean Ubuntu 24.04 machine.
#
# Every step is checked and the script stops at the first failure. That is the
# whole point: an earlier version of this piped the agent install to /dev/null,
# so when it failed the agent was left built but never installed. Nothing
# downstream explains that, and without the agent PX4 and ROS cannot talk at
# all, so the entire stack looks dead for no visible reason.
set -euo pipefail

PX4_VERSION=v1.17.0
MSGS_BRANCH=release/1.17
AGENT_VERSION=v2.4.3
WS="${WS:-$HOME/av_ws}"

say()  { printf '\n=== %s ===\n' "$1"; }
ok()   { printf '  ok    %s\n' "$1"; }
fail() { printf '  FAIL  %s\n' "$1" >&2; exit 1; }

# ---------------------------------------------------------------- preflight
say "Checking the platform"
. /etc/os-release
[ "${VERSION_ID:-}" = "24.04" ] || fail \
  "This needs Ubuntu 24.04. Found ${PRETTY_NAME:-unknown}. Raspberry Pi OS
        will not work: there are no ROS 2 Jazzy packages for it."
ok "Ubuntu 24.04 ($(dpkg --print-architecture))"

[ -d /opt/ros/jazzy ] || fail "ROS 2 Jazzy not found at /opt/ros/jazzy."
ok "ROS 2 Jazzy"

command -v gz >/dev/null || fail "Gazebo not found. Install Gazebo Harmonic."
ok "Gazebo $(gz sim --versions 2>/dev/null | head -1)"

# ------------------------------------------------------------------ the agent
say "Micro XRCE-DDS Agent $AGENT_VERSION"
if command -v MicroXRCEAgent >/dev/null; then
  ok "already installed at $(command -v MicroXRCEAgent)"
else
  cd "$HOME"
  [ -d Micro-XRCE-DDS-Agent ] || git clone -q -b "$AGENT_VERSION" --depth 1 \
    https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
  mkdir -p Micro-XRCE-DDS-Agent/build
  cd Micro-XRCE-DDS-Agent/build
  cmake .. -DCMAKE_BUILD_TYPE=Release >/dev/null
  make -j"$(nproc)" >/dev/null
  # NOT silenced, and the result is checked. This is the step that failed
  # quietly before.
  sudo make install || fail "agent install failed. It needs sudo."
  sudo ldconfig /usr/local/lib/
  command -v MicroXRCEAgent >/dev/null \
    || fail "agent installed but not on PATH. Check /usr/local/bin."
  MicroXRCEAgent --help >/dev/null 2>&1 \
    || fail "agent is on PATH but will not run. Usually a missing
        libmicroxrcedds_agent.so, which means ldconfig did not pick it up."
  ok "installed and runs"
fi

# -------------------------------------------------------------------- px4_msgs
say "px4_msgs $MSGS_BRANCH"
mkdir -p "$WS/src"
if [ ! -d "$WS/src/px4_msgs" ]; then
  git clone -q -b "$MSGS_BRANCH" --depth 1 \
    https://github.com/PX4/px4_msgs.git "$WS/src/px4_msgs"
fi
if [ -d "$WS/install/px4_msgs" ]; then
  ok "already built"
else
  echo "  building 235 messages from source, because no binary package exists"
  echo "  on any architecture. Minutes on a desktop, far longer on a Pi."
  MEM_GB=$(awk '/MemTotal/{printf "%.0f", $2/1024/1024}' /proc/meminfo)
  JOBS=$(nproc)
  if [ "$MEM_GB" -lt 8 ]; then
    JOBS=2
    echo "  only ${MEM_GB}GB of RAM, so limiting to $JOBS jobs to avoid the"
    echo "  out-of-memory killer. Add swap if this still dies."
  fi
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  ( cd "$WS" && MAKEFLAGS="-j$JOBS" colcon build --packages-select px4_msgs ) \
    || fail "px4_msgs build failed. If it was killed, that is memory: add swap
        and rerun with MAKEFLAGS=-j1."
  ok "built"
fi

# ------------------------------------------------------------------- this package
say "avoidance_sim"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
[ -e "$WS/src/avoidance_sim" ] || ln -s "$HERE" "$WS/src/avoidance_sim"
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
( cd "$WS" && colcon build --packages-select avoidance_sim ) \
  || fail "avoidance_sim build failed."
ok "built"

# -------------------------------------------------------------------- renderer
say "Checking you have hardware OpenGL"
if command -v glxinfo >/dev/null; then
  REND=$(glxinfo -B 2>/dev/null | sed -n 's/^OpenGL renderer string: //p')
  case "$REND" in
    *llvmpipe*|*softpipe*|*swrast*)
      printf '  WARNING  software rendering (%s).\n' "$REND"
      printf '           Expect a real-time factor near 0.03 instead of 1.0,\n'
      printf '           which is unusable rather than slow. Fix this first.\n' ;;
    "") printf '  WARNING  could not read a renderer. No display?\n' ;;
    *)  ok "hardware renderer: $REND" ;;
  esac
else
  printf '  note     install mesa-utils to check this (glxinfo).\n'
fi

say "Done"
cat <<EOF
  Source the workspace, then run it in two terminals:

    source $WS/install/setup.bash

    cd ~/PX4-Autopilot && PX4_GZ_WORLD=walls make px4_sitl gz_x500_depth
    ros2 launch avoidance_sim sim.launch.py

  Then switch avoidance on:  px4-param set CP_DIST 2.0

  Note this script does NOT install PX4 $PX4_VERSION itself, since that is a
  firmware build with its own setup script. See the README.
EOF
