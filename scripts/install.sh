#!/usr/bin/env bash
# Set up everything this package needs on an Ubuntu 24.04 machine that
# already has ROS 2 Jazzy and Gazebo Harmonic. scripts/prereqs.sh installs
# those; this script checks for them and stops if they are missing.
#
# Design goal: every step is checked, the script stops at the first failure,
# and the message names the thing that actually failed. An earlier version
# piped the agent install to /dev/null, so when it failed the agent was left
# built but never installed, and every downstream symptom was unexplainable.
#
# A later version then failed for three reasons of its own, all found by
# review and all fixed here:
#   * it smoke-tested the agent with `--help`, which exits 1 by design, so it
#     always failed and blamed a missing shared library that was fine
#   * it sourced ROS under `set -u`, and ROS's own setup.bash reads
#     AMENT_TRACE_SETUP_FILES unguarded, which aborts immediately
#   * it resolved its own location after changing directory, so the
#     documented ./scripts/install.sh invocation died on a clean machine
set -euo pipefail

PX4_VERSION=v1.17.0
MSGS_BRANCH=release/1.17
AGENT_VERSION=v2.4.3

# Resolve our own location BEFORE anything changes directory.
HERE="$(cd -P "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/.." && pwd -P)"
# Resolved like HERE, so the inside-the-workspace test below compares like
# with like through a symlinked home or workspace.
WS="$(realpath -m "${WS:-$HOME/av_ws}")"
AGENT_SRC="$HOME/Micro-XRCE-DDS-Agent"

say()  { printf '\n=== %s ===\n' "$1"; }
ok()   { printf '  ok    %s\n' "$1"; }
warn() { printf '  warn  %s\n' "$1"; }
fail() { printf '  FAIL  %s\n' "$1" >&2; exit 1; }

# ROS setup.bash reads unguarded variables, so -u has to come off around it.
ros_source() {
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  set -u
}

# colcon exits 0 when --packages-select matches nothing, and ament_python
# never imports the code, so the exit code proves very little. Check that
# colcon actually recorded the package as built.
assert_built() {
  local pkg=$1
  [ -f "$WS/install/$pkg/share/colcon-core/packages/$pkg" ] \
    || fail "$pkg reported success but was not actually built. If colcon said
        'ignoring unknown package', the workspace layout is wrong."
}

cat <<EOF
This will write to:
  $WS                     the colcon workspace
  $AGENT_SRC   the Micro XRCE-DDS Agent source and build
  /usr/local                   the agent binary and library, using sudo

It does NOT install ROS 2 or Gazebo (that is scripts/prereqs.sh), nor PX4
$PX4_VERSION (docs/install.md, step 2).
EOF

# ---------------------------------------------------------------- preflight
say "Checking the platform"

[ "${EUID:-$(id -u)}" -ne 0 ] \
  || fail "do not run this as root. It calls sudo itself where it needs to,
        and running the whole thing as root puts the workspace in /root."

[ -r /etc/os-release ] || fail "cannot read /etc/os-release, so I cannot tell
        what this machine is. This needs Ubuntu 24.04 (noble)."
# shellcheck disable=SC1091
. /etc/os-release
CODENAME="${UBUNTU_CODENAME:-${VERSION_CODENAME:-}}"
if [ "$CODENAME" = "noble" ]; then
  ok "Ubuntu 24.04 / noble ($(dpkg --print-architecture))"
elif [ "${VERSION_ID:-}" = "24.04" ]; then
  ok "Ubuntu 24.04 ($(dpkg --print-architecture))"
else
  fail "this needs Ubuntu 24.04 (noble). Found ${PRETTY_NAME:-unknown}.
        Raspberry Pi OS will not work: there are no ROS 2 Jazzy binary
        packages for Debian bookworm."
fi

[ -r /opt/ros/jazzy/setup.bash ] \
  || fail "ROS 2 Jazzy not found. Expected /opt/ros/jazzy/setup.bash.
        scripts/prereqs.sh installs it, with Gazebo and the build tools."
ok "ROS 2 Jazzy"

command -v colcon >/dev/null \
  || fail "colcon not found. Run scripts/prereqs.sh, or
        sudo apt install python3-colcon-common-extensions"
command -v rosdep >/dev/null \
  || fail "rosdep not found. Run scripts/prereqs.sh, or
        sudo apt install python3-rosdep"
for t in git cmake make g++; do
  command -v "$t" >/dev/null || fail "$t not found. Run scripts/prereqs.sh, or
        sudo apt install build-essential git cmake"
done
ok "build tools"

command -v gz >/dev/null || fail "Gazebo not found. scripts/prereqs.sh installs
        Gazebo Harmonic."
if GZ_VER=$(gz sim --versions 2>/dev/null | head -1); then
  case "$GZ_VER" in
    8.*) ok "Gazebo Harmonic $GZ_VER" ;;
    "")  fail "gz is installed but 'gz sim --versions' printed nothing, so
        gz-sim is probably missing. Install gz-harmonic, not just gz-tools." ;;
    *)   fail "found Gazebo $GZ_VER but this needs Harmonic (8.x). Other
        versions will not match ros_gz_bridge and will fail at runtime." ;;
  esac
else
  fail "'gz sim --versions' failed, so gz-sim is not usable. Install
        gz-harmonic."
fi

# Ask for sudo now rather than an hour into the agent build.
# Not plain `sudo -v`. With sudo's default verifypw=all, -v demands a password
# whenever ANY rule matching the user lacks NOPASSWD, so a user who can already
# run sudo without a password (Raspberry Pi and cloud images, where the first
# user is also in the sudo group) is told to type one they may not have.
# Found by this repo's own clean-clone test on its first run.
sudo -n true 2>/dev/null || sudo -v || fail "this needs sudo to install the agent into /usr/local."
ok "sudo"

# ------------------------------------------------------------------ the agent
say "Micro XRCE-DDS Agent $AGENT_VERSION"
if command -v MicroXRCEAgent >/dev/null; then
  ok "already installed at $(command -v MicroXRCEAgent)"
else
  [ -d "$AGENT_SRC" ] || git clone -b "$AGENT_VERSION" --depth 1 \
    https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "$AGENT_SRC"
  mkdir -p "$AGENT_SRC/build"
  echo "  building. This compiles Fast-DDS and Fast-CDR from source too, so"
  echo "  it takes tens of minutes, and considerably longer on a Pi."
  JOBS_AGENT=$(nproc)
  MEM_MB=$(awk '/MemTotal/{printf "%d", $2/1024}' /proc/meminfo)
  if [ "$MEM_MB" -lt 8192 ]; then
    JOBS_AGENT=2
    echo "  under 8 GB of RAM, so limiting to $JOBS_AGENT jobs."
  fi
  # Subshell, so the working directory never moves for the rest of the script.
  (
    cd "$AGENT_SRC/build"
    cmake .. -DCMAKE_BUILD_TYPE=Release >/dev/null
    make -j"$JOBS_AGENT"
  ) || fail "agent build failed. If it was killed, that is memory: add swap
        and rerun."
  ( cd "$AGENT_SRC/build" && sudo make install ) \
    || fail "agent install failed."
  sudo ldconfig /usr/local/lib/
  command -v MicroXRCEAgent >/dev/null \
    || fail "agent installed but is not on PATH. Look in /usr/local/bin."
  # Test the thing the error message actually claims. Do NOT use --help: the
  # agent prints help and exits 1 by design, so that is not a health check.
  if ldd "$(command -v MicroXRCEAgent)" 2>/dev/null | grep -q "not found"; then
    ldd "$(command -v MicroXRCEAgent)" | grep "not found" | sed 's/^/        /'
    fail "the agent is installed but cannot find its shared libraries (above).
        Note Fast-DDS stays in $AGENT_SRC/build/temp_install,
        so do not delete that directory."
  fi
  ok "installed, and its libraries resolve"
fi

# -------------------------------------------------------------------- px4_msgs
say "px4_msgs $MSGS_BRANCH"
mkdir -p "$WS/src"
if [ ! -d "$WS/src/px4_msgs" ]; then
  git clone -b "$MSGS_BRANCH" --depth 1 \
    https://github.com/PX4/px4_msgs.git "$WS/src/px4_msgs"
else
  # A workspace reused from another project may hold another branch, whose
  # messages do not match PX4 $PX4_VERSION and fail without an error.
  have=$(git -C "$WS/src/px4_msgs" rev-parse --abbrev-ref HEAD 2>/dev/null || echo unknown)
  if [ "$have" = "$MSGS_BRANCH" ]; then
    ok "already present on $MSGS_BRANCH"
  else
    warn "$WS/src/px4_msgs is on '$have', not $MSGS_BRANCH. Its messages must
        match PX4 $PX4_VERSION: git -C $WS/src/px4_msgs fetch origin $MSGS_BRANCH
        && git -C $WS/src/px4_msgs checkout $MSGS_BRANCH, then rerun."
  fi
fi

# ------------------------------------------------------------- this package
say "Linking this package into $WS/src"
# If the repo is already inside the workspace, linking it again would make
# colcon see one package under two names, and colcon hard-errors on duplicate
# package names, so nothing would build at all.
case "$HERE/" in
  "$WS/src/"*)
    ok "repo is already inside the workspace, no link needed" ;;
  *)
    ln -sfn "$HERE" "$WS/src/avoidance_sim"
    ok "linked $HERE" ;;
esac

# -------------------------------------------------------------- dependencies
say "Installing dependencies with rosdep"
# Must run AFTER px4_msgs is in src/, because px4_msgs has no rosdep key
# (it is source-only on every architecture) and --ignore-src only skips it
# once it is physically present. Without this step the build still succeeds,
# because ament_python never imports anything, and the stack then dies at
# runtime on a missing numpy or ros_gz_bridge.
ros_source
if ! rosdep update --rosdistro jazzy >/dev/null 2>&1; then
  warn "rosdep update failed (offline?). Continuing with the existing cache."
fi
rosdep install --from-paths "$WS/src" --ignore-src -y --rosdistro jazzy \
  || fail "rosdep could not install the dependencies. The package needs
        ros_gz_bridge, rviz2, python3-numpy, sensor_msgs_py, interactive_markers
        and tf2_ros_py, plus navigation2, nav2_rviz_plugins and
        pointcloud_to_laserscan for nav2.launch.py."
ok "dependencies present"

# -------------------------------------------------------------------- builds
say "Building"
JOBS=$(nproc)
MEM_MB=$(awk '/MemTotal/{printf "%d", $2/1024}' /proc/meminfo)
# Truncate rather than round: a Pi 5 reports about 7.7 GiB, which rounds up
# to 8 and would have escaped this guard.
if [ "$MEM_MB" -lt 8192 ]; then
  JOBS=2
  echo "  $((MEM_MB / 1024)) GB of RAM, so limiting to $JOBS jobs to stay"
  echo "  clear of the out-of-memory killer. Add swap if it still dies."
fi
if [ -n "${CMAKE_GENERATOR:-}" ]; then
  warn "CMAKE_GENERATOR is set to '$CMAKE_GENERATOR'. MAKEFLAGS does not
        limit Ninja, so the job limit above will not apply."
fi

echo "  px4_msgs: 235 message definitions, no binary package exists on any"
echo "  architecture, so this is a source build. Minutes on a desktop."
ros_source
( cd "$WS" && MAKEFLAGS="-j$JOBS" colcon build --packages-select px4_msgs ) \
  || fail "px4_msgs build failed. If it was killed, that is memory: add swap
        and rerun with MAKEFLAGS=-j1."
assert_built px4_msgs
ok "px4_msgs"

( cd "$WS" && colcon build --packages-select avoidance_sim ) \
  || fail "avoidance_sim build failed."
assert_built avoidance_sim
ok "avoidance_sim"

# -------------------------------------------------------------------- renderer
say "Checking you have hardware OpenGL"
# On WSL, Mesa picks the llvmpipe software renderer unless GALLIUM_DRIVER=d3d12
# names the driver that reaches the Windows GPU. The clean-clone test ran this
# check on a machine with a perfectly good GPU and got "software rendering",
# which a reader would take for a broken GPU. So on WSL the check uses the
# driver docs/install.md tells WSL users to export, and says so.
WSL_NOTE=""
if [ -d /usr/lib/wsl/lib ] && [ -z "${GALLIUM_DRIVER:-}" ]; then
  export GALLIUM_DRIVER=d3d12
  WSL_NOTE=". This is WSL: checked with GALLIUM_DRIVER=d3d12, which every
        shell that starts Gazebo must export, or Gazebo gets llvmpipe instead"
fi
if command -v glxinfo >/dev/null; then
  # || true, because glxinfo exits non-zero with no display and pipefail
  # would otherwise abort the script right before the instructions print.
  GLX=$(glxinfo -B 2>/dev/null || true)
  REND=$(printf '%s\n' "$GLX" | sed -n 's/^OpenGL renderer string: //p')
  GLVER=$(printf '%s\n' "$GLX" | sed -n 's/^OpenGL version string: //p' \
          | grep -oE '^[0-9]+\.[0-9]+' || true)
  case "$REND" in
    "") warn "could not read a renderer. No DISPLAY set? Rendering is checked
        again when Gazebo starts." ;;
    *llvmpipe*|*softpipe*|*swrast*)
      warn "software rendering ($REND).
        Expect a real-time factor near 0.03 instead of 1.0, which is
        unusable rather than slow. Fix this before going further." ;;
    *)
      ok "hardware renderer: $REND$WSL_NOTE"
      # Gazebo's default ogre2 backend asserts OpenGL 3.3. A Raspberry Pi
      # reports a hardware renderer but caps desktop GL at 3.1, so the
      # renderer name alone is not enough.
      if [ -n "$GLVER" ] && [ "$(printf '%s\n3.3\n' "$GLVER" | sort -V | head -1)" != "3.3" ]; then
        warn "but desktop OpenGL is only $GLVER, and Gazebo's default
        renderer needs 3.3. Start PX4 with PX4_GZ_SIM_RENDER_ENGINE=ogre,
        and note that backend needs a real display."
      fi ;;
  esac
else
  printf '  note  install mesa-utils to check this (glxinfo).\n'
fi

say "Linking this repository's worlds and models into PX4's Gazebo tree"
# PX4 only loads worlds and models from its own Tools/simulation/gz, and its
# generated gz_env.sh overwrites the variables that name that directory, so
# the extra world and the lidar model are symlinked in. scripts/link_assets.sh
# says why; rerun it after adding a world or model, or after cleaning PX4.
if bash "$HERE/scripts/link_assets.sh"; then
  ok "linked"
else
  warn "not linked. PX4 is not at ~/PX4-Autopilot yet (docs/install.md, step 2); run
        scripts/link_assets.sh afterwards, or the pillars world and the lidar
        model will not be found."
fi

say "Done"
cat <<EOF
  Source the workspace, then run it in two terminals:

    source /opt/ros/jazzy/setup.bash
    source $WS/install/setup.bash

    cd ~/PX4-Autopilot && PX4_GZ_WORLD=walls HEADLESS=1 make px4_sitl gz_x500_depth
    ros2 launch avoidance_sim sim.launch.py      # brake at walls
    ros2 launch avoidance_sim nav2.launch.py     # or: plan a route round them

  The launch sets the five PX4 parameters the stack needs once PX4 answers
  (it logs them as [px4_params]); docs/how-it-works.md says why each is there.

  This script did NOT install PX4 $PX4_VERSION. See docs/install.md, step 2.
EOF
