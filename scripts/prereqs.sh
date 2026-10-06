#!/usr/bin/env bash
# Install the ROS 2 and Gazebo layer on a clean Ubuntu 24.04 machine.
#
# This is the layer install.sh checks for but does not install: ROS 2 Jazzy,
# its build tools, Gazebo Harmonic and the ros_gz bridge packages. It follows
# the two projects' own install pages command for command and checks each
# result:
#
#   ROS 2 Jazzy, Ubuntu deb packages
#     https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
#   Gazebo Harmonic, Ubuntu binaries
#     https://gazebosim.org/docs/harmonic/install_ubuntu/
#   ros_gz, which pairs Jazzy with Harmonic from packages.ros.org
#     https://github.com/gazebosim/ros_gz/blob/jazzy/README.md
#
# Safe to rerun: each step is skipped when its result is already present.
# It does not install PX4 (README, Install, step 2) or this package
# (scripts/install.sh).
set -euo pipefail

say()  { printf '\n=== %s ===\n' "$1"; }
ok()   { printf '  ok    %s\n' "$1"; }
warn() { printf '  warn  %s\n' "$1"; }
fail() { printf '  FAIL  %s\n' "$1" >&2; exit 1; }

# ---------------------------------------------------------------- preflight
say "Checking the platform"

[ "${EUID:-$(id -u)}" -ne 0 ] \
  || fail "do not run this as root. It calls sudo itself where it needs to."

[ -r /etc/os-release ] || fail "cannot read /etc/os-release, so I cannot tell
        what this machine is. This needs Ubuntu 24.04 (noble)."
# shellcheck disable=SC1091
. /etc/os-release
CODENAME="${UBUNTU_CODENAME:-${VERSION_CODENAME:-}}"
[ "$CODENAME" = "noble" ] \
  || fail "this needs Ubuntu 24.04 (noble). Found ${PRETTY_NAME:-unknown}.
        ROS 2 Jazzy has binary packages for noble only. Raspberry Pi OS is
        Debian, and the ROS index for it holds zero ros-jazzy packages."
ARCH=$(dpkg --print-architecture)
ok "Ubuntu 24.04 / noble ($ARCH)"

# Ask for sudo now rather than halfway through a download.
# Not plain `sudo -v`. With sudo's default verifypw=all, -v demands a password
# whenever ANY rule matching the user lacks NOPASSWD, so a user who can already
# run sudo without a password (Raspberry Pi and cloud images, where the first
# user is also in the sudo group) is told to type one they may not have.
# Found by this repo's own clean-clone test on its first run.
sudo -n true 2>/dev/null || sudo -v || fail "this needs sudo to install packages."
ok "sudo"

# ------------------------------------------------------------------- locale
# ROS 2 needs a UTF-8 locale. Minimal cloud and container images sometimes
# ship with POSIX; WSL and desktop Ubuntu are already UTF-8.
say "Locale"
if locale 2>/dev/null | grep -qiE '^LANG=.*utf-?8'; then
  ok "already UTF-8 ($(locale 2>/dev/null | sed -n 's/^LANG=//p'))"
else
  sudo apt-get update
  sudo apt-get install -y locales
  sudo locale-gen en_US en_US.UTF-8
  sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
  export LANG=en_US.UTF-8
  ok "set to en_US.UTF-8. It takes full effect at your next login."
fi

# ------------------------------------------------------------- apt sources
say "Apt repositories"
sudo apt-get update
sudo apt-get install -y software-properties-common curl lsb-release gnupg
sudo add-apt-repository -y universe

# ROS 2. The ros2-apt-source package carries the signing key and the source
# line and keeps both current. Its version comes from the GitHub releases
# API, which allows an unauthenticated address 60 calls an hour, so an empty
# answer here is rate limiting or no network, not a broken machine.
if dpkg -s ros2-apt-source >/dev/null 2>&1; then
  ok "ros2-apt-source already installed ($(dpkg-query -W -f='${Version}' ros2-apt-source))"
elif ls /etc/apt/sources.list.d/ros2* >/dev/null 2>&1; then
  ok "a ROS 2 apt source is already configured ($(ls /etc/apt/sources.list.d/ros2* | head -1))"
else
  ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
    | grep -F "tag_name" | awk -F'"' '{print $4}')
  [ -n "$ROS_APT_SOURCE_VERSION" ] \
    || fail "could not read the ros2-apt-source version from the GitHub API.
        Offline, or rate limited? Wait a few minutes and rerun, or follow
        the ROS 2 install page by hand (link at the top of this script)."
  curl -fL -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${CODENAME}_all.deb" \
    || fail "download of ros2-apt-source ${ROS_APT_SOURCE_VERSION} failed."
  sudo dpkg -i /tmp/ros2-apt-source.deb || fail "ros2-apt-source did not install."
  ok "ros2-apt-source $ROS_APT_SOURCE_VERSION"
fi

# Gazebo. packages.osrfoundation.org, signed with its own key. ros-jazzy-ros-gz
# can pull Gazebo in by itself, but the Gazebo page is the primary source for
# gz-harmonic, and adding its repository makes every machine end up the same.
if [ -f /etc/apt/sources.list.d/gazebo-stable.list ]; then
  ok "Gazebo apt source already configured"
else
  sudo curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
    --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    || fail "could not download the Gazebo signing key."
  echo "deb [arch=$ARCH signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
  ok "Gazebo apt source added"
fi

# ----------------------------------------------------------------- packages
say "Installing ROS 2 Jazzy, build tools, Gazebo Harmonic and ros_gz"
sudo apt-get update
# The ROS 2 page asks for a full upgrade before installing Jazzy on noble:
# systemd and udev must be current or the install can leave the system in a
# bad state. It upgrades everything, so on a fresh image it takes a while.
sudo apt-get full-upgrade -y
sudo apt-get install -y \
  ros-jazzy-desktop \
  ros-dev-tools \
  gz-harmonic \
  ros-jazzy-ros-gz \
  python3-colcon-common-extensions \
  python3-rosdep \
  build-essential cmake git \
  mesa-utils
ok "packages installed"

# rosdep needs a one-time system init and a per-user cache.
if [ -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  ok "rosdep already initialised"
else
  sudo rosdep init || fail "rosdep init failed."
  ok "rosdep initialised"
fi
if rosdep update --rosdistro jazzy >/dev/null 2>&1; then
  ok "rosdep cache updated"
else
  warn "rosdep update failed (offline?). install.sh tries it again."
fi

# ------------------------------------------------------------------- checks
say "Checking the result"
[ -r /opt/ros/jazzy/setup.bash ] \
  || fail "/opt/ros/jazzy/setup.bash is missing after the install."
ok "ROS 2 Jazzy at /opt/ros/jazzy"

for t in colcon rosdep git cmake make g++; do
  command -v "$t" >/dev/null || fail "$t not found after the install."
done
ok "build tools"

GZ_VER=$(gz sim --versions 2>/dev/null | head -1 || true)
case "$GZ_VER" in
  8.*) ok "Gazebo Harmonic $GZ_VER" ;;
  *)   fail "expected Gazebo 8.x (Harmonic). 'gz sim --versions' says
        '${GZ_VER:-nothing}'." ;;
esac

dpkg -s ros-jazzy-ros-gz-bridge >/dev/null 2>&1 \
  || fail "ros-jazzy-ros-gz-bridge did not install."
ok "ros_gz_bridge"

say "Done"
cat <<EOF
  The ROS 2 and Gazebo layer is installed. Next:

    1. PX4 v1.17.0, which has its own setup script: README, Install, step 2.
    2. ./scripts/install.sh, for the agent, px4_msgs and this package.

  Every new terminal needs:

    source /opt/ros/jazzy/setup.bash
EOF
if [ -d /usr/lib/wsl/lib ]; then
  cat <<EOF

  This is WSL. Gazebo needs the D3D12 driver to reach your GPU; without it
  Mesa falls back to software rendering and the simulation runs about thirty
  times too slow (real-time factor 0.033 measured, against 1.00):

    export GALLIUM_DRIVER=d3d12      # put this in ~/.bashrc
EOF
fi
