#!/usr/bin/env bash
# Print what this machine is and what state the stack is in, so a problem
# report carries facts. No sudo, changes nothing. Paste the output when asking
# for help, or send it after a run on a platform the README lists as untested.
#
#   ~/px4-gazebo-avoidance/scripts/report.sh
set -u
line() { printf '%-22s %s\n' "$1" "$2"; }
echo "== machine =="
line "date" "$(date -u +%Y-%m-%dT%H:%MZ)"
line "os" "$( (. /etc/os-release 2>/dev/null && echo "$PRETTY_NAME") || uname -s)"
line "kernel" "$(uname -r)"
line "arch" "$(uname -m)"
line "cpus" "$(nproc 2>/dev/null || echo ?)"
line "memory" "$(free -g 2>/dev/null | awk '/Mem:/{print $2 " GB total, " $7 " GB available"}')"
line "swap" "$(free -g 2>/dev/null | awk '/Swap:/{print $2 " GB"}')"
line "disk free (home)" "$(df -h "$HOME" 2>/dev/null | awk 'NR==2{print $4}')"
line "virtualisation" "$(systemd-detect-virt 2>/dev/null || echo unknown)"
[ -d /usr/lib/wsl/lib ] && line "wsl" "yes (GALLIUM_DRIVER=${GALLIUM_DRIVER:-unset})"

echo "== graphics =="
if command -v glxinfo >/dev/null; then
  if [ -d /usr/lib/wsl/lib ] && [ -z "${GALLIUM_DRIVER:-}" ]; then export GALLIUM_DRIVER=d3d12; fi
  GLX=$(glxinfo -B 2>/dev/null || true)
  line "renderer" "$(printf '%s\n' "$GLX" | sed -n 's/^OpenGL renderer string: //p')"
  line "opengl" "$(printf '%s\n' "$GLX" | sed -n 's/^OpenGL version string: //p')"
  case "$GLX" in *llvmpipe*|*softpipe*|*swrast*) line "WARNING" "software rendering: the simulation will run about 30x too slow" ;; esac
else
  line "renderer" "glxinfo not installed (apt install mesa-utils)"
fi
line "display" "DISPLAY=${DISPLAY:-unset} WAYLAND_DISPLAY=${WAYLAND_DISPLAY:-unset}"

echo "== versions =="
line "ros" "$( [ -r /opt/ros/jazzy/setup.bash ] && echo "jazzy at /opt/ros/jazzy" || echo "not found")"
line "gazebo" "$(gz sim --versions 2>/dev/null | head -1 || echo "not found")"
line "px4 tree" "$( [ -d "$HOME/PX4-Autopilot" ] && git -C "$HOME/PX4-Autopilot" describe --tags 2>/dev/null || echo "not found at ~/PX4-Autopilot")"
line "px4 built" "$( [ -x "$HOME/PX4-Autopilot/build/px4_sitl_default/bin/px4" ] && echo yes || echo no)"
line "camera patch" "$( [ -d "$HOME/PX4-Autopilot/Tools/simulation/gz" ] && (git -C "$HOME/PX4-Autopilot/Tools/simulation/gz" diff --quiet -- models/OakD-Lite 2>/dev/null && echo "not applied" || echo "applied") || echo "?")"
line "agent" "$(command -v MicroXRCEAgent || echo "not on PATH")"
WS=${WS:-$HOME/av_ws}
line "workspace" "$( [ -f "$WS/install/setup.bash" ] && echo "$WS" || echo "not built at $WS")"
line "px4_msgs" "$( [ -d "$WS/src/px4_msgs" ] && git -C "$WS/src/px4_msgs" rev-parse --abbrev-ref HEAD 2>/dev/null || echo "not in $WS/src (install.sh puts it there)")"
line "this repo" "$(git -C "$(dirname "$0")/.." log -1 --format='%h %s' 2>/dev/null | cut -c1-70)"

echo "== running stack =="
line "px4 processes" "$(pgrep -cf 'bin/px4' 2>/dev/null)"
line "gazebo processes" "$(pgrep -cf 'gz sim' 2>/dev/null)"
line "agent processes" "$(pgrep -c MicroXRCEAgent 2>/dev/null)"
if [ -r /opt/ros/jazzy/setup.bash ] && [ -f "$WS/install/setup.bash" ]; then
  set +u; . /opt/ros/jazzy/setup.bash; . "$WS/install/setup.bash"; set -u
  # grep -c prints 0 itself when nothing matches; || true, not || echo 0,
  # or the report printed a second 0 on a line of its own.
  line "fmu topics" "$(timeout 15 ros2 topic list 2>/dev/null | grep -c '^/fmu/out/' || true)"
  # --no-daemon: a stale daemon answers 0 for a stack that is plainly running
  line "nodes" "$(timeout 20 ros2 node list --no-daemon 2>/dev/null | wc -l)"
fi
if command -v gz >/dev/null && [ "$(pgrep -cf 'gz sim')" != "0" ]; then
  # The running world's name, not walls: the stats topic is per world.
  W=$(timeout 10 gz topic -l 2>/dev/null | grep -m1 -E '^/world/[^/]+/stats$' | cut -d/ -f3)
  line "world" "${W:-unknown}"
  line "real-time factor" "$(timeout 10 gz topic -e -t /world/${W:-walls}/stats -n 3 2>/dev/null | grep -oE 'real_time_factor: [0-9.]+' | tail -1 | cut -d' ' -f2)"
fi
BIN="$HOME/PX4-Autopilot/build/px4_sitl_default/bin"
if [ -x "$BIN/px4-listener" ] && [ "$(pgrep -cf 'bin/px4')" != "0" ]; then
  line "obstacle_distance" "$(timeout 10 "$BIN/px4-listener" obstacle_distance 1 2>/dev/null | grep -aoE 'timestamp: [0-9]+ \([^)]*\)' | head -1 || echo "not arriving")"
  line "arming / mode" "$(timeout 10 "$BIN/px4-commander" status 2>/dev/null | grep -aiE 'arming|nav state|mode' | head -2 | tr -s ' ' | paste -sd'|')"
  for p in NAV_DLL_ACT NAV_RCL_ACT CP_DIST CP_GO_NO_DATA; do
    # the first output line is a legend; the value is on the line naming the parameter
    line "param $p" "$(timeout 10 "$BIN/px4-param" show "$p" 2>/dev/null | grep -a "$p" | grep -aoE ': .*' | head -1 | cut -c3-)"
  done
fi
echo "== end =="
