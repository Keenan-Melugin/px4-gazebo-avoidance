#!/usr/bin/env bash
# Set the PX4 parameters this stack needs, once PX4 is up. sim.launch.py runs
# this; you can also run it by hand after starting PX4.
#
#   px4_params.sh <px4 build bin dir> "<NAME=value> <NAME=value> ..."
#
# Why the launch does this rather than the README asking for pxh> typing, or
# PX4_PARAM_ environment variables on the PX4 command: the clean-clone test
# showed that a fresh install never arms. The x500 airframe defaults
# NAV_DLL_ACT to 2 (wait for a ground station) and the development machine
# had 0 saved for months, so nothing documented it. PX4_PARAM_NAV_DLL_ACT=0
# does not fix it either: rcS applies the environment BEFORE it sources the
# airframe file, and PX4 records a value equal to the compiled default (0) as
# "still default", so the airframe's later set-default 2 wins. Measured on the
# clean instance: the other three applied, that one did not. Setting it after
# boot through PX4's own client is the one way that works for all four.
set -u
BIN=${1:?px4 build bin dir}
PARAMS=${2:-}
P="$BIN/px4-param"
tag="[px4_params]"

if [ -z "$PARAMS" ]; then
  echo "$tag nothing to set (px4_params is empty)"
  exit 0
fi
if [ ! -x "$P" ]; then
  echo "$tag $P not found. PX4 is not at the default place, so pass px4_bin:=... to the"
  echo "$tag launch, or type these at the pxh> prompt: $PARAMS"
  exit 0
fi

# PX4 may not be up yet, or may be rebuilding. Wait, but not forever.
for _ in $(seq 1 90); do
  timeout 5 "$P" show SYS_AUTOSTART >/dev/null 2>&1 && break
  sleep 2
done
if ! timeout 5 "$P" show SYS_AUTOSTART >/dev/null 2>&1; then
  echo "$tag PX4 did not answer in 180 s. When it is up, type at pxh>: $PARAMS"
  exit 0
fi

for kv in $PARAMS; do
  name=${kv%%=*}
  value=${kv#*=}
  out=$(timeout 10 "$P" set "$name" "$value" 2>&1 | grep -aE "curr|unchanged|error|not found" | tail -1 | sed 's/^\s*//')
  echo "$tag $name = $value  (${out:-set})"
done
echo "$tag done. NAV_DLL_ACT 0 is the one a fresh install cannot fly without."
