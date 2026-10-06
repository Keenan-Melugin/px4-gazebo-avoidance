#!/usr/bin/env bash
# Link this repository's worlds/ and models/ into PX4's Gazebo tree.
#
# PX4 builds the world path itself, as ${PX4_GZ_WORLDS}/${PX4_GZ_WORLD}.sdf,
# and the model path as ${PX4_GZ_MODELS}/<name>/model.sdf, and the gz_env.sh
# its build generates exports both variables unconditionally (PX4 v1.17.0,
# src/modules/simulation/gz_bridge/gz_env.sh.in). Setting them in the
# environment therefore does nothing: a world or model PX4 is to start has
# to be present in Tools/simulation/gz/{worlds,models}. A symlink puts it
# there while the one copy stays here, under version control. install.sh
# runs this; run it again after adding a world or a model, and after
# anything that cleans the PX4 tree (git clean in the submodule removes the
# links, and PX4 then says "world not found" for a world that is still here).
set -euo pipefail

HERE=$(cd "$(dirname "$0")/.." && pwd)
PX4=${PX4_ROOT:-$HOME/PX4-Autopilot}
GZ="$PX4/Tools/simulation/gz"

if [ ! -d "$GZ/worlds" ] || [ ! -d "$GZ/models" ]; then
  echo "PX4's Gazebo tree is not at $GZ."
  echo "Clone PX4 with --recursive first (README, Install, step 2), or set PX4_ROOT."
  exit 1
fi

n=0
for f in "$HERE"/worlds/*.sdf; do
  [ -e "$f" ] || continue
  ln -sfn "$f" "$GZ/worlds/$(basename "$f")"
  printf '  world  %-20s -> %s\n' "$(basename "$f" .sdf)" "$GZ/worlds/"
  n=$((n + 1))
done
for d in "$HERE"/models/*/; do
  [ -d "$d" ] || continue
  name=$(basename "$d")
  ln -sfn "${d%/}" "$GZ/models/$name"
  printf '  model  %-20s -> %s\n' "$name" "$GZ/models/"
  n=$((n + 1))
done
echo "  linked $n. git status inside $GZ will list them as untracked; that is expected."
