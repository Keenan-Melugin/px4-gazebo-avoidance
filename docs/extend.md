# Extend it

[change-it.md](change-it.md) is for changing a number and proving the change.
These pages are for adding a thing. Each one says what PX4 and Gazebo
actually do (read in their source, version given), which files change, how to
run it, and what to measure.

| You want to add | Read | State |
|---|---|---|
| A world: a test site, an obstacle layout | [extend/worlds.md](extend/worlds.md) | Built and measured |
| A sensor: a lidar, a second camera, a real driver | [extend/sensors.md](extend/sensors.md) | Built and measured |
| An airframe: a different drone, a VTOL, the project's aircraft | [extend/airframes.md](extend/airframes.md) | Not built; the path and the hook to write |
| A second machine: the ROS side on a Raspberry Pi or companion computer | [extend/second-machine.md](extend/second-machine.md) | Not built; the path and the hook to write |

Worlds and airframes share one fact about PX4, so it lives here.

## Where PX4 looks, and why there is a link script

PX4 starts Gazebo itself and builds the world path as
`${PX4_GZ_WORLDS}/${PX4_GZ_WORLD}.sdf`, and spawns the aircraft from
`${PX4_GZ_MODELS}/<name>/model.sdf` (PX4 v1.17.0,
`ROMFS/px4fmu_common/init.d-posix/px4-rc.gzsim`). ROMFS is the read-only file
system PX4 ships its startup scripts in; `px4-rc.gzsim` is the script that
starts Gazebo. Both variables come from `gz_env.sh`, which PX4's build
generates and the startup script sources, and it exports them unconditionally
to `Tools/simulation/gz/{worlds,models}`
(`src/modules/simulation/gz_bridge/gz_env.sh.in`). Setting them in your shell
therefore changes nothing. `GZ_SIM_RESOURCE_PATH` is appended to rather than
replaced, but it only resolves `model://` URIs inside files; it does not
decide what PX4 starts. PX4's own documentation says the same in fewer words:
worlds and models go in those two directories.

So `scripts/link_assets.sh` creates symlinks (file-system shortcuts that
point to a file elsewhere) from this repository's `worlds/*.sdf` and
`models/*/` into them. `install.sh` runs it. Run it again after adding a world
or model, and after anything that cleans the PX4 tree: `git clean` in the
submodule removes the links, and PX4 then reports a world it cannot find for a
file that is still here. `git status` inside `Tools/simulation/gz` lists the
links as untracked, which is expected.

The other route, for a world that lives nowhere near either repository, is to
start Gazebo yourself and let PX4 attach. The startup script looks for a
running world (`gz topic -l`, a `/world/*/clock` topic) before launching one,
and uses it if it finds one. Source `build/px4_sitl_default/rootfs/gz_env.sh`
first, because that is where the server configuration comes from: the stock
worlds carry no `<plugin>` elements since PX4-gazebo-models #84, and PX4's
`server.config` supplies physics, sensors, IMU, GPS and the rest. Then
`gz sim -r -s /any/path/world.sdf`, then `make px4_sitl gz_x500_depth` as
usual. `world_sdf:=/any/path/world.sdf` on the launch draws its walls.

### Sources

- PX4-Autopilot v1.17.0: `ROMFS/px4fmu_common/init.d-posix/px4-rc.gzsim`
  (world and model paths, attach to a running world),
  `src/modules/simulation/gz_bridge/gz_env.sh.in` (unconditional exports).
- PX4 user guide v1.17, Gazebo Simulation ("Adding New Worlds and Models").
  https://docs.px4.io/v1.17/en/sim_gazebo_gz/
- PX4-gazebo-models #84 (plugins removed from worlds, 2025-03).
  https://github.com/PX4/PX4-gazebo-models
- Gazebo Harmonic, gz-sim 8 `Util.cc` (world file resolution) and the
  resources tutorial. https://gazebosim.org/api/sim/8/resources.html
