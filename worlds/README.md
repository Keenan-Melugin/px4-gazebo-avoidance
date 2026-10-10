# Worlds

A world is a Gazebo SDF file (Simulation Description Format, an XML
dialect) describing the ground, the light, the physics and the obstacles.
`pillars.sdf` is the second world and the template; `walls` is PX4's own and
lives in its tree. The full account is
[docs/extend/worlds.md](../docs/extend/worlds.md). The short version:

1. Copy `pillars.sdf` to `yourname.sdf`. Keep everything above the first
   obstacle (physics, gravity, magnetic field, ground, sun, spherical
   coordinates). Set `<world name="yourname">`: the file's basename, because
   PX4 spawns the aircraft into `/world/<name>/create`.
2. Obstacles are static models with a model-level `<pose>` (position and
   orientation) and a `<box><size>`. That is the one shape `world_geometry.py` reads for RViz's
   wall markers and for the tests. Other shapes are simulated but not drawn
   or measured against.
3. Link it where PX4 looks and check what the tests will see:

   ```bash
   bash scripts/link_assets.sh
   python3 -m avoidance_sim.world_geometry yourname --from 0 0 --alt 7 --dir east
   ```

4. Fly it, with the same name in both terminals:

   ```bash
   cd ~/PX4-Autopilot && PX4_GZ_WORLD=yourname HEADLESS=1 make px4_sitl gz_x500_depth
   ros2 launch avoidance_sim nav2.launch.py world:=yourname
   python3 test/gate.py --world yourname
   ```

What the gate needs from a world:

- A wall east of the working area, long enough to be found from wherever the
  plan half parks the aircraft. The brake run starts at least 4 m inside the
  wall's span, and always from exactly 8 m west of its face, so that 8 m on
  that line must be clear.
- A wall north of the plan half's start (`--start`, east -2 north -9 by
  default) whose ends the camera can see from there.

The checks use the world file, so the geometry printer in step 3 shows
what the gate will find. `PX4_GZ_MODEL_POSE="x,y,z,roll,pitch,yaw"` spawns
the aircraft elsewhere. Wind, lighting and the other conditions are in
[docs/data.md](../docs/data.md).
