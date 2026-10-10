# Add a world

What is here: `worlds/pillars.sdf`, the second world and the template, and
`avoidance_sim/world_geometry.py`, which reads a world's boxes for RViz's wall
markers and for the measurement scripts, so a new world needs no numbers
typed anywhere. Read "Where PX4 looks" in [../extend.md](../extend.md) first:
it is why a world has to be linked into PX4's tree.

## Make one

Copy `pillars.sdf`. Keep everything above the first obstacle: the physics
step, gravity and magnetic field PX4's simulated sensors expect, the ground
plane, the sun, and the spherical coordinates. Those are the latitude,
longitude and elevation that tie the world to a place on Earth and give PX4
its home.
Set `<world name>` to the file's basename. That name is what `PX4_GZ_WORLD`
and `world:=` refer to, and the startup script spawns the aircraft into
`/world/<name>/create`, so a mismatch is a world with no aircraft.

Obstacles are static models with a model-level `<pose>` (position and
orientation) and a `<box><size>`; that is the one shape the parser reads. A
yawed box (one rotated about the vertical) is drawn but skipped by the face
queries, and anything that is not a box is drawn by nothing. Then:

```bash
bash scripts/link_assets.sh
python3 -m avoidance_sim.world_geometry pillars --from 0 0 --alt 7 --dir east
```

The second line prints the boxes as the tests will see them and the faces
ahead of a point, which is the check to do before flying anything.

## Fly it

The world name goes to PX4 and to the launch, so the walls RViz draws are
the walls PX4 loaded:

```bash
cd ~/PX4-Autopilot && PX4_GZ_WORLD=pillars HEADLESS=1 make px4_sitl gz_x500_depth
ros2 launch avoidance_sim nav2.launch.py world:=pillars
python3 test/gate.py --world pillars
```

`PX4_GZ_MODEL_POSE="x,y,z,roll,pitch,yaw"` spawns the aircraft somewhere
other than the origin, in metres and radians, missing values zero.

## What the gate assumes about a world

Its brake half flies east from wherever the aircraft is and looks for the
first box face on that line, at that altitude. So a world needs a wall east
of the working area, long enough to be found from anywhere the plan half
parks the aircraft; that is why `wall_east` is 26 m. If there is none the
test says so and does not fly.

It backs off to 8 m from the face for a run-up, pushes, and reads the
closest approach, stopping the push once the aircraft has stood still for
4 s. Pushed on after braking, the aircraft creeps along the face toward free
space, 3 m in 14 s measured, which is why the end of a fixed push is the
wrong thing to read. It also moves at least 4 m inside that wall's span
first, because near a wall's end collision prevention does something else
that is also correct. `CP_GUIDE_ANG` (30 degrees, the angle PX4 may steer a
blocked setpoint toward free space) makes the aircraft slide round the end
instead of stopping. Measured once, from 0.5 m inside the walls world's box1
end: it went round that wall and the next and reached east 100 with nothing
left to brake at.

Its plan half positions at `--start` (east -2, north -9 by default) and asks
for `--goal` beyond the first box north of there. It needs that box's ends
within the camera's view from the start; from 13.5 m back a 73 degree arc is
20 m wide. `python3 test/nav2_flight.py --world x --start E N --goal E N`
moves the scenario.

## Measured on pillars

The dev machine, 2026-10-07: the whole gate passed in 220 s with nothing
changed but `--world pillars`. Brake half: flying east from the origin at
full stick, the aircraft braked 2.04 m from `wall_east`. On the way, the two
pillars 4 m either side of the line pushed it 3.9 m north (`CP_GUIDE_ANG`
again, toward the freer side). Plan half: the 6 m `wall_north` from 13.5 m
back, reached with a 4.2 m sideways excursion and 40 plans, against 6.2 to
7.6 m round the walls world's 10 m wall. Headings within 5.6 degrees, as on
walls.

## What it is for

The walls world taught the numbers in the README: the standoff and its
spread, the 10 m of observation distance a 10 m wall needs. Pillars at the
edge of the arc teach a different thing: a one-metre post fills one bin,
enters the histogram late and off-axis, and collision prevention limits the
velocity component toward it rather than stopping. A world with a person
walking through it is the next one to build, because it is the first that
breaks the never-clearing global costmap ([../how-it-works.md](../how-it-works.md));
the fix is a measured change, not a setting.

## Sources

- PX4-Autopilot v1.17.0, `ROMFS/px4fmu_common/init.d-posix/px4-rc.gzsim`
  (`/world/<name>/create`, `PX4_GZ_MODEL_POSE`).
- PX4 user guide v1.17, Gazebo Simulation (`PX4_GZ_MODEL_POSE`) and Collision
  Prevention (`CP_GUIDE_ANG`). https://docs.px4.io/v1.17/en/
- PX4-gazebo-models at the commit v1.17.0 pins (`b6127f4`): `walls.sdf`, the
  template for `pillars.sdf`'s header. https://github.com/PX4/PX4-gazebo-models
- SDFormat 1.11 specification (world, model, pose, box).
  http://sdformat.org/spec
