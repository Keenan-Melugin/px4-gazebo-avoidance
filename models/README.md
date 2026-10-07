# Models

`x500_depth_lidar` is the stock aircraft plus a 2D lidar, and the template
for a model with one more sensor. The full account, with the PX4 source lines
that decide the mechanism, is [docs/extend.md](../docs/extend.md), sections 2
and 3. The short version, in order of effort:

## A sensor variant of the stock aircraft

1. Copy `x500_depth_lidar/` to `yourname/`, rename the `<model name>` inside,
   and change the sensor. A Gazebo sensor's `<topic>` is what the bridge sees
   (`/yourtopic`); give it a `<gz_frame_id>` and a link of its own. Do not
   name a lidar `lidar_2d_v2` on a link called `link`: that is the one name
   PX4's own bridge consumes, and PX4 would then publish a second histogram.
2. `bash scripts/link_assets.sh` (the model has to be in PX4's tree; the
   link script puts it there).
3. Start PX4 on it with the stock airframe's parameters. There is no `make`
   target for a model PX4 does not know, so run the binary:

   ```bash
   cd ~/PX4-Autopilot && PX4_SYS_AUTOSTART=4002 PX4_SIM_MODEL=gz_yourname \
       PX4_GZ_WORLD=walls HEADLESS=1 ./build/px4_sitl_default/bin/px4
   ```

4. Describe the sensor to the obstacle node in a copy of
   `config/sensors_example.yaml`, bridge its topic, and launch:

   ```bash
   ros2 launch avoidance_sim sim.launch.py sensors:=/path/to/yours.yaml \
       bridge_extra:="/yourtopic@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"
   ```

   Add the sensor's static transform to `frames.py` and `tf_publisher.py`
   if RViz should draw it. Then `python3 test/histogram_selftest.py` with a
   case for it, then the gate.

Meshes and textures go in the model directory next to `model.sdf`, with a
`model.config` naming the SDF file, and are referenced as `model://yourname/
meshes/part.dae`; Gazebo finds them through the same link.

## A different vehicle

A vehicle that is not a quadrotor needs its own PX4 airframe file as well as
a model. `airframe_template/4100_gz_mymodel` is a commented template; the
steps are inside it. PX4 ships a four-rotor tailsitter (`quadtailsitter`,
airframe `4018`) and a standard VTOL with a pusher (`standard_vtol`, `4004`),
which are the starting points for the project's aircraft. Expect the
measured numbers to change: the pilot's yaw and stick models were measured
on the x500 (`test/yaw_threshold.py`, `test/xy_threshold.py` re-measure
them), the standoff depends on mass and speed, and collision prevention
applies in hover and multicopter flight only.

## Wind, if the model should feel it

Gazebo only applies wind to links that opt in: `<enable_wind>true</enable_wind>`
inside the `<link>`. The stock x500 does not, so PX4's `windy` world (a
`<wind><linear_velocity>` of 5 m/s east, 2 m/s north) leaves it untouched. A
copy of `x500_base/model.sdf` with that element on `base_link`, plus the
`WindEffects` system in the world, is the recipe; untested here.
[docs/data.md](../docs/data.md) has the other conditions.
