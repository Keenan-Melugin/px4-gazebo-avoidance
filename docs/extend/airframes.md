# Add an airframe

Not built. What a reader faces today, and the path. An airframe here means
two things: the Gazebo model of the vehicle's body, and PX4's airframe file,
the startup script that sets the vehicle's parameters (rotor layout, mass
limits, controller gains). Read "Where PX4 looks" in
[../extend.md](../extend.md) first: models have to be linked into PX4's tree.

## Where the x500 is assumed

In five places with no list of them:

- the run command names `gz_x500_depth`;
- `patches/px4-camera-res.patch` targets the OakD-Lite model;
- the camera's mount is the obstacle node's default `mount_xyz_frd` and
  `frames.py`'s `CAM_XYZ`;
- `config/nav2.yaml` sets `robot_radius: 0.3`;
- the pilot's gains and dead bands in `software_pilot.py` were measured on
  it (`test/yaw_threshold.py`, `test/xy_threshold.py`).

The hook to write is an airframe profile: one YAML file with the model name,
the sensors (in the `sources` schema of [sensors.md](sensors.md)), the radius
and the inertial facts. The launch passes it to every node (`parameters=`
takes a file) and into Nav2 through `nav2_common.launch.RewrittenYaml`, which
rewrites named keys of `nav2.yaml` at launch time and is how Nav2's own
bring-up overrides values. The pilot's gains stay in the pilot; the profile
names the two scripts that re-measure them.

## The PX4 side

A model is a directory under `models/`, linked in as above;
`models/README.md` orders the steps and `models/airframe_template/` holds a
commented PX4 airframe file. If the vehicle is still a quadrotor with
different sensors, start it with `PX4_SYS_AUTOSTART=4002` as in
[sensors.md](sensors.md) and no airframe file.

If the vehicle is different, it needs its own airframe file in PX4's tree:
`ROMFS/px4fmu_common/init.d-posix/airframes/NNNN_gz_<model>`, where `NNNN` is
an unused number. It sets `PX4_SIMULATOR`, `PX4_GZ_WORLD`, `PX4_SIM_MODEL` and
the vehicle's parameters. Then it needs a line in that directory's
`CMakeLists.txt`, the build recipe that lists the files, and a clean build
(PX4's "Adding a New Airframe Configuration" and the Gazebo page).

A VTOL (vertical take-off and landing) aircraft takes off like a multicopter
and cruises on a wing. For a four-motor VTOL with no cruise motor, the
starting points exist in PX4's own models and airframes:

- `quadtailsitter` with `4018_gz_quadtailsitter` (`CA_AIRFRAME 4`,
  `VT_TYPE 0`) matches. A tailsitter takes off pointing up on its tail and
  pitches the whole body forward to fly on the wing.
- `standard_vtol` with `4004_gz_standard_vtol` applies if a pusher is added:
  a rear-facing propeller that drives the aircraft forward in cruise while
  the lift rotors stop.
- `tiltrotor` is the third: rotors that tilt from vertical to horizontal.

Copy the airframe, point it at your model, give it a new number.

## What changes with a VTOL

Collision prevention lives in the multicopter position controller and stops
during transition, the phase when a VTOL changes from hover to wing-borne
flight. So the gate's brake half applies to hover and multicopter flight
only, and the plan half's Offboard velocity setpoints likewise. The
measurements to add are the ones a VTOL forces: the standoff at the real
mass, the yaw dead band of the new airframe (here the edge lies between 0.10 and
0.12, and the pilot steps just inside it at `YAW_DZ` 0.105; a VTOL's will differ), and what
collision prevention does in the seconds around a transition.

## Order

The model flies in Gazebo under PX4's stock airframe before any sensor is
added; then the camera, at its measured mount; then the profile; then the
gate, with the README's status table gaining a row per airframe.

## Sources

- PX4 user guide v1.17: Adding a New Airframe Configuration, Gazebo
  Simulation (vehicles and their airframe numbers), VTOL.
  https://docs.px4.io/v1.17/en/
- PX4-Autopilot v1.17.0: `ROMFS/px4fmu_common/init.d-posix/airframes/`
  (`4018_gz_quadtailsitter`, `4004_gz_standard_vtol`, `4020_gz_tiltrotor`)
  and that directory's `CMakeLists.txt`; the transition flight task, which
  does not link collision prevention: its sources under
  `src/modules/flight_mode_manager/tasks/Transition/` never include
  `CollisionPrevention`.
- PX4-gazebo-models at `b6127f4`: `quadtailsitter`, `standard_vtol`,
  `tiltrotor`. https://github.com/PX4/PX4-gazebo-models
- Nav2 Jazzy: `nav2_common/launch/rewritten_yaml.py`.
  https://github.com/ros-navigation/navigation2/tree/jazzy/nav2_common
