# Install: from an empty Ubuntu 24.04 to a flying aircraft

Every step here ends in a check you run. Every step was run, in this order,
on a fresh Ubuntu 24.04 with 4 cores, 8 GB and a spinning disk, and the
durations quoted are from that machine. A faster machine will be faster; an
SSD figure has not been measured yet.

## Before you start

You need Ubuntu 24.04 with hardware OpenGL, about 20 GB of disk, 8 GB of RAM,
an internet connection, and roughly an hour and a half. The README has the
measured hardware table. Three cases to know about now:

- **Windows.** WSL2 is the tested platform. Install Ubuntu 24.04 from the
  Microsoft Store or `wsl --install Ubuntu-24.04`, and make sure your GPU
  driver is current, because WSL reaches the GPU through it.
- **Mac.** An Ubuntu 24.04 arm64 virtual machine with GPU acceleration, on a
  Mac with 16 GB or more. Untested. Notes at the end.
- **Raspberry Pi 5 on Ubuntu 24.04 arm64.** Untested. Notes at the end.
- **Raspberry Pi OS or any other Debian.** Not possible: there are no ROS 2
  Jazzy packages for it.

The stack has five layers, and the scripts cover three of them:

| Layer | What | Installed by |
|---|---|---|
| 1 | Ubuntu 24.04 | You |
| 2 | ROS 2 Jazzy, Gazebo Harmonic, build tools | `scripts/prereqs.sh` (step 1) |
| 3 | PX4 v1.17.0, built for simulation | You, with PX4's own setup script (step 2) |
| 4 | The DDS agent, `px4_msgs`, this package, Nav2 | `scripts/install.sh` (step 3) |
| 5 | The optional camera patch | You (step 4) |

PX4 has its own setup script and this repository does not wrap it. It is a
firmware build with a toolchain installer that changes your groups and your
shell, and burying that inside another script would hide its failures.

## Step 1: ROS 2 Jazzy and Gazebo Harmonic

```bash
sudo apt install -y git
git clone https://github.com/Keenan-Melugin/px4-gazebo-avoidance.git ~/px4-gazebo-avoidance
~/px4-gazebo-avoidance/scripts/prereqs.sh
```

The first line is there because the Ubuntu Desktop image ships without git,
found on the first install from that image in a VM; the WSL image has it.
Nothing else can fetch the repository, so it cannot be in the script.

The script adds the ROS 2 and Gazebo apt repositories and installs
`ros-jazzy-desktop`, `ros-dev-tools`, `gz-harmonic`, `ros-jazzy-ros-gz` and
the build tools. It follows the two projects' own install pages command for
command; the links are at the top of the script. It runs a full
`apt upgrade` first because the ROS 2 page asks for one. Every step is skipped
when its result is already present, so it is safe on a machine that has some
of this. It asks for your password once, up front, unless you already have
passwordless sudo.

Measured: 57 minutes on the test machine, nearly all of it dpkg configuring
about 700 packages on a spinning disk. Memory never passed 0.8 GB.

Check, in a new terminal:

```bash
source /opt/ros/jazzy/setup.bash
ros2 --help | head -1          # prints usage
gz sim --versions              # 8.x
```

The script's last lines print your OpenGL renderer. It must not say
`llvmpipe`; that is software rendering, and the simulation would run thirty
times too slow. On WSL the script also tells you to put
`export GALLIUM_DRIVER=d3d12` in `~/.bashrc`, and you should, because without
it every shell that starts Gazebo gets software rendering.

## Step 2: PX4

```bash
git clone -b v1.17.0 --recursive https://github.com/PX4/PX4-Autopilot.git ~/PX4-Autopilot
cd ~/PX4-Autopilot && bash Tools/setup/ubuntu.sh
```

`--recursive` matters: the `walls` world used below lives in the
`Tools/simulation/gz` submodule, and a plain clone will not have it. On arm64
add `--no-nuttx` to the setup script, because `gcc-multilib` does not exist
there. Measured: clone 3 minutes for 2.8 GB, setup script 9.5 minutes.

In a VMware virtual machine, one more line before logging out. PX4's setup
script detects VMware and appends `export SVGA_VGPU10=0` to `~/.profile`, a
fix for the old Gazebo Classic that today disables the OpenGL 3.3 core
profile Gazebo Harmonic's renderer needs. Left in place, Gazebo dies at the
first camera with `X Error of failed request: GLXBadFBConfig`. Measured on
VMware Workstation 17.6: failing every time with the line, clean every time
without it. Remove it:

```bash
sed -i '/SVGA_VGPU10/d' ~/.profile
```

The second VMware line comes at run time: start PX4 with
`PX4_GZ_SIM_RENDER_ENGINE=ogre`. Gazebo's default renderer on VMware's virtual
GPU produces a depth image with no finite points at all (0 of 76,800,
measured), which the obstacle node now reports as a dead camera; the older
`ogre` renderer gives correct depth, at about 2 Hz. That rate is the limit of
a VM here: at full stick the aircraft covers 5 m between frames, so expect
braking to be late or absent. Use the VM to install, fly and learn the
stack; take the avoidance numbers from a machine with a real GPU.

Now log out and back in. This is not ritual: the setup script added you to
the `dialout` group, and group changes take effect at login. On WSL,
`wsl --terminate Ubuntu-24.04` from PowerShell and reopen the terminal does
the same. Check with `id -nG`, which should list `dialout`.

Then build:

```bash
cd ~/PX4-Autopilot && make px4_sitl
```

Measured: 3 minutes on 4 cores, peak 4.4 GB of memory, which is the peak of
the whole installation. On less than 8 GB expect this step to be killed; if
it is, retry with `make px4_sitl -j2`.

Check: `ls ~/PX4-Autopilot/build/px4_sitl_default/bin/px4` exists.

## Step 3: this package

```bash
cd ~/px4-gazebo-avoidance
./scripts/install.sh
```

It prints what it will write and where, checks the platform, and stops at the
first failure with a message naming the thing that failed. It builds the
Micro XRCE-DDS Agent from source and installs it to `/usr/local`. It clones
and builds `px4_msgs`, 235 messages that exist only as source on every
platform. It links this package into a workspace at `~/av_ws` (override with
`WS=...`), installs the dependencies including Nav2 with `rosdep`, and builds.
Under 8 GB it limits the compilers to two jobs.

Measured: 18 minutes, peak 1.4 GB.

Check: the script's own final lines, which end with a renderer check and the
run commands. If it says `software rendering`, go back to the end of step 1.

## Step 4: the camera patch (optional, recommended)

```bash
cd ~/PX4-Autopilot/Tools/simulation/gz
git apply ~/px4-gazebo-avoidance/patches/px4-camera-res.patch
```

This drops the simulated depth camera from 640x480 at 30 Hz to 320x240 at
15 Hz. Every measurement in this repository was taken with it applied, and on
a slower machine it is the difference between a usable frame rate and not. It
applies inside the submodule, not at the PX4 root, and leaves it dirty.

## Run it

Two terminals. In each:

```bash
source /opt/ros/jazzy/setup.bash
source ~/av_ws/install/setup.bash
```

Terminal 1, PX4 and Gazebo:

```bash
cd ~/PX4-Autopilot && PX4_GZ_WORLD=walls HEADLESS=1 make px4_sitl gz_x500_depth
```

`HEADLESS=1` suppresses the Gazebo window and buys back 10 to 45% of
real-time factor; the server still renders the depth camera. Drop it to watch
in Gazebo as well as RViz. To attach a Gazebo window later without a restart, run `gz sim -g` in another
terminal; it joins the running server. Measured once: real-time factor 0.98
with a window attached that way. This terminal becomes PX4's own shell, `pxh>`.

Terminal 2, everything else:

```bash
ros2 launch avoidance_sim sim.launch.py
```

This starts the agent that carries PX4's topics into ROS 2, the bridge for the
Gazebo clock and depth cloud, the obstacle node, the six RViz-side nodes and
RViz itself. It also sets four PX4 parameters as soon as PX4 answers, and logs
each one as `[px4_params]`:

| Parameter | Why |
|---|---|
| `NAV_DLL_ACT=0` | The x500 airframe defaults to waiting for a ground station before arming. There is none here |
| `NAV_RCL_ACT=0` | The RC-loss failsafe. The pilot's synthetic sticks are the RC link |
| `CP_DIST=2.0` | The collision-prevention standoff in metres. Off until set |
| `CP_GO_NO_DATA=1` | Let PX4 move into directions the camera cannot see. The camera sees 73 of 360 degrees |

[how-it-works.md](how-it-works.md) says why each is what it is, and why they
are set this way rather than typed at `pxh>`. If PX4 lives somewhere other
than `~/PX4-Autopilot`, pass `px4_bin:=/path/to/build/px4_sitl_default/bin`.

### Is it working?

Within about 30 seconds of terminal 2 starting, terminal 1 prints
`Ready for takeoff!`. Before the parameters are set it repeats
`Preflight Fail: No connection to the GCS`, which is expected; if it keeps
repeating that afterwards, the launch could not reach `px4-param` and said so.

In a third terminal, sourced the same way:

```bash
ros2 topic hz /fmu/out/vehicle_local_position_v1    # about 50 Hz
ros2 topic hz /depth_camera/points                  # 6 to 12 Hz
export PATH="$HOME/PX4-Autopilot/build/px4_sitl_default/bin:$PATH"
px4-listener obstacle_distance                      # the histogram PX4 is receiving
```

The last one is the single most useful check: if it prints nothing,
perception is not reaching PX4 and nothing downstream can work. Terminal 2
also logs `camera sees 15 of 72 bins` from the obstacle node at startup.

RViz is up with the aircraft, the walls and a green goal ball. Go to
[fly.md](fly.md).

## Platforms

| Platform | Status |
|---|---|
| Windows 11, WSL2, Ubuntu 24.04 | Tested: the development machine and the clean-clone test. Needs `GALLIUM_DRIVER=d3d12` |
| Ubuntu 24.04 on real hardware | Untested. Nothing here is WSL-specific; set no renderer variable, Mesa picks the driver itself |
| Ubuntu 24.04 in VMware Workstation 17 on Windows | Tested for install and run, 6 cores and 8 GB: every step passes with the two VMware lines below, the aircraft flies and holds headings. Avoidance not validated: the virtual GPU renders the depth camera at 2 Hz. See below |
| Raspberry Pi 5, Ubuntu 24.04 arm64 | Untested. See below |
| Raspberry Pi OS | No. No ROS 2 Jazzy packages exist for Debian |
| macOS, Apple Silicon or Intel | Untested. Through an Ubuntu 24.04 arm64 VM with GPU acceleration; see below |

On a Pi 5 the known obstacle is the renderer. The Pi's Mesa V3D driver is
reported to cap desktop OpenGL at 3.1, and Gazebo's default `ogre2` backend
wants 3.3. The older `ogre` backend needs a real display: measured here on
x86, it aborts with `Unable to open display` when no X server is present. So a
Pi probably needs a monitor attached and this in terminal 1:

```bash
PX4_GZ_SIM_RENDER_ENGINE=ogre PX4_GZ_WORLD=walls make px4_sitl gz_x500_depth
```

Expect the builds to take far longer than the figures above and to risk the
out-of-memory killer. Nobody has confirmed any of this on a Pi yet; if you do,
the numbers belong in this file.

### macOS

Nothing here runs natively on macOS. ROS 2 Jazzy lists macOS as Tier 3,
source build only, on both Intel and Apple Silicon (REP 2000), and this stack
needs Nav2, Gazebo and PX4's bridge on top of that. The route is a virtual
machine running Ubuntu 24.04 arm64, which is a Tier 1 ROS 2 platform with
binary packages, and for which Gazebo Harmonic's apt repository also carries
arm64 builds (its noble Release file lists `amd64 arm64 armhf`). Untested
here: the first person to do it should run `scripts/report.sh` afterwards and
send the output, and the numbers belong in this file.

Give the VM at least 6 cores, 8 GB of memory and 30 GB of disk, and switch on
its GPU acceleration. Four cores was marginal for flight in the clean-clone
test, and 8 GB for the VM means a 16 GB Mac. UTM is free and its Ubuntu guide
installs the "Ubuntu Server for ARM" image and then `sudo apt install
ubuntu-desktop`; Parallels and VMware Fusion are alternatives. None of the
three has been tried with this stack.

Before step 1, check the one thing that decides whether the hour of builds is
worth it:

```bash
sudo apt install -y mesa-utils && glxinfo -B | grep renderer
```

If it says `llvmpipe`, the VM has no GPU acceleration and the simulation
would run at a thirtieth of real time; fix the VM's display settings first.
Then follow steps 1 to 4 as written, with two arm64 differences: step 2's
setup script needs `--no-nuttx`, and `GALLIUM_DRIVER` must not be set, since
that variable is WSL's and nothing else's. If the hypervisor is VMware Fusion,
step 2's `SVGA_VGPU10` note applies: PX4's setup script writes that line on
any VMware guest.

## When it goes wrong

First, `scripts/report.sh`. It prints what the machine is, what is installed,
the renderer, and the state of the running stack, with no sudo and no changes.
Paste its output when asking for help.

| Symptom | Cause |
|---|---|
| `Package 'avoidance_sim' not found` | The workspace is not sourced in this terminal |
| Nothing on any `/fmu/out/` topic | The agent is not running or died; the launch starts it, so look at terminal 2 |
| `Preflight Fail: No connection to the GCS`, repeating | `NAV_DLL_ACT` was not set; check terminal 2 for the `[px4_params]` lines |
| Real-time factor near 0.03 | Software rendering; `glxinfo -B` says `llvmpipe`. On WSL, `GALLIUM_DRIVER=d3d12` |
| Real-time factor around 0.5 | The Gazebo window is open; use `HEADLESS=1` |
| Aircraft only flies forwards | `CP_GO_NO_DATA` is 0 |
| Aircraft will not arm | A goal is holding the throttle up: `STOP` on the green ball first. Or PX4 is still booting |
| `X Error of failed request: GLXBadFBConfig` as the model spawns | `SVGA_VGPU10=0` in the environment, which PX4's setup script writes to `~/.profile` on VMware. Remove it; step 2 says how |
| `depth cloud is all NaN` in terminal 2, aircraft flies at walls | The renderer is producing no depth. On VMware, start PX4 with `PX4_GZ_SIM_RENDER_ENGINE=ogre` |

The simulation degrades after an hour or two of flying, crashes and restarts:
the aircraft drifts, ignores commands, or reports a position far from the
world. Restart PX4 and Gazebo cleanly before anything else:

```bash
pkill -f "make px4_sitl"; pkill -f "bin/px4"; pkill -f "gz sim"
sleep 3; pkill -9 -f "gz sim"; pkill -9 -f "bin/px4"
pgrep -cf "gz sim|bin/px4"        # must print 0, or the old world survives
```

Then run terminal 1 again. The parameters are saved, so they survive.
