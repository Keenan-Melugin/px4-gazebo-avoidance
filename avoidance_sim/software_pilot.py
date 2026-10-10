"""Flies to a goal by streaming synthetic manual control, which is what keeps collision prevention active.

Split out of a single-file prototype. The behaviour here is measured, not
assumed; see the repository README for the numbers and the traps.
"""


import math

from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from px4_msgs.msg import (ManualControlSetpoint,
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus)

from .frames import MODE_QOS, PX4_QOS, YAW_SUFFIX, wrap, yaw_of


class SoftwarePilot(Node):
    """Fly to a goal by driving the sticks, so PX4 collision prevention applies.

    THE WHOLE POINT. PX4's CollisionPrevention is a member of
    StickAccelerationXY (StickAccelerationXY.hpp:73) and its only non-test call
    site is that mapper. So the stick path is the ONLY interface to it that
    exists: every auto mode (reposition, mission, offboard) is structurally
    unprotected. Instead of commanding a position and losing avoidance, this
    node runs the position controller itself and emits stick values, leaving
    PX4 in Position mode where CP is live.

    PX4 cannot tell this from QGroundControl's virtual joystick: the MAVLink
    joystick path publishes to ORB_ID(manual_control_input)
    (mavlink_receiver.h:343), the same topic /fmu/in/manual_control_input
    bridges to. Source is checked once, in ManualControlSelector via
    COM_RC_IN_MODE, and nowhere downstream.

    Why this beats a one-shot reposition even ignoring CP: CP_GUIDE_ANG allows
    30 degrees of deflection either side of the commanded direction, so a
    controller that continuously re-aims gets genuine go-around rather than
    just braking.

    Scope: CP is horizontal only. modifySetpoint takes a Vector2f and
    the library contains no Vector3f at all, so the climb and descent of a 3D
    goal are NOT protected. This is 3D waypoints with 2D avoidance.
    """

    RATE = 0.02        # 50 Hz. COM_RC_LOSS_T is 0.5 s, so never below 10 Hz.
    KP_XY = 0.22       # stick per metre of error
    KD_XY = 0.25       # stick per m/s. Measured need: P-only overshot a 5 m
                       # goal by ~3 m, because releasing to zero stick only
                       # brakes on the drag model and it coasts.
    KP_Z = 0.30
    KD_Z = 0.20
    # Yaw. The stick commands a RATE, not an angle, so this is P on heading
    # error. The sign was measured: positive stick turns clockwise, confirmed
    # symmetrically (+0.5 gave +24.3 deg/s and -0.5 gave -24.2 deg/s, each
    # averaged over a 3 s hold that includes spin-up, so those two figures
    # understate the steady rate the sweep below measures).
    #
    # The catch, measured by sweeping the stick on this airframe:
    #     0.05  0.08  0.10 | 0.12  0.15  0.20  0.30  0.50
    #     0.0   0.0   0.0  | 1.1   2.9   5.7   12.4  28.8   deg/s
    # There is a dead band up to about 0.10, and above it the response fits
    # rate = (stick - 0.10) * 72 deg/s closely. Plain P therefore CANNOT hold
    # a heading: as the error shrinks the command fades into the dead band and
    # the aircraft stalls short. It froze 14.3 degrees off every time, because
    # 14.3 deg of error times the old gain landed exactly on 0.10.
    # So the command steps over the dead band instead of fading into it.
    # The model above predicts 28.8 deg/s at stick 0.50 against 28.8
    # measured. Inverting that model
    # is what lets the command be expressed as a rate.
    # 0.105 is interpolated, not measured: the sweep jumps from 0.10 (no
    # rotation) to 0.12 (1.1 deg/s), so the edge is only known to lie in
    # (0.10, 0.12]. Closed-loop the smallest command this produces is 0.135,
    # which is clear of it either way.
    YAW_DZ = 0.105                      # just inside the dead band
    YAW_RATE_PER_STICK = math.radians(72.0)   # per unit of stick above YAW_DZ
    # Command a rate proportional to error, capped. The cap is the important
    # number: unlimited P saturated near 65 deg/s and then overshot ~25 deg,
    # because the heading feedback lags and the airframe coasts.
    KP_YAW = 1.2                        # 1/s, error rad -> target rad/s
    KD_YAW = 0.45                       # brakes the approach
    YAW_RATE_MAX = math.radians(20.0)
    YAW_TOL = math.radians(3.0)         # inside this, stop turning
    ARRIVED_YAW = math.radians(8.0)
    YAW_GIVE_UP_TICKS = 1500            # 30 s at 50 Hz

    # Velocity mode, used when Nav2 is driving. The XY stick has the same kind
    # of dead band as yaw, measured by sweeping pitch and reading the steady
    # ground speed back:
    #     stick  0.02  0.04  0.06  0.08  0.10 | 0.15  0.25  0.40
    #     m/s    0.00  0.00  0.00  0.00  0.00 | 0.20  0.64  1.45
    # Least squares over the moving points: speed = (stick - 0.114) * 5.02.
    # So full stick is about 4.4 m/s even though MPC_VEL_MANUAL says 10.0;
    # scaling off that parameter would under-command by more than double.
    # The slope is fitted over 0.15 to 0.40, so treat it as valid for the
    # gentle velocities Nav2 asks for, not as a top-speed figure.
    XY_DZ = 0.114                       # stick that produces no motion
    VEL_PER_STICK = 5.02                # m/s per unit of stick above XY_DZ
    CMD_VEL_TIMEOUT = 0.5               # s. Matches COM_RC_LOSS_T.
    # Heading follows the direction of travel while Nav2 drives, so the
    # camera looks along the path rather than wherever the nose was left.
    ALIGN_MIN_SPEED = 0.25              # m/s. Below this, do not chase noise.
    KP_ALIGN = 1.0                      # rad/s per rad of misalignment

    # Two ways to fly, chosen at runtime on /avoidance_sim/mode, plus
    # 'external' (below), which only hands the stick stream to a test script.
    #
    #   brake  Position mode, synthetic sticks. PX4 collision prevention is
    #          live and is the entire avoidance mechanism.
    #   plan   Offboard mode, TrajectorySetpoint velocity. PX4 holds
    #          CollisionPrevention only in its manual Position-mode flight
    #          tasks, so Offboard has none and the planner owns avoidance.
    #
    # They are exclusive because collision prevention vetoes a planner.
    # Measured: with CP at 1.0 m the planner commanded 1.50 m/s and the
    # aircraft achieved 0.00, deadlocked; with CP off it achieved 1.24 and
    # rounded the wall. Switching mode rather than switching CP_DIST means
    # nothing has to be reconfigured, because CP simply does not apply in
    # Offboard.
    MODE_BRAKE = 'brake'
    MODE_PLAN = 'plan'
    # For the stick-sweep measurements in test/: the pilot stops publishing
    # sticks so a script can own the stream. Two publishers on the one topic
    # made the sweeps measure nothing at all (2026-10-10). The script must
    # stream at 50 Hz itself, or PX4 declares RC loss after 0.5 s.
    MODE_EXTERNAL = 'external'
    # PX4 drops Offboard if the setpoint stream stops for COM_OF_LOSS_T,
    # which is 1.0 s by default, so plan mode keeps streaming zeros when Nav2
    # goes quiet rather than going silent and tripping the failsafe.
    OFFBOARD_WARMUP = 1.2               # s of setpoints before asking for the mode
    DEADBAND = 0.4     # m
    ARRIVED = 0.6      # m

    def __init__(self):
        super().__init__('software_pilot',
                         start_parameter_services=False)  # see rviz_bridge.py
        self.pub = self.create_publisher(
            ManualControlSetpoint, '/fmu/in/manual_control_input', PX4_QOS)
        self.create_subscription(VehicleLocalPosition,
                                 '/fmu/out/vehicle_local_position_v1',
                                 self.on_local, PX4_QOS)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1',
                                 self.on_status, PX4_QOS)
        self.create_subscription(PoseStamped, '/avoidance_sim/pilot_goal',
                                 self.on_goal, 10)
        # Nav2's output. Fresh messages here take precedence over a goal:
        # Nav2 is already doing the position control, so the pilot drops to
        # being a velocity-to-stick converter plus an altitude hold.
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)
        self.cmd = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', PX4_QOS)

        # Sign conventions are PARAMETERS, not assumptions. Verified empirically
        # like the point-cloud axis convention was; do not trust the message
        # comment alone.
        self.declare_parameter('pitch_sign', 1.0)
        self.declare_parameter('roll_sign', 1.0)
        self.declare_parameter('yaw_sign', 1.0)
        self.pitch_sign = float(self.get_parameter('pitch_sign').value)
        self.roll_sign = float(self.get_parameter('roll_sign').value)
        self.yaw_sign = float(self.get_parameter('yaw_sign').value)

        self.pos = None          # (north, east, down)
        self.vel = (0., 0., 0.)  # (vn, ve, vd)
        self.yaw = 0.0
        self.nav = None
        self.goal = None         # (north, east, alt)
        self.goal_yaw = None     # NED heading, radians. None = do not turn.
        self.mode = self.MODE_BRAKE
        self._last_posctl_req = 0.0
        self.offboard_since = None
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', PX4_QOS)
        self.traj_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', PX4_QOS)
        self.create_subscription(String, '/avoidance_sim/mode',
                                 self.on_mode, MODE_QOS)
        self.cmd_vel = None      # (vx, vy, wz) in base_link FLU
        self.cmd_vel_t = 0.0
        self.hold_alt = None     # altitude to keep while Nav2 drives XY
        self.vel_mode = False
        self.yaw_rate = 0.0      # differentiated heading, for damping
        self._yaw_ticks = 0      # how long we have been chasing a heading
        self._last_yaw = None
        self._last_yaw_t = None
        self.active = False
        self.create_timer(self.RATE, self.tick)
        self.create_timer(2.0, self.report)
        self.get_logger().info(
            'software pilot ready. Publishes sticks continuously so PX4 stays '
            'in Position mode and collision prevention applies.')

    def on_local(self, m):
        if math.isfinite(m.x):
            self.pos = (m.x, m.y, m.z)
            # All three components, not just vx. A non-finite vz used to
            # reach the throttle clamp, and max(-1, min(1, nan)) is 1.0 in
            # Python rather than nan, so it commanded FULL climb.
            self.vel = ((m.vx, m.vy, m.vz)
                        if (math.isfinite(m.vx) and math.isfinite(m.vy)
                            and math.isfinite(m.vz))
                        else (0., 0., 0.))
            # Yaw rate by differentiation. VehicleLocalPosition carries no
            # yawspeed, and pulling in VehicleAngularVelocity just for this
            # would add a subscription for a number we can difference. Lightly
            # filtered because the step size is uneven.
            now = self.get_clock().now().nanoseconds * 1e-9
            if self._last_yaw is not None and self._last_yaw_t is not None:
                dt = now - self._last_yaw_t
                if 1e-3 < dt < 0.5:
                    raw = wrap(m.heading - self._last_yaw) / dt
                    # A single non-finite sample would otherwise latch the
                    # filter at NaN for the life of the process, because the
                    # update subtracts its own current value.
                    if math.isfinite(raw):
                        self.yaw_rate += 0.5 * (raw - self.yaw_rate)
            self._last_yaw, self._last_yaw_t = m.heading, now
            self.yaw = m.heading

    def on_status(self, m):
        self.nav = m.nav_state

    def on_goal(self, msg: PoseStamped):
        """Goal arrives in ENU (RViz): x east, y north, z up."""
        if self.mode == self.MODE_PLAN:
            # A stick goal has only one meaning: fly it on sticks. Accepting
            # it in plan mode and then ignoring it left PX4 armed in Offboard
            # on the ground with nothing driving, and no message saying why.
            self.get_logger().info(
                'pilot goal received in PLAN mode: switching to BRAKE, '
                'because a pilot goal means fly it on sticks')
            self.on_mode(String(data=self.MODE_BRAKE))
        if msg.header.frame_id == "STOP":
            self.stop()
            return
        self.goal = (msg.pose.position.y, msg.pose.position.x,
                     msg.pose.position.z)

        # Heading is opt-in, flagged on the frame id. Testing the
        # quaternion instead does not work: identity in ENU is yaw 0, which
        # is due east, so "face east" would be indistinguishable from "no
        # preference". A sender that wants the heading held says so.
        if msg.header.frame_id.endswith(YAW_SUFFIX):
            # RViz is ENU (0 = east, anticlockwise). PX4 heading is NED
            # (0 = north, clockwise). They differ by a reflection about
            # 45 degrees, not merely an offset.
            self.goal_yaw = wrap(math.pi / 2.0 - yaw_of(msg.pose.orientation))
        else:
            self.goal_yaw = None
        self.active = True
        self._yaw_ticks = 0
        hdg = ('none' if self.goal_yaw is None
               else '%+.0f deg' % math.degrees(self.goal_yaw))
        self.get_logger().info(
            f'goal: north {self.goal[0]:+.2f} east {self.goal[1]:+.2f} '
            f'alt {self.goal[2]:.2f} heading {hdg} -- on sticks, CP active')
        # Position mode is enterable because we are already streaming sticks.
        self.request_posctl()

    def on_mode(self, m):
        want = (m.data or '').strip().lower()
        if want not in (self.MODE_BRAKE, self.MODE_PLAN, self.MODE_EXTERNAL):
            self.get_logger().warn(
                f'ignoring unknown mode {want!r}; use '
                f'{self.MODE_BRAKE!r}, {self.MODE_PLAN!r} or {self.MODE_EXTERNAL!r}')
            return
        if want == self.mode:
            return
        self.mode = want
        if want == self.MODE_EXTERNAL:
            self.active = False
            self.goal = None
            self.get_logger().info(
                'EXTERNAL mode: not publishing sticks; another program owns '
                'the stick stream. Send brake to take it back.')
            return
        if want == self.MODE_PLAN:
            self.active = False
            self.goal = None
            self.vel_mode = False
            self.offboard_since = self.now_s()
            self.hold_alt = -self.pos[2] if self.pos else None
            self.get_logger().info(
                'PLAN mode: Offboard, velocity setpoints, planner owns '
                'avoidance. PX4 collision prevention does not apply here.')
        else:
            self.offboard_since = None
            self.get_logger().info(
                'BRAKE mode: Position mode on sticks, PX4 collision '
                'prevention live.')
            self.request_posctl()

    def request_offboard(self):
        c = VehicleCommand()
        c.timestamp = self.get_clock().now().nanoseconds // 1000
        c.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        c.param1 = 1.0
        c.param2 = 6.0     # PX4_CUSTOM_MAIN_MODE_OFFBOARD
        c.target_system = 1
        c.target_component = 1
        c.source_system = 1
        c.source_component = 1
        c.from_external = True
        self.cmd.publish(c)

    def tick_plan(self):
        """Offboard velocity setpoints from Nav2, in NED.

        TrajectorySetpoint.velocity is the NED world frame; cmd_vel is
        base_link FLU. Rotating one into the other is the kind of conversion
        that produces believable numbers when it is wrong, so it is measured
        rather than trusted: see test/ned_check.py, which commands each
        direction and reads back the velocity PX4 reports.
        """
        # The stream has to exist before PX4 will accept the mode, and has to
        # keep existing or PX4 drops out of it.
        oc = OffboardControlMode()
        oc.timestamp = self.get_clock().now().nanoseconds // 1000
        oc.position = False
        oc.velocity = True
        oc.acceleration = False
        oc.attitude = False
        oc.body_rate = False
        self.offboard_pub.publish(oc)

        # Keep the stick stream alive. This is not optional.
        #
        # The first version stopped publishing ManualControlSetpoint the
        # moment plan mode started. PX4 treats that stream as the RC link
        # (COM_RC_IN_MODE 1, joystick only), and COM_RC_LOSS_T is 0.5 s, so it
        # declared RC loss BEFORE the 1.2 s Offboard warm-up had finished and
        # fired the RC-loss failsafe, NAV_RCL_ACT. That is where the unexplained
        # heading swings in plan mode came from: the aircraft was obeying a
        # failsafe, not the planner.
        #
        # Zero sticks are harmless in Offboard. The flight task ignores them
        # for control, and COM_RC_OVERRIDE only ejects from Offboard on stick
        # MOVEMENT, which a constant zero is not.
        m = ManualControlSetpoint()
        m.timestamp = self.get_clock().now().nanoseconds // 1000
        m.timestamp_sample = m.timestamp
        m.valid = True
        m.data_source = ManualControlSetpoint.SOURCE_MAVLINK_0
        m.roll = m.pitch = m.yaw = m.throttle = 0.0
        self.pub.publish(m)

        fresh = (self.cmd_vel is not None
                 and (self.now_s() - self.cmd_vel_t) < self.CMD_VEL_TIMEOUT)
        vx, vy, wz = self.cmd_vel if fresh else (0.0, 0.0, 0.0)

        t = TrajectorySetpoint()
        t.timestamp = self.get_clock().now().nanoseconds // 1000
        t.position = [float('nan')] * 3
        t.acceleration = [float('nan')] * 3

        # body FLU -> body FRD -> NED, using the current heading.
        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        vn = vx * cy + vy * sy
        ve = vx * sy - vy * cy

        # Nav2 knows nothing about altitude. vd is DOWN positive, so climbing
        # needs a negative value.
        vd = 0.0
        if self.pos is not None and self.hold_alt is not None:
            ez = self.hold_alt - (-self.pos[2])
            u = self.KP_Z * ez - self.KD_Z * (-self.vel[2])
            if math.isfinite(u):
                vd = float(-max(-2.0, min(2.0, u)))
        t.velocity = [float(vn), float(ve), vd]

        # Yaw comes from the planner, and ONLY from the planner.
        #
        # The first version also slaved the heading to the direction of
        # travel here, as brake mode does, and that is unstable in this
        # branch: cmd_vel is a BODY frame velocity, so it is rotated into NED
        # using the current heading, and then setting the yaw target to the
        # direction of that NED vector feeds the heading back into its own
        # input. The result is runaway rotation. Measured: headings swinging
        # through 120 degrees while the aircraft translated under a metre in
        # eleven seconds, for every commanded direction.
        #
        # Brake mode can slave the heading because nothing else is steering
        # there. Here the pure-pursuit controller already emits angular.z to
        # turn toward its path, so the planner owns the heading and the pilot
        # must not also have an opinion about it.
        t.yaw = float('nan')
        t.yawspeed = float(-wz) if abs(wz) > 0.02 else 0.0
        self.traj_pub.publish(t)

        if (self.offboard_since is not None
                and self.now_s() - self.offboard_since > self.OFFBOARD_WARMUP):
            if self.nav != 14:          # NAVIGATION_STATE_OFFBOARD
                self.request_offboard()

    def on_cmd_vel(self, m):
        self.cmd_vel = (m.linear.x, m.linear.y, m.angular.z)
        self.cmd_vel_t = self.now_s()

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def stick_for(self, v):
        """Velocity to stick, stepping over the measured dead band.

        Below a hair of a m/s, command nothing: offsetting past the dead band
        for a near-zero velocity would make the aircraft creep.
        """
        if abs(v) < 0.03:
            return 0.0
        u = self.XY_DZ + abs(v) / self.VEL_PER_STICK
        return math.copysign(min(1.0, u), v)

    def request_posctl(self):
        c = VehicleCommand()
        c.timestamp = self.get_clock().now().nanoseconds // 1000
        c.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        c.param1 = 1.0     # custom mode enabled
        c.param2 = 3.0     # PX4_CUSTOM_MAIN_MODE_POSCTL
        c.target_system = 1
        c.target_component = 1
        c.source_system = 1
        c.source_component = 1
        c.from_external = True
        self.cmd.publish(c)

    def stop(self):
        self.active = False
        self.goal = None
        self.get_logger().info('pilot stopped, holding position')

    def tick(self):
        if self.mode == self.MODE_EXTERNAL:
            return
        if self.mode == self.MODE_PLAN:
            self.tick_plan()
            return

        m = ManualControlSetpoint()
        m.timestamp = self.get_clock().now().nanoseconds // 1000
        m.timestamp_sample = m.timestamp
        m.valid = True
        # Required: the selector drops a sample whose source does not match
        # COM_RC_IN_MODE. The DDS bridge does not fill this in for you.
        m.data_source = ManualControlSetpoint.SOURCE_MAVLINK_0
        m.roll = m.pitch = m.yaw = m.throttle = 0.0

        fresh = (self.cmd_vel is not None
                 and (self.now_s() - self.cmd_vel_t) < self.CMD_VEL_TIMEOUT)

        if fresh and self.pos is not None:
            # ---- Nav2 is driving ----
            if not self.vel_mode:
                self.vel_mode = True
                self.hold_alt = -self.pos[2]
                self.active = False          # a stale goal must not fight it
                self.goal = None
                self.get_logger().info(
                    f'velocity mode: Nav2 driving, holding '
                    f'{self.hold_alt:.1f} m. CP still active.')
                self.request_posctl()
            vx, vy, wz = self.cmd_vel
            # pitch is forward, roll is right. base_link FLU y is LEFT, so a
            # positive vy means go left, which is negative roll.
            m.pitch = float(self.pitch_sign * self.stick_for(vx))
            m.roll = float(self.roll_sign * self.stick_for(-vy))
            # Point the camera where we are going.
            #
            # This is not cosmetic. The depth camera sees a 73 degree forward
            # arc and nothing else, so a 2D planner working off that costmap
            # only knows about the slice the nose happens to be facing. Flying
            # sideways means planning blind: each replan sees a different
            # slice, the planner flip-flops between routes, and the controller
            # dithers. Measured without this: the commanded sideways velocity
            # alternated sign every few seconds and the aircraft never
            # committed to a detour.
            #
            # So the heading is slaved to the direction of travel. Nav2's own
            # yaw rate is used instead whenever it asks for one.
            speed = math.hypot(vx, vy)
            if abs(wz) > 0.02:
                want_rate = -wz           # FLU anticlockwise -> clockwise stick
            elif speed > self.ALIGN_MIN_SPEED:
                # atan2(left, forward): positive means the velocity points
                # left of the nose, so the nose must turn anticlockwise.
                off = math.atan2(vy, vx)
                want_rate = -self.KP_ALIGN * off
            else:
                want_rate = 0.0
            want_rate = max(-self.YAW_RATE_MAX,
                            min(self.YAW_RATE_MAX, want_rate))
            if abs(want_rate) > 0.02:
                u = self.YAW_DZ + abs(want_rate) / self.YAW_RATE_PER_STICK
                m.yaw = float(self.yaw_sign * math.copysign(min(1.0, u),
                                                            want_rate))
            # Nav2 knows nothing about altitude, so the pilot keeps it.
            n, e, d = self.pos
            vn, ve, vd = self.vel
            ez = self.hold_alt - (-d)
            u_z = self.KP_Z * ez - self.KD_Z * (-vd)
            if math.isfinite(u_z):
                m.throttle = float(max(-1.0, min(1.0, u_z)))

        elif self.active and self.goal is not None and self.pos is not None:
            if self.vel_mode:
                self.vel_mode = False
                self.get_logger().info('velocity mode off, back to goals')
            gn, ge, galt = self.goal
            n, e, d = self.pos
            en, ee = gn - n, ge - e
            ez = galt - (-d)

            # Stick XY is in the HEADING frame (Sticks::rotateIntoHeadingFrameXY),
            # not NED. Rotate the error by the current heading or the aircraft
            # flies off at an angle that changes as it yaws.
            cy, sy = math.cos(self.yaw), math.sin(self.yaw)
            fwd = en * cy + ee * sy
            right = -en * sy + ee * cy
            vn, ve, vd = self.vel
            v_fwd = vn * cy + ve * sy
            v_right = -vn * sy + ve * cy

            # PD, not P. The D term is what stops the overshoot: zero stick
            # only decelerates through the drag model, so without damping it
            # sails past the goal and brakes afterwards.
            u_fwd = self.KP_XY * fwd - self.KD_XY * v_fwd
            u_right = self.KP_XY * right - self.KD_XY * v_right
            horiz = math.hypot(fwd, right)
            mag = math.hypot(u_fwd, u_right)
            if horiz > self.DEADBAND and mag > 1e-6:
                scale = min(1.0, mag) / mag
                m.pitch = float(self.pitch_sign * u_fwd * scale)
                m.roll = float(self.roll_sign * u_right * scale)
            if abs(ez) > self.DEADBAND:
                u_z = self.KP_Z * ez - self.KD_Z * (-vd)
                # Explicit, because the clamp alone does not reject NaN.
                if math.isfinite(u_z):
                    m.throttle = float(max(-1.0, min(1.0, u_z)))

            # Yaw is independent of translation: the XY error is rotated into
            # the heading frame every tick, so turning mid-flight does not
            # bend the path.
            eyaw = 0.0
            if self.goal_yaw is not None:
                eyaw = wrap(self.goal_yaw - self.yaw)
                if abs(eyaw) > self.YAW_TOL:
                    # Wanted rate, capped, damped by the rate we already have.
                    want = self.KP_YAW * eyaw - self.KD_YAW * self.yaw_rate
                    # Clamp AFTER rejecting non-finite values. Clamping a NaN
                    # returns the positive cap, so the aircraft would turn
                    # clockwise at full rate no matter which way was shorter.
                    if not math.isfinite(want):
                        want = 0.0
                    want = max(-self.YAW_RATE_MAX,
                               min(self.YAW_RATE_MAX, want))
                    # Invert the measured model: step over the dead band, then
                    # scale. Without the dead band term the command fades into
                    # the dead spot and the aircraft stalls short.
                    u = self.YAW_DZ + abs(want) / self.YAW_RATE_PER_STICK
                    u = min(1.0, u)
                    m.yaw = float(self.yaw_sign * math.copysign(u, want))

            # Bounded time. With an inverted yaw_sign the heading loop parks
            # 179 degrees off and limit-cycles there indefinitely, streaming
            # sticks with nothing to stop it. Give up on the heading rather
            # than fly forever.
            if self.goal_yaw is not None:
                self._yaw_ticks += 1
                if self._yaw_ticks > self.YAW_GIVE_UP_TICKS:
                    self.get_logger().warn(
                        'could not reach the commanded heading in %.0f s '
                        '(%.0f deg off). Holding position and giving up on '
                        'yaw; check yaw_sign.'
                        % (self.YAW_GIVE_UP_TICKS * self.RATE,
                           math.degrees(abs(eyaw))))
                    self.goal_yaw = None

            yaw_ok = self.goal_yaw is None or abs(eyaw) <= self.ARRIVED_YAW
            if horiz <= self.ARRIVED and abs(ez) <= self.ARRIVED and yaw_ok:
                extra = ('' if self.goal_yaw is None
                         else ', %.0f deg heading' % math.degrees(abs(eyaw)))
                self.get_logger().info(
                    f'arrived: {horiz:.2f} m horizontal, '
                    f'{abs(ez):.2f} m vertical{extra}')
                self.active = False

        self.pub.publish(m)

        # Keep asking for Position mode until PX4 is in it. One request on
        # the mode switch was not enough: if PX4 was still in Offboard and
        # declined, brake mode sat there streaming sticks that Offboard
        # ignores, forever. Plan mode already re-asks for Offboard the same way.
        if self.nav is not None and self.nav != 2:
            now = self.now_s()
            if now - self._last_posctl_req > 2.0:
                self._last_posctl_req = now
                self.request_posctl()

    def report(self):
        if self.mode == self.MODE_PLAN:
            if self.pos is None:
                return
            fresh = (self.cmd_vel is not None
                     and (self.now_s() - self.cmd_vel_t) < self.CMD_VEL_TIMEOUT)
            vx, vy, _ = self.cmd_vel if fresh else (0.0, 0.0, 0.0)
            self.get_logger().info(
                f'PLAN nav={self.nav}{"" if self.nav == 14 else " (not OFFBOARD yet)"} '
                f'cmd fwd{vx:+.2f} left{vy:+.2f} '
                f'alt{-self.pos[2]:.1f}/{self.hold_alt or 0:.1f} '
                f'{"[stale, holding]" if not fresh else ""}')
            return
        if self.vel_mode and self.pos is not None:
            vx, vy, wz = self.cmd_vel or (0.0, 0.0, 0.0)
            self.get_logger().info(
                f'nav={self.nav} Nav2 vel fwd{vx:+.2f} left{vy:+.2f} '
                f'yaw{wz:+.2f} alt{-self.pos[2]:.1f}/{self.hold_alt:.1f} '
                f'(CP {"ACTIVE" if self.nav == 2 else "INACTIVE"})')
            return
        if not self.active or self.goal is None or self.pos is None:
            return
        gn, ge, galt = self.goal
        n, e, d = self.pos
        turn = ('' if self.goal_yaw is None else ' turn %+.0f deg'
                % math.degrees(wrap(self.goal_yaw - self.yaw)))
        self.get_logger().info(
            f'nav={self.nav} pos n{n:+.1f} e{e:+.1f} alt{-d:.1f} '
            f'hdg{math.degrees(self.yaw):+.0f} -> '
            f'to go {math.hypot(gn - n, ge - e):.2f} m{turn} '
            f'(CP {"ACTIVE" if self.nav == 2 else "INACTIVE, not POSCTL"})')
