"""Flies to a goal by streaming synthetic manual control, which is what keeps collision prevention active.

Split out of a single-file prototype. The behaviour here is measured, not
assumed; see the repository README for the numbers and the traps.
"""


import math

from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import (ManualControlSetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus)

from .frames import PX4_QOS, YAW_SUFFIX, wrap, yaw_of


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

    Honest scope: CP is horizontal only. modifySetpoint takes a Vector2f and
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
    # Stick-to-rate model, measured by sweeping the stick:
    #     stick  0.05  0.08  0.10 | 0.12  0.15  0.20  0.30  0.50
    #     deg/s  0.0   0.0   0.0  | 1.1   2.9   5.7   12.4  28.8
    # Dead band to about 0.10, then rate = (stick - 0.10) * 72 deg/s, which
    # predicts 28.8 at stick 0.50 against 28.8 measured. Inverting that model
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
    DEADBAND = 0.4     # m
    ARRIVED = 0.6      # m

    def __init__(self):
        super().__init__('software_pilot')
        self.pub = self.create_publisher(
            ManualControlSetpoint, '/fmu/in/manual_control_input', PX4_QOS)
        self.create_subscription(VehicleLocalPosition,
                                 '/fmu/out/vehicle_local_position_v1',
                                 self.on_local, PX4_QOS)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1',
                                 self.on_status, PX4_QOS)
        self.create_subscription(PoseStamped, '/evtol/pilot_goal',
                                 self.on_goal, 10)
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
        m = ManualControlSetpoint()
        m.timestamp = self.get_clock().now().nanoseconds // 1000
        m.timestamp_sample = m.timestamp
        m.valid = True
        # Required: the selector drops a sample whose source does not match
        # COM_RC_IN_MODE. The DDS bridge does not fill this in for you.
        m.data_source = ManualControlSetpoint.SOURCE_MAVLINK_0
        m.roll = m.pitch = m.yaw = m.throttle = 0.0

        if self.active and self.goal is not None and self.pos is not None:
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

    def report(self):
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
