"""Frame conventions, shared QoS, and the small helpers every node needs.

The frame conversions here are the ones that fail *silently* if you get them
wrong, which is why each carries its derivation rather than just a number.

PX4 speaks NED for the world and FRD for the body. ROS speaks ENU and FLU.
Swapping the position axes alone looks correct in a plot and is still wrong:
the attitude needs a rotation applied on both sides, or the aircraft renders
upside down in RViz while its heading still reads correctly.
"""

import math

from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

# PX4's uXRCE-DDS publishers are BEST_EFFORT and VOLATILE. A subscriber that
# asks for RELIABLE or TRANSIENT_LOCAL will match nothing and simply never
# receive, with no error anywhere.
PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST, depth=5)

# The pilot's mode (brake, plan, or external for test scripts) is state, not an event. TRANSIENT_LOCAL means the last
# value is retained and handed to any subscriber that joins late, so a mode
# switch cannot be lost to discovery timing the way a one-shot VOLATILE
# message can. Publishers and subscribers must BOTH use this: a VOLATILE
# publisher does not match a TRANSIENT_LOCAL subscriber at all.
MODE_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL,
                      history=HistoryPolicy.KEEP_LAST, depth=1)

# Quaternions are (w, x, y, z) throughout this package, which is PX4's order.
# ROS messages want x, y, z, w; the one place that writes them reorders.
#
# NED -> ENU is a 180 degree turn about (1,1,0)/sqrt(2): x<->y, z flips.
# body FRD -> body FLU is 180 degrees about x.
#
# These were scipy Rotation objects. At 100 odometry messages a second the
# from_quat / multiply / as_quat / inv().apply chain was measurable in the
# bridge process, and it pulled scipy onto every machine, about 100 MB on a
# Pi, for two constant rotations. The plain products below were checked
# against scipy on 60 live samples before the switch: quaternions agree to
# 9e-8 and rotated velocities to 2e-8.
_S2 = 1.0 / math.sqrt(2.0)
Q_NED_ENU = (0.0, _S2, _S2, 0.0)
Q_FRD_FLU = (0.0, 1.0, 0.0, 0.0)

# Camera mount, from Tools/simulation/gz/models/x500_depth/model.sdf:
#   <pose>.12 .03 .242 0 0 0</pose>, no rotation. Gazebo link axes are FLU,
# which is what ROS base_link uses, so this is a pure translation.
CAM_XYZ = (0.12, 0.03, 0.242)
# The 2D lidar on models/x500_depth_lidar (model.sdf pose, FLU), for TF.
LIDAR_XYZ = (-0.10, 0.0, 0.30)

# A goal whose frame id carries this suffix wants its heading held too.
# Position-only senders leave it off and the aircraft keeps its current yaw.
#
# This is a flag rather than a test of the quaternion because the obvious test
# does not work: an identity orientation in ENU is yaw zero, which is due east,
# so "face east" would be indistinguishable from "no preference".
YAW_SUFFIX = '+yaw'


def wrap(a):
    """Shortest signed angle. Without this a 10 degree turn through north
    becomes a 350 degree one."""
    return (a + math.pi) % (2 * math.pi) - math.pi


def qmul(a, b):
    """Hamilton product of two (w, x, y, z) quaternions. As a rotation it is
    "b, then a", the same composition order scipy's `a * b` uses."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def ned_to_enu(q_body_ned):
    """PX4's body->NED attitude (w, x, y, z) as a ROS base_link->odom
    attitude (w, x, y, z): the NED->ENU turn on the outside, FRD->FLU on the
    inside. Both are needed; one alone renders the aircraft upside down."""
    return qmul(qmul(Q_NED_ENU, q_body_ned), Q_FRD_FLU)


def rotate_inv(q, v):
    """Rotate vector v by the inverse of the unit quaternion q (w, x, y, z).
    With q = body->world this takes a world-frame vector into the body."""
    w, x, y, z = q[0], -q[1], -q[2], -q[3]   # the inverse of a unit q is its conjugate
    vx, vy, vz = v
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


def yaw_of(q):
    """ENU yaw from a geometry_msgs quaternion. 0 = east, anticlockwise
    positive."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quat_onto(control, w, x, y, z):
    """Normalised quaternion onto a control. MOVE_AXIS translates along the
    control's LOCAL +x rotated by this, so a world-Z arrow needs 90 deg about
    Y, i.e. (1,0,1,0)."""
    n = 1.0 / math.sqrt(w * w + x * x + y * y + z * z)
    control.orientation.w = w * n
    control.orientation.x = x * n
    control.orientation.y = y * n
    control.orientation.z = z * n
