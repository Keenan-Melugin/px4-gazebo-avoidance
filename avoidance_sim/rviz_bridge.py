#!/usr/bin/env python3
"""Everything that makes the aircraft controllable from RViz, in one process.

Six nodes share one executor. They are one process rather than six because
they are cheap, they start and stop together, and a single process is one
thing for a reader to watch rather than six.

    TfPublisher     PX4 odometry -> the TF tree RViz needs
    WorldMarkers    the Gazebo world's obstacles, drawn in RViz
    GoalBridge      RViz's flat "2D Goal Pose" tool -> a PX4 reposition (brake)
                    or a Nav2 goal (plan)
    CommandMarker   the right-click menu above the aircraft
    Goal3D          the draggable 3D waypoint, with a ring for heading
    SoftwarePilot   flies to a goal on synthetic sticks

The pilot is the load-bearing one. PX4's collision prevention runs in Position
mode only, so the aircraft has to be flown on sticks for avoidance to apply at
all. The pilot streams synthetic manual control to do exactly that, which is
what lets a clicked goal and working avoidance coexist.

Which executor, and why it was measured. This process first ran on a
MultiThreadedExecutor and cost 102% of a core while the aircraft sat on the
ground: on a four-core machine, a quarter of the computer for a marker
publisher. Profiling showed the callbacks were small (the TF conversion, the
pilot's 50 Hz tick and all publishing came to about 3 s of a 60 s run) and
the rest was rclpy rebuilding its wait set on each of roughly 300 wake-ups a
second over about 75 waitables: every publisher, subscription, timer and
service in the process. The same bridge, fresh simulation, aircraft on the
ground, 8 s windows:

    MultiThreadedExecutor (one thread per CPU, the old default)   102 to 107 %
    SingleThreadedExecutor                                          65 to 74 %
      + TF and /odom throttled to 30 Hz                             60 %
      + no parameter services on any node (nothing calls them)      57 %
    rclpy.experimental.EventsExecutor                               17.6 %

The events executor is woken by the middleware per event and keeps no wait
set, so the per-wake-up cost that dominated here is gone. It is marked
experimental in rclpy 7.1 (Jazzy), so it is used when it imports and the
SingleThreadedExecutor is the fallback. AVOIDANCE_SIM_EXECUTOR=single forces
the fallback, for comparison or if the experimental one misbehaves. The
pilot's 50 Hz sticks and the 30 Hz TF were verified at rate under both, and
test/gate.py is the check that flying behaviour is unchanged: run it after
touching anything in here.

Single-threaded either way: no callback in these nodes blocks, each takes
well under a millisecond, and the pilot's tick can no longer interleave with
its own position callback on the same fields, which the multi-threaded
version allowed.
"""

import os

import rclpy
from rclpy.executors import SingleThreadedExecutor

try:
    from rclpy.experimental import EventsExecutor
except ImportError:          # an rclpy without it: the fallback below is used
    EventsExecutor = None

from .command_marker import CommandMarker
from .goal_3d import Goal3D
from .goal_bridge import GoalBridge
from .software_pilot import SoftwarePilot
from .tf_publisher import TfPublisher
from .world_markers import WorldMarkers

# Order matches the single-file prototype this was split from. Nothing
# depends on it, but the split was meant to preserve behaviour exactly.
NODE_TYPES = (TfPublisher, GoalBridge, WorldMarkers, CommandMarker, Goal3D,
              SoftwarePilot)


def make_executor():
    want = os.environ.get('AVOIDANCE_SIM_EXECUTOR', 'events').strip().lower()
    if want == 'events' and EventsExecutor is not None:
        return EventsExecutor(), 'EventsExecutor'
    if want == 'events':
        print('[rviz_bridge] rclpy.experimental.EventsExecutor not available, '
              'using SingleThreadedExecutor')
    return SingleThreadedExecutor(), 'SingleThreadedExecutor'


def main(args=None):
    rclpy.init(args=args)
    # Build each node separately. One throwing constructor used to kill the
    # process before the executor started, so a missing dependency in any
    # single node silently took the other five features with it and the user
    # saw one traceback naming one module.
    nodes = []
    for cls in NODE_TYPES:
        try:
            nodes.append(cls())
        except Exception as exc:
            print('[rviz_bridge] %s failed to start: %s' % (cls.__name__, exc))
    if not nodes:
        print('[rviz_bridge] nothing started, giving up')
        rclpy.try_shutdown()
        return
    if len(nodes) < len(NODE_TYPES):
        print('[rviz_bridge] running degraded: %d of %d nodes'
              % (len(nodes), len(NODE_TYPES)))
    ex, name = make_executor()
    print('[rviz_bridge] executor: %s' % name)
    for n in nodes:
        ex.add_node(n)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for n in nodes:
            n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
