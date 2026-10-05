#!/usr/bin/env python3
"""Everything that makes the aircraft controllable from RViz, in one process.

Six nodes share a MultiThreadedExecutor. They are one process rather than six
because they are cheap, they start and stop together, and a single process is
one thing for a reader to watch rather than six.

    TfPublisher     PX4 odometry -> the TF tree RViz needs
    WorldMarkers    the Gazebo world's obstacles, drawn in RViz
    GoalBridge      RViz's flat "2D Goal Pose" tool -> a PX4 reposition
    CommandMarker   the right-click menu above the aircraft
    Goal3D          the draggable 3D waypoint, with a ring for heading
    SoftwarePilot   flies to a goal on synthetic sticks

The pilot is the load-bearing one. PX4's collision prevention runs in Position
mode only, so the aircraft has to be flown on sticks for avoidance to apply at
all. The pilot streams synthetic manual control to do exactly that, which is
what lets a clicked goal and working avoidance coexist.
"""

import rclpy
from rclpy.executors import MultiThreadedExecutor

from .command_marker import CommandMarker
from .goal_3d import Goal3D
from .goal_bridge import GoalBridge
from .software_pilot import SoftwarePilot
from .tf_publisher import TfPublisher
from .world_markers import WorldMarkers

NODE_TYPES = (TfPublisher, WorldMarkers, GoalBridge, CommandMarker, Goal3D,
              SoftwarePilot)


def main(args=None):
    rclpy.init(args=args)
    nodes = [cls() for cls in NODE_TYPES]
    ex = MultiThreadedExecutor()
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
