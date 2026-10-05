"""Bring up everything except PX4 itself.

PX4 is deliberately left out. Starting it means running a make target from the
PX4 source tree, which is a build step as much as a run step, and burying a
build inside a launch file makes failures hard to read. So this is two
commands rather than one:

    cd ~/PX4-Autopilot && PX4_GZ_WORLD=walls make px4_sitl gz_x500_depth
    ros2 launch avoidance_sim sim.launch.py

Down from the four terminals in a fixed order that this replaces.

use_sim_time is set on every node. Gazebo owns the clock here, and a node left
on wall time will timestamp its messages in a different epoch to PX4, which
shows up as transforms that will not resolve rather than as an error.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Gazebo topic -> ROS topic. The [ means Gazebo to ROS only.
BRIDGE_TOPICS = [
    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
    '/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
]


def generate_launch_description():
    share = get_package_share_directory('avoidance_sim')
    rviz_config = os.path.join(share, 'config', 'avoidance.rviz')

    use_rviz = LaunchConfiguration('rviz')
    use_agent = LaunchConfiguration('agent')
    agent_cmd = LaunchConfiguration('agent_cmd')
    sim_time = [{'use_sim_time': True}]

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Start RViz with the bundled config.'),
        DeclareLaunchArgument(
            'agent', default_value='true',
            description='Start the Micro XRCE-DDS Agent. Set false if you '
                        'already run it yourself.'),
        DeclareLaunchArgument(
            'agent_cmd', default_value='MicroXRCEAgent',
            description='Path to the agent binary, if it is not on PATH.'),
        DeclareLaunchArgument(
            'world_sdf', default_value='',
            description='Gazebo world file to draw obstacles from. Empty '
                        'means the node default, ~/PX4-Autopilot/Tools/'
                        'simulation/gz/worlds/walls.sdf.'),

        # PX4 talks to ROS 2 through this. Without it nothing below receives
        # anything and the stack looks dead with no diagnosable cause.
        ExecuteProcess(
            condition=IfCondition(use_agent),
            cmd=[agent_cmd, 'udp4', '-p', '8888'],
            output='screen'),

        Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            name='gz_bridge', arguments=BRIDGE_TOPICS,
            parameters=sim_time, output='screen'),

        Node(
            package='avoidance_sim', executable='obstacle_distance',
            parameters=sim_time, output='screen'),

        # No name= here, deliberately. This executable hosts six nodes in one
        # process, and name= becomes a __node remap that renames all of them
        # to the same thing, so they vanish from `ros2 node list` under their
        # real names and collide with each other. Each node names itself.
        Node(
            package='avoidance_sim', executable='rviz_bridge',
            parameters=sim_time + [{'world_sdf': LaunchConfiguration('world_sdf')}],
            output='screen'),

        Node(
            condition=IfCondition(use_rviz),
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', rviz_config],
            # screen, not log: RViz's OpenGL startup failures are the most
            # common new-user problem, and in a log file nobody finds them.
            parameters=sim_time, output='screen'),
    ])
