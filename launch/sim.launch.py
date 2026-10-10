"""Bring up everything except PX4 itself.

PX4 is left out on purpose. Starting it means running a make target from the
PX4 source tree, which is a build step as much as a run step, and burying a
build inside a launch file makes failures hard to read. So this is two
commands rather than one:

    cd ~/PX4-Autopilot && PX4_GZ_WORLD=walls HEADLESS=1 make px4_sitl gz_x500_depth
    ros2 launch avoidance_sim sim.launch.py

Another world: PX4_GZ_WORLD=pillars for PX4 and world:=pillars here, so the
walls RViz draws are the ones PX4 loaded. The aircraft with the extra lidar
(models/x500_depth_lidar) is started through PX4's binary, because make
targets only exist for models with an airframe file in PX4's tree:

    cd ~/PX4-Autopilot && PX4_SYS_AUTOSTART=4002 PX4_SIM_MODEL=gz_x500_depth_lidar \\
        PX4_GZ_WORLD=walls HEADLESS=1 ./build/px4_sitl_default/bin/px4
    ros2 launch avoidance_sim sim.launch.py lidar:=true

Sensors are described to the obstacle node by a ROS parameter file, not by
editing this launch: lidar:=true loads config/sensors_lidar.yaml, and
sensors:=/path/to/yours.yaml loads any other (config/sensors_example.yaml is
the template). A sensor on a new Gazebo topic also needs that topic bridged,
which is bridge_extra:="/topic@ros_type[gz_type ..." in the same syntax as
BRIDGE_TOPICS below. docs/extend/sensors.md has the recipes, docs/data.md the inputs.

This launch also sets the five PX4 parameters the stack needs, through PX4's
own px4-param client, as soon as PX4 answers (px4_params to change them,
px4_bin if PX4 is not at ~/PX4-Autopilot). The clean-clone test found that a
fresh install never arms: the x500 airframe defaults NAV_DLL_ACT to 2, wait
for a ground station, and the development machine had 0 saved for months.
scripts/px4_params.sh says why it is done this way and not with PX4_PARAM_
environment variables, which cannot set that one.

Down from the four terminals in a fixed order that this replaces.

use_sim_time is set on every node. Gazebo owns the clock here, and a node left
on wall time will timestamp its messages in a different epoch to PX4, which
shows up as transforms that will not resolve rather than as an error.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Gazebo topic -> ROS topic. The [ means Gazebo to ROS only.
BRIDGE_TOPICS = [
    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
    '/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
]
# The 2D lidar on models/x500_depth_lidar, bridged with lidar:=true.
LIDAR_BRIDGE = '/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'


def sensor_nodes(context, share, sim_time):
    """The Gazebo bridge and the obstacle node, shaped by the sensor arguments.

    An OpaqueFunction because a launch argument cannot grow a list: the extra
    bridge topics and the parameter file are only known once the arguments
    are resolved. Everything else in this launch is static.
    """
    lidar = LaunchConfiguration('lidar').perform(context).strip().lower() in ('true', '1', 'yes')
    sensors = LaunchConfiguration('sensors').perform(context).strip()
    extra = LaunchConfiguration('bridge_extra').perform(context).split()
    if lidar:
        sensors = sensors or os.path.join(share, 'config', 'sensors_lidar.yaml')
        if LIDAR_BRIDGE not in extra:
            extra.append(LIDAR_BRIDGE)
    params = list(sim_time)
    if sensors:
        params.append(os.path.expanduser(sensors))
    return [
        Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            name='gz_bridge', arguments=BRIDGE_TOPICS + extra,
            parameters=sim_time, output='screen'),
        Node(
            package='avoidance_sim', executable='obstacle_distance',
            parameters=params, output='screen'),
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
            'px4_bin',
            default_value=os.path.expanduser('~/PX4-Autopilot/build/px4_sitl_default/bin'),
            description='PX4 SITL build bin directory, for px4-param.'),
        DeclareLaunchArgument(
            'px4_params',
            default_value='NAV_DLL_ACT=0 NAV_RCL_ACT=0 CP_DIST=2.0 CP_GO_NO_DATA=1 '
                          'COM_RC_OVERRIDE=0',
            description='PX4 parameters set once PX4 answers. The README '
                        'says why each is needed. Empty string to skip.'),
        DeclareLaunchArgument(
            'world', default_value='walls',
            description='The world PX4 was started with (PX4_GZ_WORLD), so '
                        'RViz draws its obstacles. Looked up where PX4 looks, '
                        'then in this package\'s worlds/.'),
        DeclareLaunchArgument(
            'world_sdf', default_value='',
            description='A world file by path instead of by name. Empty '
                        'means use world:=.'),
        DeclareLaunchArgument(
            'lidar', default_value='false',
            description='The aircraft is models/x500_depth_lidar: bridge its '
                        '/lidar scan and merge it into the histogram '
                        '(config/sensors_lidar.yaml).'),
        DeclareLaunchArgument(
            'sensors', default_value='',
            description='A ROS parameter file describing the obstacle node\'s '
                        'sensors. config/sensors_example.yaml is the template. '
                        'Empty means the stock camera, or the lidar file with '
                        'lidar:=true.'),
        DeclareLaunchArgument(
            'bridge_extra', default_value='',
            description='Extra Gazebo-to-ROS bridge specs, space-separated, '
                        'for the topics a sensors file needs: '
                        '"/rear/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked".'),

        # PX4 talks to ROS 2 through this. Without it nothing below receives
        # anything and the stack looks dead with no diagnosable cause.
        ExecuteProcess(
            condition=IfCondition(use_agent),
            cmd=[agent_cmd, 'udp4', '-p', '8888'],
            output='screen'),

        # The parameters PX4 needs for this stack, set by PX4's own client once
        # PX4 is up. NAV_DLL_ACT 0 is the one a fresh install cannot fly
        # without; scripts/px4_params.sh has the whole story.
        ExecuteProcess(
            cmd=['bash', os.path.join(share, 'scripts', 'px4_params.sh'),
                 LaunchConfiguration('px4_bin'), LaunchConfiguration('px4_params')],
            output='screen'),

        # The bridge and the obstacle node, shaped by lidar:=, sensors:= and
        # bridge_extra:= (see sensor_nodes above).
        OpaqueFunction(function=sensor_nodes, args=[share, sim_time]),

        # No name= here, on purpose. This executable hosts six nodes in one
        # process, and name= becomes a __node remap that renames all of them
        # to the same thing, so they vanish from `ros2 node list` under their
        # real names and collide with each other. Each node names itself.
        Node(
            package='avoidance_sim', executable='rviz_bridge',
            parameters=sim_time + [{'world': LaunchConfiguration('world'),
                                    'world_sdf': LaunchConfiguration('world_sdf')}],
            output='screen'),

        Node(
            condition=IfCondition(use_rviz),
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', rviz_config],
            # screen, not log: RViz's OpenGL startup failures are the most
            # common new-user problem, and in a log file nobody finds them.
            parameters=sim_time, output='screen'),
    ])
