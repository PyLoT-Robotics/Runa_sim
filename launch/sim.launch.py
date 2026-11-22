"""Runa + SOBITS arena + optional SLAM/Nav2, ROS 2 Jazzy / Harmonic."""
import os
import tempfile
from pathlib import Path

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable, LogInfo, EmitEvent
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
from runa_sim.world import prepare_world
from runa_sim.model import prepare_model


def build(context):
    share = Path(get_package_share_directory('runa_sim'))
    desc = Path(get_package_share_directory('runa_description'))
    worlds = Path(get_package_share_directory('sobits_gazebo_worlds'))
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    full = LaunchConfiguration('full_field').perform(context).lower() == 'true'
    models = [worlds / 'models', desc.parent]
    for package in ['tmc_wrs_gz_worlds', 'aws_small_house_world']:
        try:
            models.append(Path(get_package_share_directory(package)) / 'models')
        except PackageNotFoundError:
            pass
    inherited = [p for p in os.environ.get('GZ_SIM_RESOURCE_PATH', '').split(os.pathsep) if p]
    models.extend(Path(p) for p in inherited)
    source = worlds / 'worlds' / LaunchConfiguration('world').perform(context)
    temp = Path(tempfile.mkdtemp(prefix='runa_sim_'))
    world_name, omitted = prepare_world(source, models, temp / 'arena.sdf', full)
    description = xacro.process_file(str(desc / 'urdf/runa.urdf.xacro'), mappings={
        'sim': 'true', 'controllers_file': str(share / 'config/controllers.yaml')}).toxml()
    (temp / 'robot.urdf').write_text(description)
    prepare_model(temp / 'robot.urdf', temp / 'robot.sdf')
    nav2 = Path(get_package_share_directory('nav2_bringup'))
    slam = Path(get_package_share_directory('slam_toolbox'))
    gz_args = '-r ' + ('' if gui else '-s --headless-rendering ') + str(temp / 'arena.sdf')
    spawn = Node(package='ros_gz_sim', executable='create', output='screen', arguments=[
        '-world', world_name, '-name', 'runa', '-file', str(temp / 'robot.sdf'),
        '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'), '-z', '0.03',
        '-Y', LaunchConfiguration('yaw'), '-allow_renaming', 'false'])
    broadcaster = Node(package='controller_manager', executable='spawner', output='screen',
                       arguments=['joint_state_broadcaster', '--controller-manager-timeout', '120'],
                       parameters=[{'use_sim_time': True}])
    controllers = Node(package='controller_manager', executable='spawner', output='screen',
                       arguments=['arm_controller', 'gripper_controller', 'head_controller', 'lift_controller',
                                  '--controller-manager-timeout', '120'], parameters=[{'use_sim_time': True}])
    # Start controllers only after successful spawn. Failures terminate the launch.
    def after_spawn(event, _context):
        return [broadcaster] if event.returncode == 0 else [EmitEvent(event=Shutdown(reason='Robot spawn failed'))]
    def after_broadcaster(event, _context):
        return [controllers] if event.returncode == 0 else [EmitEvent(event=Shutdown(reason='Joint-state broadcaster failed'))]
    return [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.pathsep.join(str(m) for m in models)),
        LogInfo(msg='SOBITS field: ' + str(source) + '; omitted unavailable optional furniture: ' + str(omitted)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(get_package_share_directory('ros_gz_sim')) / 'launch/gz_sim.launch.py')),
                                 launch_arguments={'gz_args': gz_args}.items()),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': description, 'use_sim_time': True}]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             parameters=[{'config_file': str(share / 'config/bridge.yaml'), 'use_sim_time': True}]),
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=after_spawn)),
        RegisterEventHandler(OnProcessExit(target_action=broadcaster, on_exit=after_broadcaster)),
        spawn,
        Node(package='runa_sim', executable='compatibility', parameters=[{'use_sim_time': True}], output='screen'),
        *[Node(package='runa_sim', executable='rgbd_bridge', name='rgbd_' + cam,
               parameters=[{'use_sim_time': True, 'camera': cam}], output='screen') for cam in ['d455', 'd415']],
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(slam / 'launch/online_async_launch.py')),
            launch_arguments={'use_sim_time': 'true', 'slam_params_file': str(share / 'config/slam.yaml')}.items(),
            condition=IfCondition(LaunchConfiguration('navigation'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(nav2 / 'launch/navigation_launch.py')),
            launch_arguments={'use_sim_time': 'true', 'autostart': 'true', 'use_composition': 'False',
                              'params_file': str(share / 'config/nav2.yaml')}.items(),
            condition=IfCondition(LaunchConfiguration('navigation'))),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('navigation', default_value='true'),
        DeclareLaunchArgument('full_field', default_value='false'),
        DeclareLaunchArgument('world', default_value='rcjo2025_arena.world.xacro'),
        DeclareLaunchArgument('x', default_value='-2.0'),
        DeclareLaunchArgument('y', default_value='1.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        OpaqueFunction(function=build),
    ])
