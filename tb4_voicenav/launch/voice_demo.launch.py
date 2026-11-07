import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TOOLS_DIR = os.path.join(PKG_ROOT, 'tools')
CONFIG_DIR = os.path.join(PKG_ROOT, 'config')
MAPS_DIR = os.path.join(PKG_ROOT, 'maps')

PY_ENV = dict(os.environ)
PY_ENV['PYTHONPATH'] = PKG_ROOT + os.pathsep + os.environ.get('PYTHONPATH', '')


def generate_launch_description():
    rviz_arg = DeclareLaunchArgument('rviz', default_value='false')
    regions_arg = DeclareLaunchArgument(
        'regions_json_path', default_value=os.path.join(MAPS_DIR, 'regions.json'))

    sim_and_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(TOOLS_DIR, 'headless_nav_eval.launch.py')),
        launch_arguments={
            'map': os.path.join(MAPS_DIR, 'map.yaml'),
            'software_render': 'true',
            'no_dock': 'true',
            'run_nav2': 'true',
            'x': '1.0', 'y': '1.0', 'yaw': '0.0',
        }.items())

    voice_nav_node = ExecuteProcess(
        cmd=['python3', os.path.join(PKG_ROOT, 'tb4_voicenav', 'voice_nav_node.py'),
             '--ros-args', '-p', ['regions_json_path:=', LaunchConfiguration('regions_json_path')],
             '-p', 'use_sim_time:=true'],
        name='voice_nav_node', output='screen', env=PY_ENV)

    region_markers_node = ExecuteProcess(
        cmd=['python3', os.path.join(PKG_ROOT, 'tb4_voicenav', 'region_markers_node.py'),
             '--ros-args', '-p', ['regions_json_path:=', LaunchConfiguration('regions_json_path')],
             '-p', 'use_sim_time:=true'],
        name='region_markers_node', output='screen', env=PY_ENV)

    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', os.path.join(CONFIG_DIR, 'voice_demo.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')))

    return LaunchDescription([
        rviz_arg, regions_arg, sim_and_nav2, voice_nav_node, region_markers_node, rviz_node])
