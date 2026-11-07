import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            RegisterEventHandler, SetEnvironmentVariable)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = [
        DeclareLaunchArgument('world', default_value='small_house'),
        DeclareLaunchArgument('x', default_value='1.0'),
        DeclareLaunchArgument('y', default_value='1.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('software_render', default_value='true'),
        DeclareLaunchArgument('run_nav2', default_value='true'),
        DeclareLaunchArgument('no_dock', default_value='true'),
        DeclareLaunchArgument('model', default_value='standard'),
        DeclareLaunchArgument('no_camera', default_value='false'),
        DeclareLaunchArgument('lidar_lowrate', default_value='false'),
        DeclareLaunchArgument(
            'nav_params_file',
            default_value=os.path.join(
                os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                'config', 'nav2_voicenav.yaml')),
        DeclareLaunchArgument(
            'localization_params_file',
            default_value=os.path.join(
                os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                'config', 'localization_voicenav.yaml')),
    ]

    pkg_bringup = get_package_share_directory('turtlebot4_ignition_bringup')
    pkg_gui_plugins = get_package_share_directory('turtlebot4_ignition_gui_plugins')
    pkg_tb4_description = get_package_share_directory('turtlebot4_description')
    pkg_create_description = get_package_share_directory('irobot_create_description')
    pkg_create_bringup = get_package_share_directory('irobot_create_ignition_bringup')
    pkg_create_plugins = get_package_share_directory('irobot_create_ignition_plugins')
    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')

    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[os.path.join(pkg_bringup, 'worlds'), ':' +
               os.path.join(pkg_create_bringup, 'worlds'), ':' +
               str(Path(pkg_tb4_description).parent.resolve()), ':' +
               str(Path(pkg_create_description).parent.resolve())])
    ign_gui_plugin_path = SetEnvironmentVariable(
        name='IGN_GUI_PLUGIN_PATH',
        value=[os.path.join(pkg_gui_plugins, 'lib'), ':' +
               os.path.join(pkg_create_plugins, 'lib')])
    software_render = SetEnvironmentVariable(
        name='LIBGL_ALWAYS_SOFTWARE', value='1',
        condition=IfCondition(LaunchConfiguration('software_render')))

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py'])]),
        launch_arguments=[
            ('ign_args', [LaunchConfiguration('world'),
                          '.sdf -s -r --headless-rendering'])])

    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge',
        output='screen',
        arguments=['/clock' + '@rosgraph_msgs/msg/Clock' + '[ignition.msgs.Clock'])

    spawn_with_dock = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([pkg_bringup, 'launch', 'turtlebot4_spawn.launch.py'])]),
        launch_arguments={
            'rviz': 'false', 'model': LaunchConfiguration('model'),
            'x': LaunchConfiguration('x'), 'y': LaunchConfiguration('y'),
            'yaw': LaunchConfiguration('yaw')}.items(),
        condition=UnlessCondition(LaunchConfiguration('no_dock')))

    spawn_no_dock = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          'turtlebot4_spawn_no_dock.launch.py')]),
        launch_arguments={
            'rviz': 'false', 'model': LaunchConfiguration('model'),
            'no_camera': LaunchConfiguration('no_camera'),
            'lidar_lowrate': LaunchConfiguration('lidar_lowrate'),
            'x': LaunchConfiguration('x'), 'y': LaunchConfiguration('y'),
            'yaw': LaunchConfiguration('yaw')}.items(),
        condition=IfCondition(LaunchConfiguration('no_dock')))

    set_safety_override = ExecuteProcess(
        cmd=['python3', '-c', (
            "import rclpy, sys, time\n"
            "from rclpy.node import Node\n"
            "from rcl_interfaces.srv import SetParameters\n"
            "from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType\n"
            "rclpy.init()\n"
            "node = Node('set_safety_override_check')\n"
            "cli = node.create_client(SetParameters, '/motion_control/set_parameters')\n"
            "t0 = time.time()\n"
            "ok = False\n"
            "while time.time() - t0 < 1200:\n"
            "    if not cli.wait_for_service(timeout_sec=5.0):\n"
            "        continue\n"
            "    req = SetParameters.Request()\n"
            "    val = ParameterValue(type=ParameterType.PARAMETER_STRING,"
            " string_value='backup_only')\n"
            "    req.parameters = [Parameter(name='safety_override', value=val)]\n"
            "    fut = cli.call_async(req)\n"
            "    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)\n"
            "    res = fut.result() if fut.done() else None\n"
            "    if res is not None and res.results and res.results[0].successful:\n"
            "        ok = True\n"
            "        break\n"
            "    time.sleep(1.0)\n"
            "node.destroy_node()\n"
            "rclpy.shutdown()\n"
            "sys.exit(0 if ok else 1)\n")],
        name='set_safety_override', output='screen',
        condition=IfCondition(LaunchConfiguration('no_dock')))

    wait_sim = ExecuteProcess(
        cmd=['python3', '-c', (
            "import rclpy, re, sys, time\n"
            "from rclpy.node import Node\n"
            "from controller_manager_msgs.srv import ListControllers\n"
            "rclpy.init()\n"
            "node = Node('wait_sim_check')\n"
            "cli = node.create_client(ListControllers, '/controller_manager/list_controllers')\n"
            "t0 = time.time()\n"
            "ok = False\n"
            "while time.time() - t0 < 1200:\n"
            "    if not cli.wait_for_service(timeout_sec=5.0):\n"
            "        continue\n"
            "    fut = cli.call_async(ListControllers.Request())\n"
            "    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)\n"
            "    res = fut.result() if fut.done() else None\n"
            "    if res is not None:\n"
            "        for c in res.controller:\n"
            "            if re.search(r'diff_?drive_controller', c.name, re.I) "
            "and c.state == 'active':\n"
            "                ok = True\n"
            "    if ok:\n"
            "        break\n"
            "    time.sleep(1.0)\n"
            "node.destroy_node()\n"
            "rclpy.shutdown()\n"
            "sys.exit(0 if ok else 1)\n")],
        name='wait_sim', output='log')

    nav_pkg = FindPackageShare('turtlebot4_navigation')
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav_pkg, 'launch', 'localization.launch.py'])),
        launch_arguments={
            'map': LaunchConfiguration('map'), 'use_sim_time': 'true',
            'params': LaunchConfiguration('localization_params_file')}.items(),
        condition=IfCondition(LaunchConfiguration('run_nav2')))

    wait_amcl = ExecuteProcess(
        cmd=['python3', '-c', (
            "import rclpy, sys, time\n"
            "from rclpy.node import Node\n"
            "from lifecycle_msgs.srv import GetState\n"
            "rclpy.init()\n"
            "node = Node('wait_amcl_check')\n"
            "cli = node.create_client(GetState, '/amcl/get_state')\n"
            "t0 = time.time()\n"
            "ok = False\n"
            "while time.time() - t0 < 1200:\n"
            "    if not cli.wait_for_service(timeout_sec=5.0):\n"
            "        continue\n"
            "    fut = cli.call_async(GetState.Request())\n"
            "    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)\n"
            "    res = fut.result() if fut.done() else None\n"
            "    if res is not None and res.current_state.label == 'active':\n"
            "        ok = True\n"
            "        break\n"
            "    time.sleep(1.0)\n"
            "node.destroy_node()\n"
            "rclpy.shutdown()\n"
            "sys.exit(0 if ok else 1)\n")],
        name='wait_amcl', output='log',
        condition=IfCondition(LaunchConfiguration('run_nav2')))

    cov = '[' + ', '.join(
        ['0.25' if i in (0, 7) else '0.07' if i == 35 else '0.0' for i in range(36)]) + ']'
    pose = ('{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, '
            'orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, covariance: %s}}' % cov)
    initial = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '-t', '8', '-r', '1', '/initialpose',
             'geometry_msgs/msg/PoseWithCovarianceStamped', pose], output='screen')
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav_pkg, 'launch', 'nav2.launch.py'])),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': LaunchConfiguration('nav_params_file')}.items())

    on_sim = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_sim,
            on_exit=[localization, wait_amcl, set_safety_override]))
    on_amcl = RegisterEventHandler(
        OnProcessExit(target_action=wait_amcl, on_exit=[initial, nav2]))

    return LaunchDescription(
        args + [ign_resource_path, ign_gui_plugin_path, software_render, gazebo, clock_bridge,
                spawn_with_dock, spawn_no_dock, wait_sim, on_sim, on_amcl])
