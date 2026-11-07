import os

from ament_index_python.packages import get_package_share_directory

from irobot_create_common_bringup.namespace import GetNamespacedName

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node, PushRosNamespace


ARGUMENTS = [
    DeclareLaunchArgument(
        'rviz', default_value='false', choices=['true', 'false']),
    DeclareLaunchArgument(
        'use_sim_time', default_value='true', choices=['true', 'false']),
    DeclareLaunchArgument(
        'model', default_value='standard', choices=['standard', 'lite']),
    DeclareLaunchArgument('namespace', default_value=''),
    DeclareLaunchArgument(
        'localization', default_value='false', choices=['true', 'false']),
    DeclareLaunchArgument(
        'slam', default_value='false', choices=['true', 'false']),
    DeclareLaunchArgument(
        'nav2', default_value='false', choices=['true', 'false']),
    DeclareLaunchArgument(
        'no_camera', default_value='false', choices=['true', 'false']),
    DeclareLaunchArgument(
        'lidar_lowrate', default_value='false', choices=['true', 'false']),
]

for pose_element in ['x', 'y', 'z', 'yaw']:
    ARGUMENTS.append(
        DeclareLaunchArgument(pose_element, default_value='0.0'))


def generate_launch_description():
    pkg_turtlebot4_ignition_bringup = get_package_share_directory(
        'turtlebot4_ignition_bringup')
    pkg_turtlebot4_description = get_package_share_directory(
        'turtlebot4_description')
    pkg_turtlebot4_viz = get_package_share_directory(
        'turtlebot4_viz')
    pkg_turtlebot4_navigation = get_package_share_directory(
        'turtlebot4_navigation')
    pkg_irobot_create_common_bringup = get_package_share_directory(
        'irobot_create_common_bringup')
    pkg_irobot_create_ignition_bringup = get_package_share_directory(
        'irobot_create_ignition_bringup')

    turtlebot4_ros_ign_bridge_launch = [
        pkg_turtlebot4_ignition_bringup, '/launch/ros_ign_bridge.launch.py']
    rviz_launch = [pkg_turtlebot4_viz, '/launch/view_robot.launch.py']
    turtlebot4_node_launch = [
        pkg_turtlebot4_ignition_bringup, '/launch/turtlebot4_nodes.launch.py']
    create3_nodes_launch = [
        pkg_irobot_create_common_bringup, '/launch/create3_nodes.launch.py']
    create3_ignition_nodes_launch = [
        pkg_irobot_create_ignition_bringup, '/launch/create3_ignition_nodes.launch.py']
    robot_description_launch = [
        pkg_turtlebot4_description, '/launch/robot_description.launch.py']
    robot_description_no_camera_launch = [
        os.path.dirname(os.path.abspath(__file__)),
        '/robot_description_no_camera.launch.py']
    robot_description_lowrate_lidar_launch = [
        os.path.dirname(os.path.abspath(__file__)),
        '/robot_description_lowrate_lidar.launch.py']
    robot_description_no_camera_lowrate_lidar_launch = [
        os.path.dirname(os.path.abspath(__file__)),
        '/robot_description_no_camera_lowrate_lidar.launch.py']
    localization_launch = [
        pkg_turtlebot4_navigation, '/launch/localization.launch.py']
    slam_launch = [pkg_turtlebot4_navigation, '/launch/slam.launch.py']
    nav2_launch = [pkg_turtlebot4_navigation, '/launch/nav2.launch.py']

    param_file_cmd = DeclareLaunchArgument(
        'param_file',
        default_value=[
            pkg_turtlebot4_ignition_bringup, '/config/turtlebot4_node.yaml'],
        description='Turtlebot4 Robot param file')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    x, y, z = LaunchConfiguration('x'), LaunchConfiguration('y'), LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    turtlebot4_node_yaml_file = LaunchConfiguration('param_file')

    robot_name = GetNamespacedName(namespace, 'turtlebot4')
    dock_name = GetNamespacedName(namespace, 'standard_dock')

    spawn_robot_group_action = GroupAction([
        PushRosNamespace(namespace),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_description_launch),
            launch_arguments=[
                ('model', LaunchConfiguration('model')),
                ('use_sim_time', LaunchConfiguration('use_sim_time'))],
            condition=UnlessCondition(PythonExpression([
                "'", LaunchConfiguration('no_camera'), "' == 'true' or '",
                LaunchConfiguration('lidar_lowrate'), "' == 'true'"])),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_description_no_camera_launch),
            launch_arguments=[
                ('use_sim_time', LaunchConfiguration('use_sim_time'))],
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('no_camera'), "' == 'true' and '",
                LaunchConfiguration('lidar_lowrate'), "' == 'false'"])),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_description_lowrate_lidar_launch),
            launch_arguments=[
                ('use_sim_time', LaunchConfiguration('use_sim_time'))],
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('no_camera'), "' == 'false' and '",
                LaunchConfiguration('lidar_lowrate'), "' == 'true'"])),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_description_no_camera_lowrate_lidar_launch),
            launch_arguments=[
                ('use_sim_time', LaunchConfiguration('use_sim_time'))],
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('no_camera'), "' == 'true' and '",
                LaunchConfiguration('lidar_lowrate'), "' == 'true'"])),
        ),

        Node(
            package='ros_ign_gazebo',
            executable='create',
            arguments=['-name', robot_name,
                       '-x', x,
                       '-y', y,
                       '-z', z,
                       '-Y', yaw,
                       '-topic', 'robot_description'],
            output='screen'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(turtlebot4_ros_ign_bridge_launch),
            launch_arguments=[
                ('model', LaunchConfiguration('model')),
                ('robot_name', robot_name),
                ('dock_name', dock_name),
                ('namespace', namespace)]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(turtlebot4_node_launch),
            launch_arguments=[
                ('model', LaunchConfiguration('model')),
                ('param_file', turtlebot4_node_yaml_file)]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(create3_nodes_launch),
            launch_arguments=[
                ('namespace', namespace)
            ]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(create3_ignition_nodes_launch),
            launch_arguments=[
                ('robot_name', robot_name),
                ('dock_name', dock_name),
            ]
        ),

        Node(
            name='rplidar_stf',
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=[
                '0', '0', '0', '0', '0', '0.0',
                'rplidar_link', [robot_name, '/rplidar_link/rplidar']],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
            ]
        ),

        Node(
            name='camera_stf',
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=[
                '0', '0', '0',
                '1.5707', '-1.5707', '0',
                'oakd_rgb_camera_optical_frame',
                [robot_name, '/oakd_rgb_camera_frame/rgbd_camera']
            ],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
            ]
        ),

    ])

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch),
        launch_arguments=[
            ('namespace', namespace),
            ('use_sim_time', use_sim_time)
        ],
        condition=IfCondition(LaunchConfiguration('localization'))
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch),
        launch_arguments=[
            ('namespace', namespace),
            ('use_sim_time', use_sim_time)
        ],
        condition=IfCondition(LaunchConfiguration('slam'))
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments=[
            ('namespace', namespace),
            ('use_sim_time', use_sim_time)
        ],
        condition=IfCondition(LaunchConfiguration('nav2'))
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rviz_launch),
        launch_arguments=[
            ('namespace', namespace),
            ('use_sim_time', use_sim_time)],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(param_file_cmd)
    ld.add_action(spawn_robot_group_action)
    ld.add_action(localization)
    ld.add_action(slam)
    ld.add_action(nav2)
    ld.add_action(rviz)
    return ld
