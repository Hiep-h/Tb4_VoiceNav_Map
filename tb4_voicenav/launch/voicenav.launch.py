from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    speech_node = Node(
        package='tb4_voicenav',
        executable='speech_node',
        name='speech_node',
        output='screen',
    )
    voice_nav_node = Node(
        package='tb4_voicenav',
        executable='voice_nav_node',
        name='voice_nav_node',
        output='screen',
    )
    return LaunchDescription([speech_node, voice_nav_node])
