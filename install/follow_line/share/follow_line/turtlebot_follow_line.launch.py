from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
import launch_ros
from launch_ros.actions import Node

def generate_launch_description():

    kobuki_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('kobuki_node'),
                'launch',
                'kobuki_node-launch.py'
            ])
        ]),
        launch_arguments={
            'cmd_vel_topic': '/cmd_vel'
        }.items()
    )

    nav_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ])
    )

    follow_line_node = Node(
        package='follow_line',
        executable='follow_line_node',
        name='follow_line_node',
        output='screen',
        # parameters=[{'param_name': 'param_value'}], 
    )

    return LaunchDescription([
        kobuki_node,
        nav_node,
        follow_line_node
    ])
