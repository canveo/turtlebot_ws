import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    pkg_share = FindPackageShare(package='turtlebot2_description').find('turtlebot2_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'turtlebot_gazebo.urdf.xacro')

    # Nodo para spawnear el modelo en Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'turtlebot2', '-file', urdf_file],
        output='screen'
    )

    # Nodo para publicar el estado del robot
    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': urdf_file}]
    )

    # Nodo para lanzar Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ])
    )

    return LaunchDescription([
        gazebo,
        spawn_entity,
        state_publisher
    ])
