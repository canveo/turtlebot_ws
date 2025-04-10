from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlebot2_bringup',
            executable='turtlebot2_bringup.launch.py',
            name='turtlebot2_bringup'
        ),
        Node(
            package='follow_line',
            executable='follow_line.launch.py',
            name='follow_line'
        ),
        ExecuteProcess(
            cmd=['python3', '-u', 'bag_turtle.py'],  
        )
    ])