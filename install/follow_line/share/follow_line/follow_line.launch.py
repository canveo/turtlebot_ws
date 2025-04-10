import launch
from launch_ros.actions import Node

def generate_launch_description():
    return launch.LaunchDescription([
        #  Node(
        #     package='usb_cam',
        #     executable='usb_cam_node_exe',
        #     name='usb_cam_node',
        #     output='screen',
        #     parameters=[{'use_camera_info': False}],
        # ),
        Node(
            package='follow_line',
            executable='follow_line_node',
            name='follow_line_node',
            output='screen',  
        ),
        
    ])
