from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        # Static transform publisher node
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_transform',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']
        ),

        # Node to run the circle_wave_publisher
        Node(
            package='rviz2_visuals',
            executable='circle_wave',
            name='circle_wave_node',
            output='screen'
        ),
        
        # Node to run RViz2
        ExecuteProcess(
            cmd=['rviz2', '-d', os.path.join(
                get_package_share_directory('rviz2_visuals'), 'rviz', 'rviz2_visual.rviz')],
            output='screen'
        ),
    ])
