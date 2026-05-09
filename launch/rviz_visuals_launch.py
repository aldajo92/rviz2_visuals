from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')

    rviz_config = os.path.join(
        get_package_share_directory('rviz2_visuals'),
        'rviz',
        'rviz2_visual.rviz'
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Whether to launch RViz2'
        ),

        # Static transform publisher node
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_transform',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']
        ),

        # Node to run the arrow_random
        Node(
            package='rviz2_visuals',
            executable='arrow_random',
            name='arrow_random_node',
            output='screen'
        ),

        # Node to run the circle_wave_publisher
        Node(
            package='rviz2_visuals',
            executable='circle_wave',
            name='circle_wave_node',
            output='screen'
        ),

        # Node to run the circle_path
        Node(
            package='rviz2_visuals',
            executable='circle_path',
            name='circle_path_node',
            output='screen'
        ),

        # Node to run the image_publisher
        Node(
            package='rviz2_visuals',
            executable='image_publisher',
            name='image_publisher_node',
            output='screen'
        ),

        # Node to run RViz2
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config],
            output='screen',
            condition=IfCondition(use_rviz)
        ),
    ])