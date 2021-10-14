import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='joy',
            executable='joy_node',
            name='f1tenth_simulator',
            # output='screen'
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            arguments=['$(find f1tenth_simulator)/maps/levine.yaml'],
            output='screen'
        ),
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', 'rviz/simulator.rviz'],
        #     parameters=[{'use_sim_time': use_sim_time}],
        #     output='screen'
        # )
    ])
