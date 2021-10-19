import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import ThisLaunchFileDir

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='False')
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('f1tenth_simulator'),
            'maps',
            'berlin.yaml'))

    param_dir = LaunchConfiguration(
        'param_dir',
         default= os.path.join(
                get_package_share_directory('f1tenth_simulator'),
                'param',
                'params.yaml'))

    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    
    rviz_config_dir = os.path.join(
        get_package_share_directory('f1tenth_simulator'),
        'rviz',
        'simulator.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value = map_dir,
            description = 'Full path to map file to load'
        ),
        DeclareLaunchArgument(
            'param_file',
            default_value = param_dir,
            description = 'param directory'
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/racecar_model.launch.py']
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                # 'autostart': True,
                'params_file': param_dir}.items(),
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            # output='screen'
        ),
        Node(
            package= 'f1tenth_simulator',
            executable= 'mux',
            name = 'mux_controller',
            parameters = [param_dir],
            output = 'screen'
        ),        
        Node(
            package = 'f1tenth_simulator',
            executable = 'simulator',
            name = 'racecar_simulator',
            parameters = [param_dir],
            output = 'screen'
        ),
        Node(
            package= 'f1tenth_simulator',
            executable= 'behavior_controller',
            name = 'behavior_controller',
            parameters = [param_dir],
            output = 'screen'
        ),
        Node(
            package= 'f1tenth_simulator',
            executable= 'random_walk',
            name = 'random_walker',
            parameters = [param_dir],
            output = 'screen'
        ),
        Node(
            package= 'f1tenth_simulator',
            executable= 'keyboard',
            name = 'keyboard',
            parameters = [param_dir],
            output = 'screen'
        ),   
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        )
    ])
