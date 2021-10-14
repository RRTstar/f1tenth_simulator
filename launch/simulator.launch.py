import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
#from launch_ros.parameters_type import Parameters

def generate_launch_description():
    # use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    param_dir = LaunchConfiguration(
        'param_dir',
         default= os.path.join(
                get_package_share_directory('f1tenth_simulator'),
                'param',
                'params.yaml'))

    return LaunchDescription([
        DeclareLaunchArgument(
            'param_dir',
            default_value = param_dir,
            description = 'param directory'),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            # output='screen'
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            arguments=['$(find f1tenth_simulator)/maps/levine.yaml'],
            output='screen'
        ),
        Node(
            package = 'f1tenth_simulator',
            executable = 'simulator',
            name = 'simulator',
            parameters = [param_dir],
            output = 'screen'
        ),
        Node(
            package= 'f1tenth_simulator',
            executable= 'mux',
            name = 'mux',
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
            name = 'random_walk',
            parameters = [param_dir],
            output = 'screen'
        ),
        Node(
            package= 'f1tenth_simulator',
            executable= 'keyboard',
            name = 'keyboard',
            parameters = [param_dir],
            output = 'screen'
        )
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', 'rviz/simulator.rviz'],
        #     parameters=[{'use_sim_time': use_sim_time}],
        #     output='screen'
        # )
    ])
