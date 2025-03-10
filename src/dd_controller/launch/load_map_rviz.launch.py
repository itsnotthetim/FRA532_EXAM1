from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Get the path to the map file
    my_dir = get_package_share_directory('dd_controller')

    my_map_file = os.path.join(my_dir, 'map', 'map.yaml')
    params_file = os.path.join(my_dir, 'config', 'param.yaml')
    rviz_config_file = os.path.join(my_dir, 'rviz', 'rviz_cfg.rviz')

    return LaunchDescription([

        # Start the Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': my_map_file},]
        ),

        # Start AMCL
        # Node(
        #     package='nav2_amcl',
        #     executable='amcl',
        #     name='amcl',
        #     output='screen',
        #     parameters=[{'use_sim_time': True}, params_file]  # Set to False if not using simulation
        # ),

        # Start the Lifecycle Manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            parameters=[{'use_sim_time': True,
                        'autostart': True,
                        'node_names': ['map_server']}] # , 'amcl'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],  # Load a saved RViz configuration
            parameters=[{'use_sim_time': True}]
        )
    ])