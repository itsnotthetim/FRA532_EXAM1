import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define package directories and configuration file paths
    my_ws_dir = get_package_share_directory('dd_controller')
    nav2_dir = get_package_share_directory('nav2_bringup')
    
    my_map_file = os.path.join(my_ws_dir, 'map', 'map.yaml')
    params_file = os.path.join(my_ws_dir, 'config', 'test.yaml')
    rviz_config_file = os.path.join(my_ws_dir, 'rviz', 'rviz_cfg.rviz')
    
    # Include the localization launch from nav2
    nav2_localization_launch_file = os.path.join(nav2_dir, 'launch', 'localization_launch.py')
    nav2_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_localization_launch_file),
        launch_arguments={
            'map': my_map_file,
            'params_file': params_file,
            'use_sim_time': 'true',
            'autostart': 'true',
        }.items()
    )
    
    # RViz for Visualization (optional)
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )
    
    # Assemble the Launch Description
    ld = LaunchDescription()
    ld.add_action(nav2_localization)

    ld.add_action(rviz2_node)
    
    return ld
