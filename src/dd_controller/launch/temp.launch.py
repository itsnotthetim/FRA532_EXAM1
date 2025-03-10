import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    my_ws_dir = get_package_share_directory('dd_controller')
    nav2_dir = get_package_share_directory('nav2_bringup')

    my_map_file = os.path.join(my_ws_dir, 'map', 'map.yaml')
    params_file = os.path.join(my_ws_dir, 'config', 'test.yaml')
    rviz_config_file = os.path.join(my_ws_dir, 'rviz', 'navigation.rviz')

    # Path to the localization launch file in the nav2 package
    nav2_localization_launch_file = os.path.join(
        nav2_dir,
        'launch',
        'localization_launch.py'
    )

    
    # Include the nav2 localization launch file
    nav2_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_localization_launch_file),
        launch_arguments={
            'map': my_map_file,  # Replace with the path to your map file
            'params_file': params_file,  # Replace with the path to your params file
            'use_sim_time': 'true',  # Set to 'true' if using simulation time
            'autostart': 'true',  # Automatically start the lifecycle nodes
        }.items()
    )

    # Global Costmap
    global_costmap = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        namespace='global_costmap',  # Namespace for global costmap
        name='global_costmap',
        output='screen',
        parameters=[params_file],
        
    )

    # Local Costmap
    local_costmap = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        namespace='local_costmap',  # Namespace for local costmap
        name='local_costmap',
        output='screen',
        parameters=[params_file]
    )

    # Lifecycle Manager for Costmaps
    start_lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_costmap_filters',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': [ 'filter_mask_server', 'costmap_filter_info_server']}] # 'global_costmap/global_costmap', 'local_costmap/local_costmap',
    )

    # start_map_server_cmd = Node(
    #         package='nav2_map_server',
    #         executable='map_server',
    #         name='filter_mask_server',
    #         output='screen',
    #         emulate_tty=True,
    #         parameters=[params_file])

    # start_costmap_filter_info_server_cmd = Node(
    #         package='nav2_map_server',
    #         executable='costmap_filter_info_server',
    #         name='costmap_filter_info_server',
    #         output='screen',
    #         emulate_tty=True,
    #         parameters=[params_file])

    # RViz2 Node
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],  # Load the RViz2 configuration file
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the nav2 localization launch
    ld.add_action(nav2_localization)

    # Add lifecycle manager and costmaps
    # ld.add_action(start_lifecycle_manager_cmd)
    ld.add_action(global_costmap)
    ld.add_action(local_costmap)

    # Add RViz2 for visualization
    ld.add_action(rviz2_node)

    # ld.add_action(start_map_server_cmd)
    # ld.add_action(start_costmap_filter_info_server_cmd)

    return ld