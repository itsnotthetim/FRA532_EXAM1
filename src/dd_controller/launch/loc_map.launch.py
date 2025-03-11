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
    
    # RViz for Visualization
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )

    # Global Planner (A* Algorithm)
    global_planner_node = Node(
        package='dd_controller',
        executable='astar_global.py',
        name='global_planner_node',
        output='screen'
    )

    # Local Planner (DWA Algorithm) with Parameters
    sbmpo_controller_node = Node(
        package='dd_controller',
        executable='sbmpo_controller.py',
        name='sbmpo_controller',
        output='screen',
        parameters=[{
            'horizon': 2.5,
            'dt': 0.1,
            'num_samples_v': 7,
            'num_samples_w': 7,
            'v_max': 0.5,
            'w_max': 1.0,
            'obstacle_weight': 15.0,
            'path_weight': 0.2,
            'forward_reward': 0.1,
            'obstacle_threshold': 30,
            'penalty_factor': 300,
            'emergency_ttc': 0.5, 
            'emergency_stop_distance': 0.2,
        }]
    )

    # Global Costmap Node
    global_costmap_node = Node(
        package='dd_controller',
        executable='global_costmap_node.py',
        name='global_costmap_node',
        output='screen'
    )

    # Local Costmap Node
    local_costmap_node = Node(
        package='dd_controller',
        executable='local_costmap_node.py',
        name='local_costmap_node',
        output='screen',
        parameters=[{
            'inflation_radius': 0.2,
            'local_size': 6.0,
            'resolution': 0.05,
            'publish_rate' : 100.0,
        }]
    )
    
    # Assemble the Launch Description
    ld = LaunchDescription()
    ld.add_action(nav2_localization)
    ld.add_action(rviz2_node)
    ld.add_action(global_planner_node)
    ld.add_action(global_costmap_node)
    ld.add_action(local_costmap_node)
    # ld.add_action(sbmpo_controller_node)
    return ld
