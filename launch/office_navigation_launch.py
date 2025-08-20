#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_turtlebot3_autonomy = FindPackageShare('turtlebot3_autonomy')
    pkg_turtlebot3_gazebo = FindPackageShare('turtlebot3_gazebo')
    pkg_nav2_bringup = FindPackageShare('nav2_bringup')
    
    # Parameters
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_file = LaunchConfiguration('map', default=PathJoinSubstitution([pkg_turtlebot3_autonomy, 'worlds', 'map', 'map.yaml']))
    world_file = PathJoinSubstitution([pkg_turtlebot3_autonomy, 'worlds', 'service.world'])
    nav2_params = PathJoinSubstitution([pkg_turtlebot3_autonomy, 'config', 'nav2_params.yaml'])
    rviz_config = PathJoinSubstitution([pkg_turtlebot3_autonomy, 'config', 'navigation.rviz'])
    
    return LaunchDescription([
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', 
                             PathJoinSubstitution([pkg_turtlebot3_autonomy, 'worlds', 'models'])),
        
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map', 
                            default_value=PathJoinSubstitution([pkg_turtlebot3_autonomy, 'worlds', 'map', 'map.yaml']),
                            description='Path to saved map yaml file'),
        
        # 1. Start Gazebo with office world (same environment)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([pkg_turtlebot3_gazebo, 'launch', 'turtlebot3_house.launch.py'])
            ]),
            launch_arguments={
                'world': world_file
            }.items()
        ),
        
        # 2. Start Navigation with saved map
        TimerAction(
            period=3.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        PathJoinSubstitution([pkg_nav2_bringup, 'launch', 'bringup_launch.py'])
                    ]),
                    launch_arguments={
                        'use_sim_time': use_sim_time,
                        'map': map_file,
                        'params_file': nav2_params
                    }.items()
                )
            ]
        ),
        
        # 3. Start RRT Path Planner
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='turtlebot3_autonomy',
                    executable='rrt_planner',
                    name='office_rrt_planner',
                    parameters=[
                        nav2_params,
                        {
                            'use_sim_time': use_sim_time,
                            'planning_timeout': 30.0,
                            'step_size': 0.3,
                            'goal_tolerance': 0.4,
                            'max_iterations': 10000
                        }
                    ],
                    output='screen'
                )
            ]
        ),
        
        # 4. Start Semantic Location Manager (loads saved coordinates)
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='turtlebot3_autonomy',
                    executable='semantic_location_manager',
                    name='semantic_location_manager',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'database_file': '~/office_semantic_map.db',
                        'coordinate_file': '~/office_exploration_data.json',
                        'enable_numbered_queries': True  # Enable 1,2,3,4... queries
                    }],
                    output='screen'
                )
            ]
        ),
        
        # 5. Start Semantic Navigation Controller
        TimerAction(
            period=7.0,
            actions=[
                Node(
                    package='turtlebot3_autonomy',
                    executable='semantic_navigator',
                    name='semantic_navigator',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'navigation_timeout': 60.0,
                        'path_planning_service': '/compute_path_to_pose',
                        'follow_path_action': '/follow_path'
                    }],
                    output='screen'
                )
            ]
        ),
        
        # 6. Start Interactive Query Interface
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='turtlebot3_autonomy',
                    executable='interactive_query_interface',
                    name='query_interface',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'show_menu_on_start': True,
                        'auto_execute_goals': True
                    }],
                    output='screen'
                )
            ]
        ),
        
        # 7. Start RViz for navigation visualization
        TimerAction(
            period=9.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_config],
                    parameters=[{'use_sim_time': use_sim_time}],
                    output='screen'
                )
            ]
        ),
        
        # 8. Start Status Monitor
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='turtlebot3_autonomy',
                    executable='navigation_status_monitor',
                    name='navigation_monitor',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'status_publish_rate': 2.0
                    }],
                    output='screen'
                )
            ]
        )
    ])

if __name__ == '__main__':
    generate_launch_description()