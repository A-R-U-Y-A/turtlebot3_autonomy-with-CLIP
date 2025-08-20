#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction, OpaqueFunction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_turtlebot3_autonomy = FindPackageShare('turtlebot3_autonomy')
    pkg_turtlebot3_gazebo = FindPackageShare('turtlebot3_gazebo')
    pkg_slam_toolbox = FindPackageShare('slam_toolbox')
    pkg_nav2_bringup = FindPackageShare('nav2_bringup')
    
    # Parameters
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file = PathJoinSubstitution([FindPackageShare('turtlebot3_gazebo'), 'worlds', 'turtlebot3_house.world'])
    #world_file = PathJoinSubstitution([FindPackageShare('turtlebot3_gazebo'), 'worlds', 'service.world'])
    nav2_params = PathJoinSubstitution([pkg_turtlebot3_autonomy, 'config', 'nav2_params.yaml'])
    rviz_config = PathJoinSubstitution([pkg_turtlebot3_autonomy, 'config', 'exploration.rviz'])
    
    return LaunchDescription([
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger_cam'),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', 
                             [PathJoinSubstitution([pkg_turtlebot3_autonomy, 'models']), ':', PathJoinSubstitution([pkg_turtlebot3_autonomy, 'worlds', 'models']), ':/home/aru/.gazebo/models']),
        SetEnvironmentVariable('GAZEBO_RESOURCE_PATH', 
                             '/usr/share/gazebo-11:/usr/share/gazebo/'),
        
        # Performance optimization environment variables for WSL2
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '0'),  
        SetEnvironmentVariable('MESA_GL_VERSION_OVERRIDE', '3.3'),
        SetEnvironmentVariable('QT_X11_NO_MITSHM', '1'),
        SetEnvironmentVariable('GAZEBO_VERBOSE', '1'),
        # SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1'),
        # SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia'),
        
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        
        # 1. Start Gazebo with office world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gzserver.launch.py'])
            ]),
            launch_arguments={
                'world': world_file,
                'verbose': 'false',  # Reduce verbose output for performance
                #'physics': 'ode',
                #'paused': 'false',
                #'gui': 'false'  # Start without GUI, gzclient will handle visualization
            }.items()
        ),
        
        # Start Gazebo client
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gzclient.launch.py'])
            ])
        ),
        
        # Spawn TurtleBot3 with camera after Gazebo is ready
        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        PathJoinSubstitution([FindPackageShare('turtlebot3_gazebo'), 'launch', 'robot_state_publisher.launch.py'])
                    ]),
                    launch_arguments={
                        'use_sim_time': use_sim_time
                    }.items()
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        PathJoinSubstitution([FindPackageShare('turtlebot3_gazebo'), 'launch', 'spawn_turtlebot3.launch.py'])
                    ]),
                    launch_arguments={
                        'use_sim_time': use_sim_time,
                        # 'x_pose': '0.0',
                        # 'y_pose': '20.0',
                        'x_pose': '0.0',
                        'y_pose': '0.0',
                        # 'z_pose': '0.0',
                        #'yaw': '-1.5'      # Same orientation
                    }.items()
                )
            ]
        ),
        
        # 2. Start SLAM after robot spawns and starts publishing odom
        TimerAction(
            period=10.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        PathJoinSubstitution([pkg_slam_toolbox, 'launch', 'online_async_launch.py'])
                    ]),
                    launch_arguments={
                        'use_sim_time': use_sim_time,
                    }.items()
                )
            ]
        ),
        
        # 3. Start Navigation2 (for obstacle avoidance during exploration)
        TimerAction(
            period=12.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        PathJoinSubstitution([pkg_nav2_bringup, 'launch', 'navigation_launch.py'])
                    ]),
                    launch_arguments={
                        'use_sim_time': use_sim_time,
                        'params_file': nav2_params
                    }.items()
                )
            ]
        ),
        
        # 4. Start Office Semantic Mapping (identifies rooms and saves coordinates)
        TimerAction(
            period=9.0,
            actions=[
                Node(
                    package='turtlebot3_autonomy',
                    executable='semantic_node',
                    name='office_semantic_node',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'capture_interval': 3.0,  # Analyze every 3 seconds
                        'min_confidence': 0.4,    # Lower threshold for exploration
                        'use_clip': False,        # Disable CLIP for now due to packaging issues
                        'lidar_weight': 0.6,      # LIDAR confidence weight
                        'vision_weight': 0.4      # Vision confidence weight
                    }],
                    output='screen'
                )
            ]
        ),
        
        # 5. Start Exploration Node with obstacle avoidance
        TimerAction(
            period=11.0,
            actions=[
                Node(
                    package='turtlebot3_autonomy',
                    executable='exploration_node',
                    name='office_explorer',
                    parameters=[
                        nav2_params,
                        {
                            'use_sim_time': use_sim_time,
                            'exploration_timeout': 600.0,  # 10 minutes max
                            'frontier_min_size': 8,        # Smaller frontiers for detailed exploration
                            'max_exploration_distance': 15.0,  # Larger office coverage
                            'safety_distance': 0.3         # Safe distance from obstacles
                        }
                    ],
                    output='screen'
                )
            ]
        ),
        
        # 6. Start Data Logger (saves coordinates and paths)
        TimerAction(
            period=11.0,
            actions=[
                Node(
                    package='turtlebot3_autonomy',
                    executable='exploration_data_logger',
                    name='exploration_data_logger',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'output_file': '~/office_exploration_data.json',
                        'auto_save_interval': 30.0  # Save every 30 seconds
                    }],
                    output='screen'
                )
            ]
        ),
        
        # 7. Start RViz for visualization
        TimerAction(
            period=12.0,
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
        
        # 8. Start exploration completion monitor
        TimerAction(
            period=14.0,
            actions=[
                Node(
                    package='turtlebot3_autonomy',
                    executable='exploration_monitor',
                    name='exploration_monitor',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'completion_check_interval': 10.0,
                        'auto_save_on_completion': True
                    }],
                    output='screen'
                )
            ]
        )
    ])

if __name__ == '__main__':
    generate_launch_description()