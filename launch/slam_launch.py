import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directories
    turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    slam_toolbox = get_package_share_directory('slam_toolbox')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time if true'),
            
        # Start Gazebo simulation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(turtlebot3_gazebo, 'launch', 'turtlebot3_world.launch.py')
            ])
        ),
        
        # Start SLAM
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(slam_toolbox, 'launch', 'online_async_launch.py')
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
            }.items()
        ),
        
        # Start exploration node
        Node(
            package='turtlebot3_autonomy',
            executable='exploration_node',
            name='exploration_node',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
        
        # Start RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(get_package_share_directory('turtlebot3_autonomy'), 'config', 'exploration.rviz')],
            parameters=[{'use_sim_time': use_sim_time}]
        )
    ])