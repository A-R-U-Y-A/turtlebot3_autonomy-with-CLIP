from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_turtlebot3_autonomy = FindPackageShare('turtlebot3_autonomy')
    pkg_slam_toolbox = FindPackageShare('slam_toolbox')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    nav2_params = PathJoinSubstitution([pkg_turtlebot3_autonomy, 'config', 'nav2_params.yaml'])
    
    return LaunchDescription([
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        
        # Start office world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([pkg_turtlebot3_autonomy, 'launch', 'office_world_launch.py'])
            ])
        ),
        
        # Start SLAM after 3 seconds
        TimerAction(
            period=3.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        PathJoinSubstitution([pkg_slam_toolbox, 'launch', 'online_async_launch.py'])
                    ]),
                    launch_arguments={'use_sim_time': use_sim_time}.items()
                )
            ]
        ),

        # Start autonomy nodes after 6 seconds
       TimerAction(
           period=6.0,
           actions=[
               # Office semantic mapping node
               Node(
                   package='turtlebot3_autonomy',
                   executable='semantic_node',
                   name='office_semantic_node',
                   parameters=[{'use_sim_time': use_sim_time}],
                   output='screen'
               ),
               
               # Exploration node
               Node(
                   package='turtlebot3_autonomy',
                   executable='exploration_node',
                   name='exploration_node',
                   parameters=[nav2_params, {'use_sim_time': use_sim_time}],
                   output='screen'
               ),
               
               # RRT planner
               Node(
                   package='turtlebot3_autonomy',
                   executable='rrt_planner',
                   name='rrt_planner',
                   parameters=[nav2_params, {'use_sim_time': use_sim_time}],
                   output='screen'
               ),
               
               # Semantic query service
               Node(
                   package='turtlebot3_autonomy',
                   executable='semantic_query_service',
                   name='semantic_query_service',
                   parameters=[{'use_sim_time': use_sim_time}],
                   output='screen'
               )
           ]
       ),
       
       # Start RViz after 8 seconds
       TimerAction(
           period=8.0,
           actions=[
               Node(
                   package='rviz2',
                   executable='rviz2',
                   name='rviz2',
                   parameters=[{'use_sim_time': use_sim_time}],
                   output='screen'
               )
           ]
       )
   ])