from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'turtlebot3_autonomy'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.py')),
        # Configuration files
        (os.path.join('share', package_name, 'config'), 
         glob('config/*.yaml') + glob('config/*.rviz')),
        # World files for Gazebo simulation
        (os.path.join('share', package_name, 'worlds'), 
         glob('worlds/*.world')),
        # Model files for Gazebo (worlds/models)
        *[(os.path.join('share', package_name, os.path.dirname(f)), [f]) 
          for f in glob('worlds/models/**/*', recursive=True) if os.path.isfile(f)],
        # Custom robot models (models/)
        *[(os.path.join('share', package_name, os.path.dirname(f)), [f]) 
          for f in glob('models/**/*', recursive=True) if os.path.isfile(f)],
        # Media files for Gazebo (meshes, materials, textures)
        *[(os.path.join('share', package_name, os.path.dirname(f)), [f]) 
          for f in glob('worlds/media/**/*', recursive=True) if os.path.isfile(f)],
        # Maps
        (os.path.join('share', package_name, 'maps'), 
         glob('maps/*.yaml') + glob('maps/*.pgm')),
        # URDF and robot description files
        (os.path.join('share', package_name, 'urdf'), 
         glob('urdf/*.urdf') + glob('urdf/*.xacro')),
        # Documentation
        (os.path.join('share', package_name, 'docs'), 
         glob('docs/*.md') + glob('docs/*.rst')),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'numpy',
        'geometry_msgs',
        'nav_msgs',
        'std_msgs',
        'sensor_msgs',
        'tf2_ros',
        'tf2_geometry_msgs',
        'nav2_msgs',
        'action_msgs',
        'diagnostic_msgs',
        'std_srvs',
        'cv_bridge',
        'image_transport',
        'builtin_interfaces',
    ],
    zip_safe=True,
    maintainer='Aru',
    maintainer_email='amruyassar6c@gmail.com',
    description='Turtlebot3 Semantic Autonomy Simulation',
    license='Apache License 2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'exploration_node = turtlebot3_autonomy.exploration.exploration_node:main',
            'semantic_node = turtlebot3_autonomy.semantic_mapping.semantic_node:main',
            'semantic_query_service = turtlebot3_autonomy.semantic_mapping.semantic_query_service:main',
            'rrt_planner = turtlebot3_autonomy.path_planning.rrt_planner:main',
            'semantic_navigator = turtlebot3_autonomy.navigation.semantic_navigator:main',
            'semantic_location_manager = turtlebot3_autonomy.navigation.semantic_location_manager:main',
            'navigation_status_monitor = turtlebot3_autonomy.navigation.navigation_status_monitor:main',
            'interactive_query_interface = turtlebot3_autonomy.semantic_mapping.interactive_query_interface:main',
            'exploration_data_logger = turtlebot3_autonomy.exploration.exploration_data_logger:main',
            'exploration_monitor = turtlebot3_autonomy.exploration.exploration_monitor:main'
        ],
    },
)