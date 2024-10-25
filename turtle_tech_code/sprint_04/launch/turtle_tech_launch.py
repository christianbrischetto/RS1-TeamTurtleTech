from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        # Set the TURTLEBOT3_MODEL environment variable
        SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value='waffle_pi'),

        # Include the external launch file from the turtlebot3_gazebo package
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch', 'turtle_tech.launch.py')
            )
        ),

        # Start the rgbDetect node
        Node(
            package='sprint_04',
            executable='rgbDetect',
            name='rgb_detect_node',
            output='screen',
            parameters=[]  # Add if you have any parameter files to load
        )
    ])