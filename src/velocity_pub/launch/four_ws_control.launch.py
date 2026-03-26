from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # Create a variable to grab the value of the launch argument
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',  # Changed from false to true
            description='Use simulation (Gazebo) clock if true'),
        
        Node(
            package="joy",
            executable="joy_node",
            # Pass the time parameter to the joy node
            parameters=[{'use_sim_time': use_sim_time}] 
        ),
        
        Node(
            package='velocity_pub', 
            executable='robot_control_ai.py', 
            output='screen',
            # Pass the time parameter to your python script
            parameters=[{'use_sim_time': use_sim_time}] 
        ),
    ])