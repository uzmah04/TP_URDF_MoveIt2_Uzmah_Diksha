from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
import os

def generate_launch_description():
    # Path to the main xacro file
    xacro_file = os.path.join(
        FindPackageShare('robotic_arm').find('robotic_arm'),
        'xacro',
        'robotic_arm.urdf.xacro'
    )
    
    # Path to RViz config file
    rviz_config_file = os.path.join(
        FindPackageShare('robotic_arm').find('robotic_arm'),
        'config',
        'xacro_config.rviz'
    )
    
    # Process xacro to URDF and explicitly mark as string
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )
    
    # Robot state publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )
    
    # Joint state publisher GUI (for testing)
    jsp_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )
    
    # RViz with config file (if it exists)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
        output='screen'
    )
    
    return LaunchDescription([
        rsp_node,
        jsp_node,
        rviz_node
    ])