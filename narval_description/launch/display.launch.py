# display.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # Declare launch arguments
    narval_description_path = FindPackageShare('narval_description')

    # Define paths for URDF and RViz configuration
    default_model_path = PathJoinSubstitution([narval_description_path, 'urdf', 'full.urdf'])
    default_rviz_config_path = PathJoinSubstitution([narval_description_path, 'rviz', 'narval.rviz'])

    # Declare launch arguments
    ld.add_action(DeclareLaunchArgument('model', default_value=default_model_path,
                                        description='Path to the robot URDF file'))
    ld.add_action(DeclareLaunchArgument('rvizconfig', default_value=default_rviz_config_path,
                                        description='Path to RViz config file'))

    # Robot state publisher node
    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': LaunchConfiguration('model')}]
    ))

    # Launch RViz2 with the given configuration file
    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')]
    ))

    return ld