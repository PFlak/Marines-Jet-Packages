from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
        description_share = FindPackageShare('narval_description')

        default_config_path = PathJoinSubstitution([description_share, 'config', 'params.yaml'])
        default_rviz_config_path = PathJoinSubstitution([description_share, 'rviz', 'urdf.rviz'])

        config_arg = DeclareLaunchArgument(
                name="config",
                default_value=default_config_path,
                description="Path to the configuration file for the narval state publisher"
        )

        rviz_config_arg = DeclareLaunchArgument(
                name="rviz_config",
                default_value=default_rviz_config_path,
                description='Path to the RViz2 configuration file'
        )

        narval_state_publisher_node = Node(
                package='narval_description',
                executable='narval_state_publisher',
                parameters=[LaunchConfiguration('config')]
        )

        rviz_node = Node(
                package='rviz2',
                executable='rviz2',
                output='screen',
                arguments=['-d', LaunchConfiguration('rviz_config')]
        )

        return LaunchDescription([
                config_arg,
                rviz_config_arg,
                narval_state_publisher_node,
                rviz_node
        ])
