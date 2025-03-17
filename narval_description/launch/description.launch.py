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

        config_arg = DeclareLaunchArgument(
                name="config",
                default_value=default_config_path,
                description="Path to the configuration file for the narval state publisher"
        )

        narval_state_publisher_node = Node(
                package='narval_description',
                executable='narval_state_publisher',
                parameters=[LaunchConfiguration('config')]
        )

        return LaunchDescription([
                config_arg,
                narval_state_publisher_node
        ])
