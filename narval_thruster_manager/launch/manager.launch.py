from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    narval_thruster_manager = FindPackageShare('narval_thruster_manager')
    narval_description_package = FindPackageShare('narval_description')
    narval_wrench_system = FindPackageShare('narval_wrench_system')

    default_config_path = PathJoinSubstitution([narval_thruster_manager, 'config', 'params.yaml'])
    ld.add_action(DeclareLaunchArgument(name="config", default_value=default_config_path, description="File name of configuration"))

    ld.add_action(Node(
        package='narval_thruster_manager',
        executable="thruster_manager",
        parameters=[LaunchConfiguration('config')],
        output="screen"
    ))

    # ld.add_action(IncludeLaunchDescription(
    #     PathJoinSubstitution([narval_description_package, "launch", "description.launch.py"])))
    
    # ld.add_action(IncludeLaunchDescription(
    #     PathJoinSubstitution([narval_wrench_system, 'launch', 'base.launch.py'])
    # ))

    return ld

