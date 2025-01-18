from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
        ld = LaunchDescription()
        ld.add_action(DeclareLaunchArgument('urdf_file_name',
                                            description="file of the urdf description", 
                                            default_value="narval_base.urdf.xacro"))
        
        package_dir = FindPackageShare("narval_description")
        urdf_file = PathJoinSubstitution([package_dir, LaunchConfiguration('urdf_file_name')])

        robot_description_content = ParameterValue(Command(command=['xacro ', urdf_file]), value_type=str)

        robot_state_publisher_node = Node(package="robot_state_publisher",
                                          executable="robot_state_publisher",
                                          parameters=[{
                                                  "robot_description": robot_description_content,
                                          }])
        
        ld.add_action(robot_state_publisher_node)
        return ld