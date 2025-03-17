import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from launch_ros.actions import Node as LaunchNode
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import subprocess
import os

class NarvalStatePublisher(Node):
    def __init__(self):
        super().__init__('n_state_publisher')
        self.declare_parameter('robot_name', 'bluerov2')
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        self.package_dir = get_package_share_directory('narval_description')
        self.urdf_file_name = f"{self.robot_name}.urdf.xacro"
        self.urdf_file_path = os.path.join(self.package_dir, 'xacro', self.urdf_file_name)
        
        self.perform_xacro()
        self.launch_robot_state_publisher()

    def perform_xacro(self):
        command = ['xacro', self.urdf_file_path]
        result = subprocess.run(command, capture_output=True, text=True)
        if result.returncode != 0:
            self.get_logger().error(f"Failed to run xacro: {result.stderr}")
        else:
            self.robot_description = result.stdout

    def launch_robot_state_publisher(self):
        from launch import LaunchDescription
        from launch_ros.actions import Node as LaunchNode

        robot_state_publisher_node = LaunchNode(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': self.robot_description}]
        )

        ld = LaunchDescription()
        ld.add_action(robot_state_publisher_node)

        from launch import LaunchService
        ls = LaunchService()
        ls.include_launch_description(ld)
        ls.run()

def main(args=None):
    rclpy.init(args=args)
    node = NarvalStatePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()