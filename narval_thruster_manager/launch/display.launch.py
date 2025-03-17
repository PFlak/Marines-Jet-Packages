from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # Define package share paths
    thruster_manager_share = FindPackageShare('narval_thruster_manager')
    description_share = FindPackageShare('narval_description')
    wrench_system_share = FindPackageShare('narval_wrench_system')

    # Define default configuration paths
    default_config_path = PathJoinSubstitution([thruster_manager_share, 'config', 'params.yaml'])
    default_rviz_config_path = PathJoinSubstitution([thruster_manager_share, 'rviz', 'thruster.rviz'])

    # Launch arguments
    config_arg = DeclareLaunchArgument(
        name="config",
        default_value=default_config_path,
        description="Path to the configuration file for the thruster manager"
    )
    rviz_config_arg = DeclareLaunchArgument(
        name="rviz_config",
        default_value=default_rviz_config_path,
        description="Path to the RViz2 configuration file"
    )

    # Nodes and launch inclusions
    thruster_manager_node = Node(
        package='narval_thruster_manager',
        executable="thruster_manager",
        parameters=[LaunchConfiguration('config')],
        output="screen"
    )

    narval_state_publisher_node = Node(
        package='narval_description',
        executable='narval_state_publisher',
        parameters=[LaunchConfiguration('config')]
    )

    wrench_system_launch = IncludeLaunchDescription(
        launch_description_source=PathJoinSubstitution([wrench_system_share, 'launch', 'base.launch.py']),
        launch_arguments={
            "config": LaunchConfiguration('config')
        }.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        output='screen'
    )


    # Timed actions
    description_timer = TimerAction(period=1.0, actions=[narval_state_publisher_node])
    rviz_timer = TimerAction(period=1.0, actions=[rviz_node])

    # Combine into launch description
    return LaunchDescription([
        config_arg,
        rviz_config_arg,
        thruster_manager_node,
        description_timer,
        wrench_system_launch,
        rviz_timer
    ])