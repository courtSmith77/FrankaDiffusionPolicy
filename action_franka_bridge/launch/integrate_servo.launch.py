from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(name="use_fake_hardware", default_value="true",
                                  description="whether or not to use fake hardware."),
            DeclareLaunchArgument(name="collect_data", default_value="false",
                                  description="whether or not to collect data during this trial."),
            DeclareLaunchArgument(name="robot_ip", default_value="dont-care",
                                  description="IP address of the robot"),
            DeclareLaunchArgument(name="frequency", default_value="10.0",
                                  description="the frequency of the nodes (action_franka_bridge and franka_servo)."),
            DeclareLaunchArgument(name="launch_moveit", default_value="true",
                                  description="launch the move it node with the current settings"),
            DeclareLaunchArgument(name="launch_controllers", default_value="true",
                                  description="launch all controller nodes with the current settings"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([PathJoinSubstitution(
                    [FindPackageShare('franka_moveit_config'), 'launch', 'moveit.launch.py'])]),
                launch_arguments={'use_fake_hardware': LaunchConfiguration("use_fake_hardware"),
                                  'robot_ip': LaunchConfiguration("robot_ip")}.items(),
                condition=IfCondition(LaunchConfiguration("launch_moveit"))
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([PathJoinSubstitution(
                    [FindPackageShare('commandmode'), 'launch', 'camera.launch.py'])]),
                condition=IfCondition(LaunchConfiguration("launch_controllers"))
            ),
            Node(
                package="action_franka_bridge",
                executable="action_franka_bridge",
                output="screen",
                condition=IfCondition(LaunchConfiguration("launch_controllers")),
                parameters=[{"frequency": LaunchConfiguration("frequency")}],
            ),
            Node(
                package="action_franka_bridge",
                executable="data_collection",
                output="screen",
                condition=IfCondition(LaunchConfiguration("collect_data")),
            ),
            Node(
                package="action_franka_bridge",
                executable="model_input_publisher",
                output="screen",
                condition=IfCondition(LaunchConfiguration("launch_controllers")),
            ),
        ]
    )