import launch
import launch_ros.actions
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import ExecutableInPackage, FindPackageShare


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            name='rviz_config_file',
            default_value=[get_package_share_directory('luci_third_party'),
                           '/config/luci_view.rviz'],
            description='Full path to the RVIZ config file to use'),

        Node(package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher'),

        Node(package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                    {"robot_description":
                        Command([ExecutableInPackage("xacro", "xacro"), " ",
                                PathJoinSubstitution(
                                [FindPackageShare("luci_third_party"), "urdf/awl_description.xacro"])])}]),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d',  LaunchConfiguration('rviz_config_file')],
            output='screen'),

    ])