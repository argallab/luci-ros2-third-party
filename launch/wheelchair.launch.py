from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        ExecuteProcess(
            cmd=['ros2', 'run', 'luci_grpc_interface', 'grpc_interface_node', '-a', '192.168.8.203'],
            output='screen',
            name='grpc_node'
        ),

        Node(
            package='luci_transforms',
            executable='quickie_500m_tf_node',
            name='quickie_500m_tf_node'
        ),
    ])