from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    ip = LaunchConfiguration('ip')
    ip_launch_arg = DeclareLaunchArgument('ip', description='LUCI Chair IP address')

    return LaunchDescription([
        ip_launch_arg,

        ExecuteProcess(
            cmd=['ros2', 'run', 'luci_grpc_interface', 'grpc_interface_node', '-a', ip],
            output='screen',
            name='grpc_node'
        ),

        Node(
            package='luci_transforms',
            executable='quickie_500m_tf_node',
            name='quickie_500m_tf_node'
        ),
    ])  