from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('doppler_icp')
    default_param_file = os.path.join(pkg_share, 'config', 'dicp.yaml')

    param_file_arg = DeclareLaunchArgument(
        'param_file',
        default_value=default_param_file,
        description='Path to parameter file'
    )

    return LaunchDescription([
        param_file_arg,
        Node(
            package='doppler_icp',
            executable='doppler_icp_node',
            name='dicp_node',
            output='screen',
            parameters=[LaunchConfiguration('param_file')]
        )
    ])
