from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('arm_controller')

    urdf_path_arg = DeclareLaunchArgument(
        'urdf_path',
        default_value='',
        description='Path to URDF file'
    )

    arm_controller_node = Node(
        package='arm_controller',
        executable='arm_controller_node',
        name='arm_controller_node',
        parameters=[{
            'urdf_path': LaunchConfiguration('urdf_path'),
            'control_rate': 100.0,
            'trajectory_duration': 3.0
        }],
        output='screen'
    )

    return LaunchDescription([
        urdf_path_arg,
        arm_controller_node
    ])
