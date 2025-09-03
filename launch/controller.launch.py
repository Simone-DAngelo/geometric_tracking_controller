from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('geometric_tracking_controller')
    
    # Declare launch arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robot namespace for topic remapping'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'controller_params.yaml'),
        description='Path to controller configuration file'
    )
    
    # Controller node
    controller_node = Node(
        package='geometric_tracking_controller',
        executable='geometric_tracking_controller',
        name='geometric_tracking_controller',
        namespace=LaunchConfiguration('namespace'),
        parameters=[LaunchConfiguration('config_file'),
                   {'namespace': LaunchConfiguration('namespace'),
                    'use_sim_time': True}],
        output='screen',
        emulate_tty=True
    )
    
    return LaunchDescription([
        namespace_arg,
        config_file_arg,
        controller_node
    ])
