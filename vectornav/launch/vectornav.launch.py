import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():

    this_dir = get_package_share_directory('vectornav')

    # Launch argument (default = existing config)
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(this_dir, 'config', 'vectornav.yaml'),
        description='Path to VectorNav config file'
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robot namespace'
    )

    config_file = LaunchConfiguration('config_file')
    namespace = LaunchConfiguration('namespace')
    
    # Vectornav
    start_vectornav_cmd = Node(
        package='vectornav', 
        executable='vectornav',
        name='vectornav',
        output='screen',
        namespace=namespace,
        parameters=[config_file])
    
    start_vectornav_sensor_msgs_cmd = Node(
        package='vectornav', 
        executable='vn_sensor_msgs',
        name='vn_sensor_msgs',
        output='screen',
        namespace=namespace,
        parameters=[config_file])

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(config_arg)
    ld.add_action(namespace_arg)
    ld.add_action(start_vectornav_cmd)
    ld.add_action(start_vectornav_sensor_msgs_cmd)

    return ld
