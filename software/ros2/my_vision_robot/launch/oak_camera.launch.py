from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'myrobot'

    # Path to our YAML config (installed by colcon)
    config_file = PathJoinSubstitution([
        FindPackageShare(package_name),
        'config',
        'oak_run.yaml'
    ])

    return LaunchDescription([
        Node(
            package='depthai_ros_driver',
            executable='camera_node',
            name='camera',
            output='screen',
            parameters=[config_file],
            # Add remappings or extra parameters here later if needed
        )
    ])