from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    name_arg = DeclareLaunchArgument(
        'name',
        default_value='rm0',
        description='Namespace for the node'
    )

    # Use the launch argument as the namespace
    namespace = LaunchConfiguration('name')

    return LaunchDescription([
        name_arg,
        Node(
            package='robo_roomba',
            executable='open_loop_controller', 
            namespace=namespace,
            name='open_loop_controller',
            output='screen'
        )
    ])
