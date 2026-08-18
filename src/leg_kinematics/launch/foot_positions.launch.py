"""Start the foot position node against one robot's configuration.

    ros2 launch leg_kinematics foot_positions.launch.py
    ros2 launch leg_kinematics foot_positions.launch.py config:=go2
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = LaunchConfiguration('config')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value='mini_cheetah',
            description='Robot configuration in config/, without the .yaml suffix.',
        ),

        Node(
            package='leg_kinematics',
            executable='foot_position_node',
            # Must match the top-level key in the YAML, or no parameters load.
            name='foot_position_node',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('leg_kinematics'), 'config', [config, '.yaml'],
                ]),
            ],
        ),
    ])
