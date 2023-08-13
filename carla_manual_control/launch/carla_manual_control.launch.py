import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='role_name',
            default_value='ego_vehicle'
        ),
        launch.actions.DeclareLaunchArgument(
            name='objects_definition_file',
            default_value=get_package_share_directory(
                'carla_spawn_objects') + '/config/objects.json'
        ),
        launch_ros.actions.Node(
            package='carla_manual_control',
            executable='carla_manual_control',
            name=['carla_manual_control_', launch.substitutions.LaunchConfiguration('role_name')],
            output='screen',
            emulate_tty=True,
            parameters=[
                {
                    'role_name': launch.substitutions.LaunchConfiguration('role_name'),
                    'objects_definition_file': launch.substitutions.LaunchConfiguration('objects_definition_file')
                }
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
