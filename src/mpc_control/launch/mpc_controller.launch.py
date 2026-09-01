import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    mpc_share = get_package_share_directory('mpc_control')
    description_share = get_package_share_directory('rebot_description')

    return LaunchDescription([
        Node(
            package='mpc_control',
            executable='mpc_controller_node',
            name='mpc_controller',
            output='screen',
            parameters=[
                os.path.join(mpc_share, 'config', 'mpc_controller.yaml'),
                {
                    'task_file': os.path.join(
                        mpc_share, 'config', 'task.info'),
                    'urdf_file': os.path.join(
                        description_share,
                        'urdf',
                        'reBot-DevArm_fixend.urdf'),
                },
            ],
        ),
    ])
