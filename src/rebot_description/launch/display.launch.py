import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = get_package_share_directory(
        'rebot_description'
    )

    urdf_file = os.path.join(
        pkg_share,
        'urdf',
        'reBot_B601_DM_with_gripper.urdf'
    )

    rviz_file = os.path.join(
        pkg_share,
        'rviz',
        'rebotWithMaker.rviz'
    )

    # 读取 URDF
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        # =========================
        # 1. robot_state_publisher
        # =========================
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {
                    'robot_description': robot_description
                }
            ]
        ),

        # =========================
        # 3. Interactive Marker
        # =========================
        Node(
            package='rebot_description',
            executable='target_marker',
            name='target_marker',
            output='screen'
        ),

        # =========================
        # 4. RViz
        # =========================
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=[
                '-d',
                rviz_file
            ],
            output='screen'
        ),
    ])