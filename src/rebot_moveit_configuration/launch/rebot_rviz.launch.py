from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            "rebot_B601_DM_with_gripper",
            package_name="rebot_moveit_configuration"
        ).to_moveit_configs()
    )

    return LaunchDescription([
        # robot state publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[
                moveit_config.robot_description
            ],
            output="screen"
        ),

        # move_group
        # 只负责规划
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
            ],
            output="screen"
        ),

        # RViz
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=[
                "-d",
                moveit_config.package_path
                + "/config/moveit.rviz"
            ],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
            ],
            output="screen"
        )

    ])