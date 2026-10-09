"""Publish Baby Jerry's model and optionally provide local joint sliders."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ROS finds this directory in the sourced workspace, independent of checkout path.
    assets = Path(get_package_share_directory("ghost_visualization_assets"))
    urdf_path = assets / "baby_jerry" / "urdf" / "baby_jerry.urdf"
    description = urdf_path.read_text(encoding="utf-8")
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="false",
                              description="Use wheel sliders instead of fixed zero joint positions."),
        DeclareLaunchArgument("publish_joint_states", default_value="true",
                              description="Disable when another node supplies actual joint feedback."),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="baby_jerry_state_publisher",
            parameters=[{"robot_description": description}],
            output="screen",
        ),
        # The group condition keeps both preview publishers off during external feedback.
        _joint_publishers(description),
    ])


def _joint_publishers(description):
    from launch.actions import GroupAction
    return GroupAction(
        condition=IfCondition(LaunchConfiguration("publish_joint_states")),
        actions=[
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="baby_jerry_preview_joint_publisher",
                condition=UnlessCondition(LaunchConfiguration("gui")),
                parameters=[{"robot_description": description}],
                output="screen",
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="baby_jerry_preview_joint_gui",
                condition=IfCondition(LaunchConfiguration("gui")),
                parameters=[{"robot_description": description}],
                output="screen",
            ),
        ],
    )
