import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    urdf_path = Path(LaunchConfiguration("urdf").perform(context)).expanduser().resolve()
    if not urdf_path.exists():
        raise RuntimeError(f"URDF file not found: {urdf_path}")

    robot_description = urdf_path.read_text(encoding="utf-8")
    pkg_share = get_package_share_directory("urdf_xacro_tuner")
    rviz_config = os.path.join(pkg_share, "rviz", "preview_link.rviz")

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="urdf_xacro_tuner",
            executable="robot_description_file_publisher",
            parameters=[{"urdf_path": str(urdf_path)}],
        ),
        Node(
            package="urdf_xacro_tuner",
            executable="inertia_preview_marker",
            parameters=[{"urdf_path": str(urdf_path), "frame_id": "base_link", "link_name": "base_link"}],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("urdf"),
            OpaqueFunction(function=launch_setup),
        ]
    )

