import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("irb120_description")
    urdf_path = os.path.join(package_share, "urdf", "irb120.urdf")

    with open(urdf_path, "r", encoding="utf-8") as urdf_file:
        robot_description = urdf_file.read()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
    )

    shutdown_on_rviz_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=rviz,
            on_exit=[
                EmitEvent(
                    event=Shutdown(reason="RViz closed")
                )
            ],
        )
    )

    return LaunchDescription(
        [
            robot_state_publisher,
            joint_state_publisher_gui,
            rviz,
            shutdown_on_rviz_exit,
        ]
    )
