"""Arranca la misma escena en offline, observe o control con backend seleccionable."""
from pathlib import Path
from xml.etree import ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.actions import Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def close_gui(event, context):
    if event.returncode and not context.is_shutdown:
        raise RuntimeError(f"RViz terminó con código {event.returncode}. Revise XWayland y OpenGL.")
    return [Shutdown(reason="RViz finalizado")]


def generate_launch_description():
    description = Path(get_package_share_directory("imt342_irb120_description"))
    share = Path(get_package_share_directory("imt342_irb120_bringup"))
    robot = (description / "urdf/irb120.urdf").read_text(encoding="utf-8")
    ghost = ET.fromstring(robot)
    for link in ghost.findall("link"):
        link.set("name", "preview_" + link.attrib["name"])
        for visual in link.findall("visual"):
            for material in visual.findall("material"):
                visual.remove(material)
            material = ET.SubElement(visual, "material", {"name": "preview_cyan"})
            ET.SubElement(material, "color", {"rgba": "0.06 0.68 0.96 1.0"})
    for ref in ghost.findall("./joint/parent") + ghost.findall("./joint/child"):
        ref.set("link", "preview_" + ref.attrib["link"])

    def publisher(name, xml, state, description_topic):
        return Node(
            package="robot_state_publisher", executable="robot_state_publisher", name=name,
            parameters=[{"robot_description": ParameterValue(xml, value_type=str)}],
            remappings=[("/joint_states", state), ("/robot_description", description_topic)],
        )

    return LaunchDescription([
        SetEnvironmentVariable("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST"),
        DeclareLaunchArgument("mode", default_value="offline", choices=["offline", "observe", "control"]),
        DeclareLaunchArgument("backend", default_value="rws",
                              choices=["external", "rws", "rws_trajectory"]),
        DeclareLaunchArgument("backend_config", default_value=str(share / "config/backends.yaml")),
        DeclareLaunchArgument("cell_config", default_value=str(share / "config/cell.yaml")),
        DeclareLaunchArgument("gui", default_value="true"),
        Node(
            package="imt342_irb120_bringup", executable="coordinator.py", output="screen",
            parameters=[LaunchConfiguration("backend_config"),
                        {"mode": LaunchConfiguration("mode"),
                         "backend": LaunchConfiguration("backend"),
                         "cell_config": ParameterValue(LaunchConfiguration("cell_config"), value_type=str),
                         "model_xml": ParameterValue(robot, value_type=str)}],
        ),
        publisher("irb120_actual", robot, "/imt342/actual", "/imt342/robot_description"),
        publisher("irb120_preview", ET.tostring(ghost, encoding="unicode"),
                  "/imt342/preview", "/imt342/preview_description"),
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            arguments=["--frame-id", "base_link", "--child-frame-id", "preview_base_link"],
        ),
        Node(
            package="rviz2", executable="rviz2", condition=IfCondition(LaunchConfiguration("gui")),
            arguments=["-d", str(share / "config/session.rviz")],
            on_exit=close_gui,
        ),
    ])
