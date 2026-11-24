"""ROS display of the nominal four-wall cell and its planned TCP paths."""

from math import isfinite
from pathlib import Path

import yaml
from geometry_msgs.msg import Point, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray

from imt342_irb120_kinematics import pose_from_rpy
from imt342_irb120_kinematics.analysis import pose_record

FRAME = "cell_frame"
MESH = "package://imt342_irb120_bringup/config/cell/muro.STL"
CELL_TASK_KINDS = frozenset({"cell_wall", "cell_perimeter", "air_triangle"})


def finite_vector(values, count, name):
    if (not isinstance(values, (tuple, list)) or len(values) != count
            or any(type(value) not in (int, float) or not isfinite(value) for value in values)):
        raise ValueError(f"{name} requiere {count} valores finitos")
    return tuple(float(value) for value in values)


class CellScene:
    def __init__(self, file_path):
        config = yaml.safe_load(Path(file_path).read_text(encoding="utf-8"))
        if not isinstance(config, dict) or type(config.get("geometry_verified")) is not bool:
            raise ValueError("La configuración de celda requiere geometry_verified booleano")
        self.config = config
        self.verified = config["geometry_verified"]
        frame = config["frame"]
        self.xyz = finite_vector(frame["xyz_m"], 3, "Traslación de cell_frame")
        self.rpy = finite_vector(frame["rpy_rad"], 3, "Orientación de cell_frame")
        self.transform = pose_from_rpy(self.xyz, self.rpy)
        walls = config["walls"]
        self.outer = finite_vector(walls["outer_xy_m"], 2, "Dimensiones exteriores")
        self.height, self.thickness = finite_vector(
            [walls["height_m"], walls["thickness_m"]], 2, "Altura y espesor")
        if min(self.height, self.thickness) <= 0 or min(self.outer) <= 2 * self.thickness:
            raise ValueError("La celda requiere cuatro paredes y un interior de tamaño positivo")
        mesh = config["mesh"]
        self.mesh_xyz = finite_vector(mesh["xyz_m"], 3, "Traslación STL")
        mesh_rpy = finite_vector(mesh["rpy_rad"], 3, "Orientación STL")
        self.mesh_xyzw = pose_record(pose_from_rpy(self.mesh_xyz, mesh_rpy))["xyzw"]
        (self.scale,) = finite_vector([mesh["scale_to_m"]], 1, "Escala STL")
        if self.scale <= 0:
            raise ValueError("La escala STL debe ser positiva")

    def static_transform(self, stamp):
        transform = TransformStamped()
        transform.header.frame_id = "base_link"
        transform.header.stamp = stamp
        transform.child_frame_id = FRAME
        v = transform.transform.translation
        v.x, v.y, v.z = self.xyz
        q = transform.transform.rotation
        q.x, q.y, q.z, q.w = pose_record(self.transform)["xyzw"]
        return transform

    @staticmethod
    def walls_markers(stamp):
        marker = Marker(ns="cell_walls", id=0)
        marker.header.frame_id = FRAME
        marker.header.stamp = stamp
        marker.color.r = .18
        marker.color.g = .46
        marker.color.b = .74
        marker.color.a = .34
        return marker

    def markers(self, stamp):
        half_x, half_y = (length / 2 for length in self.outer)
        # Show the supplied mesh at nominal dimensions. With adjusted wall
        # dimensions, display boxes so visual and collision walls still agree.
        if (self.outer, self.height, self.thickness) == ((.8, .8), .2, .05):
            marker = self.walls_markers(stamp)
            marker.type = Marker.MESH_RESOURCE
            marker.mesh_resource = MESH
            marker.mesh_use_embedded_materials = False
            marker.scale.x = marker.scale.y = marker.scale.z = self.scale
            p = marker.pose.position
            p.x, p.y, p.z = self.mesh_xyz
            q = marker.pose.orientation
            q.x, q.y, q.z, q.w = self.mesh_xyzw
            return MarkerArray(markers=[marker])
        t = self.thickness
        boxes = (
            (-half_x + t / 2, 0., t, 2 * half_y),
            (half_x - t / 2, 0., t, 2 * half_y),
            (0., -half_y + t / 2, 2 * (half_x - t), t),
            (0., half_y - t / 2, 2 * (half_x - t), t),
        )
        markers = []
        for index, (x, y, width, depth) in enumerate(boxes):
            marker = self.walls_markers(stamp)
            marker.id = index
            marker.type = Marker.CUBE
            marker.pose.position = Point(x=x, y=y, z=self.height / 2)
            marker.pose.orientation.w = 1.
            marker.scale.x, marker.scale.y, marker.scale.z = width, depth, self.height
            markers.append(marker)
        return MarkerArray(markers=markers)


def task_path_markers(stamp, reference=(), planned=()):
    markers = [Marker(action=Marker.DELETEALL)]
    for index, (path, color, width) in enumerate((
        (reference, (.97, .59, .14), .008), (planned, (.19, .88, .90), .004),
    )):
        if not path:
            continue
        marker = Marker(ns="cell_tcp", id=index, type=Marker.LINE_STRIP, action=Marker.ADD)
        marker.header.frame_id = "base_link"
        marker.header.stamp = stamp
        marker.pose.orientation.w = 1.
        marker.scale.x = width
        marker.color.r, marker.color.g, marker.color.b = color
        marker.color.a = .94
        marker.points = [Point(x=float(point[0]), y=float(point[1]), z=float(point[2])) for point in path]
        markers.append(marker)
    return MarkerArray(markers=markers)
