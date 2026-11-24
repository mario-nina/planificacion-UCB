"""Geometry of the four cell walls and conservative URDF collision checks."""

from dataclasses import dataclass, replace
from functools import lru_cache
from math import hypot, isfinite
from pathlib import Path
from struct import iter_unpack, unpack_from
from xml.etree.ElementTree import fromstring

from .model import Model, _dot, _multiply, _origin, _rotation, _vector
from .pose import pose_from_rpy, validate_pose


_EPS = 1e-10
_MAX_DEPTH = 12
_ZERO = (0.0, 0.0, 0.0)


def _translation(pose):
    return tuple(row[3] for row in pose[:3])


def _transform_point(pose, point):
    return tuple(_dot(row[:3], point) + row[3] for row in pose[:3])


def _inverse_transform(pose):
    axes = tuple(zip(*(row[:3] for row in pose[:3])))
    origin = _translation(pose)
    return tuple(axis + (-_dot(axis, origin),) for axis in axes) + ((0., 0., 0., 1.),)


@dataclass(frozen=True)
class Wall:
    """Axis-aligned solid in cell_frame, including its boundary."""

    name: str
    lower: tuple
    upper: tuple

    @property
    def center(self):
        return tuple((a + b) / 2 for a, b in zip(self.lower, self.upper))

    @property
    def half_size(self):
        return tuple((b - a) / 2 for a, b in zip(self.lower, self.upper))

    def distance(self, xyz):
        return hypot(*(max(lo - pos, 0., pos - hi) for lo, pos, hi in zip(self.lower, xyz, self.upper)))


@dataclass(frozen=True)
class CollisionReport:
    ok: bool
    reason: str = ""
    link: str | None = None
    wall: str | None = None
    segment_index: int | None = None
    fraction: float | None = None
    configurations_checked: int = 0


@dataclass(frozen=True)
class _Body:
    name: str
    link_index: int
    local_pose: tuple
    half_size: tuple
    radii: tuple


@lru_cache(maxsize=32)
def _stl_bounds(path):
    raw = path.read_bytes()
    if len(raw) >= 84 and 84 + 50 * unpack_from("<I", raw, 80)[0] == len(raw):
        vertices = [record[start:start+3] for record in iter_unpack("<12fH", raw[84:]) for start in (3, 6, 9)]
    else:
        vertices = [tuple(map(float, fields[1:])) for row in raw.decode("ascii").splitlines()
                    if (fields := row.split()) and fields[0] == "vertex"]
    if (not vertices or len(vertices) % 3 != 0
            or any(len(vertex) != 3 or not all(map(isfinite, vertex)) for vertex in vertices)):
        raise ValueError(f"STL de colisión inválido: {path.name}")
    axes = tuple(zip(*vertices))
    return tuple(map(min, axes)), tuple(map(max, axes))


def _geometry_box(collision, share):
    geometry = collision.find("geometry")
    if geometry is None:
        raise ValueError("Falta la geometría de colisión URDF")
    mesh = geometry.find("mesh")
    box = geometry.find("box")
    if mesh is not None:
        uri = mesh.attrib["filename"]
        prefix = "package://imt342_irb120_description/"
        if not uri.startswith(prefix):
            raise ValueError(f"URI de malla de colisión fuera de description: {uri}")
        relative = Path(uri[len(prefix):])
        if relative.is_absolute() or ".." in relative.parts:
            raise ValueError(f"Ruta de malla de colisión fuera de description: {uri}")
        path = share / relative
        scale = _vector(mesh.get("scale", "1 1 1").split(), 3)
        if min(scale) <= 0:
            raise ValueError("La escala URDF de colisión debe ser positiva")
        lo, hi = _stl_bounds(path)
        center = tuple((a + b) * s / 2 for a, b, s in zip(lo, hi, scale))
        half = tuple((b - a) * s / 2 for a, b, s in zip(lo, hi, scale))
    elif box is not None:
        center = _ZERO
        half = tuple(value / 2 for value in _vector(box.attrib["size"].split(), 3))
    else:
        raise ValueError("Geometría de colisión URDF no soportada")
    if min(half) <= 0 or not all(map(isfinite, center + half)):
        raise ValueError("El volumen de colisión URDF debe tener dimensiones positivas y finitas")
    return _multiply(_origin(collision.find("origin")), pose_from_rpy(center, _ZERO)), half


def _intersects(pose, half_size, wall, margin):
    """Separating-axis test between the transformed body box and one wall."""
    rotation = tuple(row[:3] for row in pose[:3])
    absolute = tuple(tuple(abs(x) + _EPS for x in row) for row in rotation)
    delta = tuple(a - b for a, b in zip(_translation(pose), wall.center))
    half_wall = tuple(size + margin + _EPS for size in wall.half_size)
    # The three world axes are also the inexpensive broad-phase test.
    if any(abs(delta[i]) > half_wall[i] + _dot(absolute[i], half_size) for i in range(3)):
        return False
    if any(abs(_dot(delta, axis)) > half_size[j] + _dot(half_wall, projections)
           for j, (axis, projections) in enumerate(zip(zip(*rotation), zip(*absolute)))):
        return False
    # The remaining separating axes are the nine pairwise edge cross products.
    for i in range(3):
        i1, i2 = (i + 1) % 3, (i + 2) % 3
        for j in range(3):
            j1, j2 = (j + 1) % 3, (j + 2) % 3
            wall_extent = half_wall[i1] * absolute[i2][j] + half_wall[i2] * absolute[i1][j]
            body_extent = half_size[j1] * absolute[i][j2] + half_size[j2] * absolute[i][j1]
            offset = abs(delta[i2] * rotation[i1][j] - delta[i1] * rotation[i2][j])
            if offset > wall_extent + body_extent:
                return False
    return True


class Cell:
    """Center-origin wall geometry and collision envelopes in the original URDF chain."""

    @classmethod
    def from_urdf(cls, config, robot_xml, description_share):
        return cls(config, robot_xml, description_share)

    def __init__(self, config, robot_xml, description_share):
        self.geometry_verified = config.get("geometry_verified") is True
        frame = config["frame"]
        self.xyz_m = _vector(frame["xyz_m"], 3)
        self.rpy_rad = _vector(frame["rpy_rad"], 3)
        self._base_from_cell = pose_from_rpy(self.xyz_m, self.rpy_rad)
        self._cell_from_base = _inverse_transform(self._base_from_cell)

        walls = config["walls"]
        outside_x, outside_y = (length / 2 for length in _vector(walls["outer_xy_m"], 2))
        self.wall_top_z, thickness = _vector((walls["height_m"], walls["thickness_m"]), 2)
        self.wall_thickness = thickness
        if not 0 < thickness < min(outside_x, outside_y) or self.wall_top_z <= 0:
            raise ValueError("Dimensiones no válidas para los cuatro muros de la celda")
        inside_x, inside_y = outside_x - thickness, outside_y - thickness
        self.inner_bounds = ((-inside_x, inside_x), (-inside_y, inside_y))
        height = self.wall_top_z
        self.walls = (
            Wall("x_min", (-outside_x, -outside_y, 0.), (-inside_x, outside_y, height)),
            Wall("x_max", (inside_x, -outside_y, 0.), (outside_x, outside_y, height)),
            Wall("y_min", (-inside_x, -outside_y, 0.), (inside_x, -inside_y, height)),
            Wall("y_max", (-inside_x, inside_y, 0.), (inside_x, outside_y, height)),
        )
        self._walls_by_name = {wall.name: wall for wall in self.walls}
        collision = config.get("collision", {})
        self.clearance_m, self.joint_margin_rad = _vector(
            (collision.get("clearance_m", 0.), collision.get("joint_margin_rad", 0.)), 2,
        )
        if min(self.clearance_m, self.joint_margin_rad) < 0.:
            raise ValueError("Los márgenes de colisión deben ser no negativos")

        root = fromstring(robot_xml)
        self._model = Model.from_xml(robot_xml)
        self._chain = self._model._chain
        by_name = {joint.get("name"): joint.find("child").get("link") for joint in root.findall("joint")}
        links = ("base_link",) + tuple(by_name[joint.name] for joint in self._chain)
        self._flange = links.index("dh_frame_6")
        self._wrist = links.index("link_5")
        self._limits = self._model.limits
        self._joint_lengths = tuple(hypot(*_translation(joint.origin)) for joint in self._chain)
        self._moving = tuple(i for i, joint in enumerate(self._chain) if joint.axis is not None)
        share = Path(description_share).resolve()
        bodies = []
        for link in root.findall("link"):
            name = link.get("name")
            for collision in link.findall("collision"):
                if name not in links:
                    raise ValueError(f"La geometría de colisión de {name} no pertenece a la cadena del ABB")
                body_pose, half = _geometry_box(collision, share)
                bodies.append(self._body(name, links.index(name), body_pose, half))
        if not bodies:
            raise ValueError("El URDF no aporta geometrías de colisión del robot")
        self._bodies = tuple(bodies)
        self._flange_point = self._body("flange", self._flange, pose_from_rpy(_ZERO, _ZERO), _ZERO)
        self._tcp = self._body("tcp", links.index("tcp"), pose_from_rpy(_ZERO, _ZERO), _ZERO)

    def _body(self, name, link_index, local_pose, half):
        # Distance from a revolute axis to any body point is at most the sum
        # of subsequent fixed link offsets and the body's local bounding radius.
        radius = hypot(*_translation(local_pose)) + hypot(*half)
        radii = tuple(sum(self._joint_lengths[joint_index + 1:link_index]) + radius
                      if joint_index < link_index else 0. for joint_index in self._moving)
        return _Body(name, link_index, local_pose, half, radii)

    def to_base(self, xyz):
        return _transform_point(self._base_from_cell, _vector(xyz, 3))

    def to_cell(self, xyz):
        return _transform_point(self._cell_from_base, _vector(xyz, 3))

    def pose_to_base(self, matrix):
        return _multiply(self._base_from_cell, validate_pose(matrix))

    def wall_distance(self, name, xyz_base):
        return self._walls_by_name[name].distance(self.to_cell(xyz_base))

    def first_joint_axis(self):
        """Axis and origin of J1 expressed in base_link."""
        _, axes, origins = self._model._evaluate((0.,) * 6)
        return axes[0], origins[0]

    def wrist_tcp_bound(self, tool_offset=None):
        """Maximum distance from the J5 wrist center to the selected TCP."""
        if tool_offset is None:
            return sum(self._joint_lengths[self._wrist:self._tcp.link_index])
        return sum(self._joint_lengths[self._wrist:self._flange]) + hypot(*_translation(validate_pose(tool_offset)))

    def _poses(self, q, bodies):
        angles = iter(q)
        chain_poses = [self._cell_from_base]
        for joint in self._chain:
            pose = _multiply(chain_poses[-1], joint.origin)
            if joint.axis is not None:
                pose = _multiply(pose, _rotation(joint.axis, next(angles)))
            chain_poses.append(pose)
        return tuple(_multiply(chain_poses[body.link_index], body.local_pose) for body in bodies)

    def _collision(self, q, bodies, static_margins, rates=None, half_interval=0.):
        poses = self._poses(q, bodies)
        possible = None
        for body, pose, static_margin in zip(bodies, poses, static_margins):
            for wall in self.walls:
                if _intersects(pose, body.half_size, wall, static_margin):
                    return (body.name, wall.name), None
            if rates is not None and possible is None:
                margin = _dot(body.radii, rates) * half_interval
                if margin > 0.:
                    for wall in self.walls:
                        if _intersects(pose, body.half_size, wall, static_margin + margin):
                            possible = body.name, wall.name
                            break
        return None, possible

    def _with_tcp(self, tool_offset):
        if tool_offset is None:
            return self._bodies + (self._flange_point, self._tcp,)
        tcp = self._body("tcp_active", self._flange, validate_pose(tool_offset), _ZERO)
        return self._bodies + (self._flange_point, tcp,)

    def _within_limits(self, q):
        return all(lo <= angle <= hi for angle, (lo, hi) in zip(q, self._limits))

    def check_pose(self, q, tool_offset=None, *, joint_margin_rad=None):
        return self.check_segment(q, q, tool_offset=tool_offset, joint_margin_rad=joint_margin_rad)

    def check_segment(self, start, goal, tool_offset=None, *, joint_margin_rad=None):
        """Check the whole synchronized segment by bounding between tested configurations.

        Both supported profiles interpolate a common scalar from 0 to 1. For
        an interval about its midpoint, each body point moves by at most the
        sum of (downstream joint radius × joint angle deviation). If the box
        at the midpoint misses every wall expanded by that bound, the whole
        interval is clear. Otherwise subdivide or reject an unresolved interval.
        """
        start, goal = _vector(start, 6), _vector(goal, 6)
        if not self._within_limits(start) or not self._within_limits(goal):
            return CollisionReport(False, "Inicio u objetivo fuera de límites articulares URDF")
        bodies = self._with_tcp(tool_offset)
        angular_margin = self.joint_margin_rad
        if joint_margin_rad is not None:
            joint_margin_rad, = _vector((joint_margin_rad,), 1)
            if joint_margin_rad < 0.:
                raise ValueError("El margen articular debe ser no negativo")
            angular_margin = max(angular_margin, joint_margin_rad)
        static_margins = tuple(self.clearance_m + angular_margin * sum(body.radii) for body in bodies)
        rates = tuple(abs(b - a) for a, b in zip(start, goal))
        configurations_checked = 0

        def failure(location, fraction, *, unresolved=False):
            link, wall = location
            reason = (f"No se pudo confirmar separación de {link} y {wall} entre muestras"
                      if unresolved else f"Posible colisión de {link} con {wall} en la envolvente URDF")
            return CollisionReport(False, reason, link, wall, fraction=fraction,
                                   configurations_checked=configurations_checked)

        endpoints = ((0., start),) if start == goal else ((0., start), (1., goal))
        for fraction, q in endpoints:
            configurations_checked += 1
            collision, _ = self._collision(q, bodies, static_margins)
            if collision is not None:
                return failure(collision, fraction)
        if start == goal:
            return CollisionReport(True, configurations_checked=configurations_checked)

        intervals = [(0., 1., 0)]
        while intervals:
            lo, hi, depth = intervals.pop()
            mid = (lo + hi) / 2
            q = tuple(a + mid * (b - a) for a, b in zip(start, goal))
            configurations_checked += 1
            collision, possible = self._collision(q, bodies, static_margins, rates, (hi - lo) / 2)
            if collision is not None:
                return failure(collision, mid)
            if possible is not None:
                if depth >= _MAX_DEPTH:
                    return failure(possible, mid, unresolved=True)
                intervals.extend(((mid, hi, depth + 1), (lo, mid, depth + 1)))

        return CollisionReport(True, configurations_checked=configurations_checked)

    def check_trajectory(self, points, trajectory, *, joint_margin_rad=None):
        segments = getattr(trajectory, "segments", (trajectory,))
        checked = 0
        previous_goal = None
        for index, segment in enumerate(segments):
            if previous_goal is not None and tuple(segment.start) != previous_goal:
                return CollisionReport(False, "Hay una discontinuidad entre tramos de la trayectoria",
                                       segment_index=index, configurations_checked=checked)
            if segment.profile not in {"lspb", "quintic"}:
                raise ValueError("Perfil sin cota de colisión continua para la celda")
            if getattr(segment, "linear_cartesian", False):
                # The continuous reference is not the endpoint joint chord.
                # Bound each cubic between dense IK nodes conservatively.
                chords = segment.collision_chords()
            else:
                chords = ((segment.start, segment.goal, 0.),)
            for start, goal, extra in chords:
                margin = joint_margin_rad
                if getattr(segment, "linear_cartesian", False):
                    if margin is not None and (not isfinite(margin) or margin < 0):
                        raise ValueError("El margen articular debe ser no negativo y finito")
                    margin = max(self.joint_margin_rad, margin or 0.) + extra
                result = self.check_segment(start, goal, tool_offset=points.tool_offset,
                                            joint_margin_rad=margin)
                checked += result.configurations_checked
                if not result.ok:
                    return replace(result, segment_index=index, configurations_checked=checked)
            previous_goal = tuple(segment.goal)
        if not segments:
            raise ValueError("Trayectoria sin tramos para verificar")
        return CollisionReport(True, configurations_checked=checked)
