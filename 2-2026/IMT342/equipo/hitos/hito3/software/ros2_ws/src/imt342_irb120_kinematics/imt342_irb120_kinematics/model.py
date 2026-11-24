"""Pose y Jacobiano espacial del TCP a partir de la cadena URDF."""

from dataclasses import dataclass
from math import atan2, cos, hypot, isfinite, pi, sin, sqrt
from pathlib import Path
from xml.etree import ElementTree


JOINT_NAMES = tuple(f"joint_{index}" for index in range(1, 7))
_IDENTITY = ((1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1))


def _vector(values, size):
    vector = tuple(float(value) for value in values)
    if len(vector) != size or not all(map(isfinite, vector)):
        raise ValueError(f"Se esperaban {size} números finitos")
    return vector


def _numbers(text):
    return _vector(text.split(), 3)


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def _cross(a, b):
    return (a[1]*b[2] - a[2]*b[1], a[2]*b[0] - a[0]*b[2], a[0]*b[1] - a[1]*b[0])


def _multiply(a, b):
    columns = tuple(zip(*b))
    return tuple(tuple(_dot(row, col) for col in columns) for row in a)


def _rotation(axis, angle):
    x, y, z = axis
    c, s = cos(angle), sin(angle)
    t = 1 - c
    return (
        (c + x*x*t, x*y*t - z*s, x*z*t + y*s, 0),
        (y*x*t + z*s, c + y*y*t, y*z*t - x*s, 0),
        (z*x*t - y*s, z*y*t + x*s, c + z*z*t, 0),
        (0, 0, 0, 1),
    )


def _xyz_rpy(xyz, rpy):
    roll, pitch, yaw = rpy
    rotation = _multiply(
        _multiply(_rotation((0, 0, 1), yaw), _rotation((0, 1, 0), pitch)),
        _rotation((1, 0, 0), roll),
    )
    return tuple(rotation[i][:3] + (xyz[i],) for i in range(3)) + (_IDENTITY[3],)


def _origin(element):
    attributes = {} if element is None else element.attrib
    xyz = _numbers(attributes.get("xyz", "0 0 0"))
    rpy = _numbers(attributes.get("rpy", "0 0 0"))
    return _xyz_rpy(xyz, rpy)


def _rotation_error(target, actual):
    # log(R_target R_actual^T), expresado en la base de la cadena.
    relative = tuple(tuple(_dot(a[:3], b[:3]) for b in actual[:3]) for a in target[:3])
    skew = (
        (relative[2][1] - relative[1][2]) / 2,
        (relative[0][2] - relative[2][0]) / 2,
        (relative[1][0] - relative[0][1]) / 2,
    )
    sine = hypot(*skew)
    cosine = (sum(relative[i][i] for i in range(3)) - 1) / 2
    angle = atan2(sine, cosine)
    if angle < 1e-9:
        return skew
    if pi - angle > 1e-5:
        return tuple(angle * value / sine for value in skew)
    # En una media vuelta el término antisimétrico se anula.
    index = max(range(3), key=lambda i: relative[i][i])
    denominator = 1 - cosine
    component = sqrt(max(0, (relative[index][index] - cosine) / denominator))
    axis = [0.0] * 3
    axis[index] = component
    for other in range(3):
        if other != index:
            axis[other] = (
                (relative[other][index] + relative[index][other]) / (2 * denominator * component)
            )
    sign = -1 if _dot(axis, skew) < 0 else 1
    return tuple(sign * angle * value for value in axis)


def _pose_error(target, actual):
    translation = tuple(target[i][3] - actual[i][3] for i in range(3))
    return translation + _rotation_error(target, actual)


def _cholesky_solve(matrix, rhs):
    size = len(rhs)
    lower = [[0.0] * size for _ in rhs]
    for i in range(size):
        for j in range(i + 1):
            residual = matrix[i][j] - _dot(lower[i][:j], lower[j][:j])
            lower[i][j] = sqrt(residual) if i == j else residual / lower[j][j]
    forward = []
    for i, value in enumerate(rhs):
        forward.append((value - _dot(lower[i][:i], forward)) / lower[i][i])
    solution = [0.0] * size
    for i in reversed(range(size)):
        column_tail = tuple(lower[j][i] for j in range(i + 1, size))
        solution[i] = (forward[i] - _dot(column_tail, solution[i+1:])) / lower[i][i]
    return solution


def _svd(matrix, *, right_vectors=False):
    # Jacobi unilateral sobre las columnas. Evita elevar al cuadrado la condición.
    columns = list(zip(*matrix))
    right = (list(tuple(float(i == j) for i in range(len(columns)))
                  for j in range(len(columns))) if right_vectors else None)
    for _ in range(40):
        changed = False
        for i, first in enumerate(columns):
            for j in range(i + 1, len(columns)):
                second = columns[j]
                alpha = _dot(first, first)
                beta = _dot(second, second)
                cross = _dot(first, second)
                if abs(cross) <= max(1e-28, 1e-14 * sqrt(alpha * beta)):
                    continue
                tau = (beta - alpha) / (2 * cross)
                tangent = (1 if tau >= 0 else -1) / (abs(tau) + hypot(1, tau))
                cosine = 1 / hypot(1, tangent)
                sine = cosine * tangent
                first, columns[j] = (
                    tuple(cosine * a - sine * b for a, b in zip(first, second)),
                    tuple(sine * a + cosine * b for a, b in zip(first, second)),
                )
                columns[i] = first
                if right is not None:
                    right[i], right[j] = (
                        tuple(cosine * a - sine * b for a, b in zip(right[i], right[j])),
                        tuple(sine * a + cosine * b for a, b in zip(right[i], right[j])),
                    )
                changed = True
        if not changed:
            break
    values = tuple(hypot(*column) for column in columns)
    order = sorted(range(len(columns)), key=values.__getitem__, reverse=True)
    sorted_values = tuple(values[index] for index in order)
    if right is not None:
        return sorted_values, tuple(right[index] for index in order)
    return sorted_values


@dataclass(frozen=True)
class _Joint:
    name: str
    origin: tuple
    axis: tuple | None
    limits: tuple | None


@dataclass(frozen=True)
class IKResult:
    q: tuple
    converged: bool
    position_error_m: float
    orientation_error_rad: float
    iterations: int
    source: str = "numerical"


@dataclass(frozen=True)
class Model:
    """Cadena revoluta ordenada de base_link a tcp."""

    _chain: tuple

    @classmethod
    def from_file(cls, path):
        return cls.from_xml(Path(path).read_text(encoding="utf-8"))

    @classmethod
    def from_xml(cls, xml):
        root = ElementTree.fromstring(xml)
        if root.tag != "robot":
            raise ValueError("El documento debe contener un robot URDF")
        by_child = {}
        for joint in root.findall("joint"):
            child = joint.find("child").attrib["link"]
            if child in by_child:
                raise ValueError(f"Más de un padre para {child}")
            by_child[child] = joint
        chain = []
        visited = set()
        link = "tcp"
        while link != "base_link":
            if link in visited or link not in by_child:
                raise ValueError("No existe una cadena de base_link a tcp")
            visited.add(link)
            joint = by_child[link]
            name = joint.attrib["name"]
            kind = joint.attrib["type"]
            axis = limits = None
            if kind == "revolute":
                axis_element = joint.find("axis")
                axis = (1, 0, 0) if axis_element is None else _numbers(axis_element.attrib["xyz"])
                magnitude = hypot(*axis)
                if magnitude == 0:
                    raise ValueError(f"Eje nulo en {name}")
                axis = tuple(value / magnitude for value in axis)
                limit = joint.find("limit")
                if limit is None:
                    raise ValueError(f"Faltan límites en {name}")
                limits = _vector((limit.attrib["lower"], limit.attrib["upper"]), 2)
                if limits[0] >= limits[1]:
                    raise ValueError(f"Límites no crecientes en {name}")
            elif kind != "fixed":
                raise ValueError(f"Tipo de articulación no soportado: {kind}")
            chain.append(_Joint(name, _origin(joint.find("origin")), axis, limits))
            link = joint.find("parent").attrib["link"]
        chain.reverse()
        if tuple(joint.name for joint in chain if joint.axis is not None) != JOINT_NAMES:
            raise ValueError("La cadena debe contener joint_1 a joint_6 en ese orden")
        return cls(tuple(chain))

    @property
    def limits(self):
        """Pares (mínimo, máximo) en radianes en el orden ROS."""
        return tuple(joint.limits for joint in self._chain if joint.axis is not None)

    def _evaluate(self, q):
        angles = iter(_vector(q, 6))
        transform = _IDENTITY
        axes = []
        positions = []
        for joint in self._chain:
            transform = _multiply(transform, joint.origin)
            if joint.axis is None:
                continue
            axes.append(tuple(_dot(row[:3], joint.axis) for row in transform[:3]))
            positions.append(tuple(row[3] for row in transform[:3]))
            transform = _multiply(transform, _rotation(joint.axis, next(angles)))
        return transform, axes, positions

    def fk(self, q):
        """Matriz homogénea de base_link a tcp."""
        return self._evaluate(q)[0]

    def jacobian(self, q):
        """Filas lineales (m/rad) y angulares (rad/rad) expresadas en la base."""
        transform, axes, positions = self._evaluate(q)
        columns = []
        for axis, origin in zip(axes, positions):
            delta = tuple(transform[i][3] - origin[i] for i in range(3))
            columns.append(_cross(axis, delta) + axis)
        return tuple(zip(*columns))

    def singular_values(self, q, characteristic_length_m=0.5):
        """Valores singulares de J con filas lineales divididas por L."""
        if not isfinite(characteristic_length_m) or characteristic_length_m <= 0:
            raise ValueError("La longitud característica debe ser positiva y finita")
        jacobian = self.jacobian(q)
        scaled = tuple(
            tuple(value / characteristic_length_m for value in row) if i < 3 else row
            for i, row in enumerate(jacobian)
        )
        return _svd(scaled)

    def jacobian_fd(self, q, step=1e-5):
        """Central differences in the base frame, with a spatial SO(3) logarithm."""
        q = _vector(q, 6)
        if not isfinite(step) or step <= 0:
            raise ValueError("El paso debe ser positivo y finito")
        columns = []
        for i, angle in enumerate(q):
            plus = self.fk(q[:i] + (angle + step,) + q[i+1:])
            minus = self.fk(q[:i] + (angle - step,) + q[i+1:])
            columns.append(tuple(value / (2 * step) for value in _pose_error(plus, minus)))
        return tuple(zip(*columns))

    def ik_pieper(self, target, seed=None):
        """Return bounded, FK-verified analytic branches ordered by seed distance."""
        from .pieper import solve

        return solve(self, target, seed)

    def ik_multiseed(self, target, seeds, **options):
        """Return distinct converged numerical results in seed order."""
        from .pose import validate_pose

        target = validate_pose(target)
        results = []
        for seed in seeds:
            result = self.ik(target, seed, **options)
            if result.converged and not any(
                hypot(*(a - b for a, b in zip(result.q, previous.q))) <= 1e-5
                for previous in results
            ):
                results.append(result)
        return tuple(results)

    def ik(self, target, seed, *, max_iterations=150, position_tolerance_m=1e-6,
           orientation_tolerance_rad=1e-6):
        """Solución local por mínimos cuadrados amortiguados y límites URDF."""
        from .pose import validate_pose

        target = validate_pose(target)
        if type(max_iterations) is not int or max_iterations < 0:
            raise ValueError("max_iterations debe ser un entero no negativo")
        if not all(isfinite(tol) and tol > 0 for tol in
                   (position_tolerance_m, orientation_tolerance_rad)):
            raise ValueError("Las tolerancias deben ser positivas y finitas")

        limits = self.limits

        def project(q):
            return tuple(min(high, max(low, angle)) for angle, (low, high) in zip(q, limits))

        # La escala evita sumar números en metros y radianes sin un criterio.
        orientation_length = 0.5
        weights = (1, 1, 1, orientation_length, orientation_length, orientation_length)
        q = project(_vector(seed, 6))
        error = _pose_error(target, self.fk(q))
        for iteration in range(max_iterations + 1):
            position_error = hypot(*error[:3])
            orientation_error = hypot(*error[3:])
            converged = (position_error <= position_tolerance_m
                         and orientation_error <= orientation_tolerance_rad)
            if converged or iteration == max_iterations:
                break
            weighted = tuple(value * weight for value, weight in zip(error, weights))
            jacobian = tuple(
                tuple(weight * value for value in row)
                for row, weight in zip(self.jacobian(q), weights)
            )
            columns = tuple(zip(*jacobian))
            normal = tuple(tuple(_dot(a, b) + (0.002**2 if i == j else 0)
                                 for j, b in enumerate(columns)) for i, a in enumerate(columns))
            delta = _cholesky_solve(normal, tuple(_dot(col, weighted) for col in columns))
            scale = min(1.0, 0.3 / max(hypot(*delta), 1e-12))
            cost = _dot(weighted, weighted)
            for _ in range(12):
                candidate = project(tuple(angle + scale * step for angle, step in zip(q, delta)))
                trial_error = _pose_error(target, self.fk(candidate))
                trial_cost = sum((value * weight)**2 for value, weight in zip(trial_error, weights))
                if trial_cost < cost:
                    q, error = candidate, trial_error
                    break
                scale *= 0.5
            else:
                break
        return IKResult(q, converged, position_error, orientation_error, iteration)
