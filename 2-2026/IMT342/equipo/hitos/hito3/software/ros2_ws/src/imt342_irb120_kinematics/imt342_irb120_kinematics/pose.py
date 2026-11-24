"""Rigid poses in metres and radians with extrinsic XYZ RPY and XYZW quaternions."""

from math import atan2, hypot, sqrt

from .model import _IDENTITY, _cross, _dot, _vector, _xyz_rpy


def validate_pose(target):
    target = tuple(_vector(row, 4) for row in target)
    if len(target) != 4 or target[3] != _IDENTITY[3]:
        raise ValueError("Se esperaba una matriz homogénea 4×4")
    rotation = tuple(row[:3] for row in target[:3])
    if (any(abs(_dot(a, b) - (i == j)) > 1e-6
            for i, a in enumerate(rotation) for j, b in enumerate(rotation))
            or abs(_dot(rotation[0], _cross(rotation[1], rotation[2])) - 1) > 1e-6):
        raise ValueError("La orientación objetivo debe pertenecer a SO(3)")
    return target


def pose_from_rpy(xyz, rpy):
    """Build Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    return _xyz_rpy(_vector(xyz, 3), _vector(rpy, 3))


def pose_to_rpy(target):
    """Return (xyz, rpy). Choose roll=0 at gimbal lock."""
    matrix = validate_pose(target)
    cosine = hypot(matrix[0][0], matrix[1][0])
    pitch = atan2(-matrix[2][0], cosine)
    if cosine > 1e-12:
        roll = atan2(matrix[2][1], matrix[2][2])
        yaw = atan2(matrix[1][0], matrix[0][0])
    else:
        roll = 0.0
        yaw = atan2(-matrix[0][1], matrix[1][1])
    return tuple(row[3] for row in matrix[:3]), (roll, pitch, yaw)


def pose_from_quaternion(xyz, xyzw):
    """Normalize a finite nonzero quaternion before constructing its rotation."""
    px, py, pz = _vector(xyz, 3)
    quaternion = _vector(xyzw, 4)
    scale = max(map(abs, quaternion))
    if scale == 0:
        raise ValueError("El cuaternión debe ser no nulo")
    scaled = tuple(value / scale for value in quaternion)
    length = hypot(*scaled)
    x, y, z, w = (value / length for value in scaled)
    return (
        (1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w), px),
        (2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w), py),
        (2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y), pz),
        _IDENTITY[3],
    )


def pose_to_quaternion(target):
    """Return (xyz, xyzw), choosing w >= 0."""
    matrix = validate_pose(target)
    trace = sum(matrix[i][i] for i in range(3))
    if trace > 0:
        scale = 2 * sqrt(1 + trace)
        quaternion = (
            (matrix[2][1] - matrix[1][2]) / scale,
            (matrix[0][2] - matrix[2][0]) / scale,
            (matrix[1][0] - matrix[0][1]) / scale,
            scale / 4,
        )
    else:
        i = max(range(3), key=lambda axis: matrix[axis][axis])
        j, k = (i + 1) % 3, (i + 2) % 3
        scale = 2 * sqrt(1 + matrix[i][i] - matrix[j][j] - matrix[k][k])
        quaternion = [0.0] * 4
        quaternion[i] = scale / 4
        quaternion[j] = (matrix[i][j] + matrix[j][i]) / scale
        quaternion[k] = (matrix[i][k] + matrix[k][i]) / scale
        quaternion[3] = (matrix[k][j] - matrix[j][k]) / scale
    length = hypot(*quaternion)
    sign = -1 if quaternion[3] < 0 else 1
    return tuple(row[3] for row in matrix[:3]), tuple(sign * value / length for value in quaternion)
