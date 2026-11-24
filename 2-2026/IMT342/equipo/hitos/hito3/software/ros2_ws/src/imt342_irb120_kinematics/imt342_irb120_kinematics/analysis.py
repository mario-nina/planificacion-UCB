"""Point-aware analysis of the audited IRB-120 URDF, without ROS dependencies."""

from dataclasses import dataclass
from functools import lru_cache
from math import hypot
from sys import float_info

from .dynamics import SingleLinkDynamics
from .model import Model, _IDENTITY, _Joint, _multiply, _pose_error, _svd
from .pose import pose_to_quaternion, pose_to_rpy, validate_pose


BASE_FRAME = "base_link"
# The URDF gives dh_frame_6 the same origin as link_6 and rotates its axes.
FLANGE_FRAME = "dh_frame_6"
ACTIVE_TCP_FRAME = "tcp_active"
POINTS = ("flange", "tcp")
LENGTH_M = 0.5
FD_STEPS_RAD = (1e-2, 1e-3, 1e-4, 1e-5, 1e-6, 1e-7)
DYNAMICS_1R = SingleLinkDynamics(inertia_kg_m2=0.08, mass_kg=1.2, com_radius_m=0.16)


@dataclass(frozen=True)
class PointModels:
    flange: Model
    tcp: Model
    tool_offset: tuple
    model_tool_offset: tuple

    @classmethod
    def from_model(cls, model):
        for i, joint in enumerate(model._chain):
            if joint.name == "link6_to_dh_frame_6":
                flange = Model(model._chain[:i + 1])
                fixed_tail = model._chain[i + 1:]
                if joint.axis is not None or any(member.axis is not None for member in fixed_tail):
                    break
                offset = _IDENTITY
                for member in fixed_tail:
                    offset = _multiply(offset, member.origin)
                return cls(flange, model, offset, offset)
        raise ValueError("El URDF no contiene la brida dh_frame_6 seguida de un TCP fijo")

    def with_tool(self, offset):
        offset = validate_pose(offset)
        tool = _Joint("flange_to_tcp_active", offset, None, None)
        return PointModels(self.flange, Model(self.flange._chain + (tool,)), offset,
                           self.model_tool_offset)

    def at(self, point):
        if point == "flange":
            return self.flange
        if point == "tcp":
            return self.tcp
        raise ValueError("El destino debe ser Brida o TCP activo")


def pose_record(matrix):
    xyz, rpy = pose_to_rpy(matrix)
    _, xyzw = pose_to_quaternion(matrix)
    return dict(matrix=matrix, xyz=xyz, rpy=rpy, xyzw=xyzw)


@lru_cache(maxsize=128)
def pose_records(points, q):
    return {point: pose_record(points.at(point).fk(q)) for point in POINTS}


def pose_residual(desired, actual):
    error = _pose_error(desired, actual)
    return dict(position_error_m=hypot(*error[:3]), orientation_error_rad=hypot(*error[3:]))


def twist(jacobian, qdot):
    return tuple(sum(coefficient * velocity for coefficient, velocity in zip(row, qdot))
                 for row in jacobian)


@lru_cache(maxsize=32)
def jacobian_record(model, q):
    jacobian = model.jacobian(q)
    linear, angular = jacobian[:3], jacobian[3:]
    scaled = tuple(tuple(value / LENGTH_M for value in row) for row in linear) + angular
    # Right vectors of Js transpose are the scaled Cartesian left directions of Js.
    sigma, left_vectors = _svd(tuple(zip(*scaled)), right_vectors=True)
    tol = len(jacobian) * float_info.epsilon * sigma[0]
    rank = sum(value > tol for value in sigma)
    fd = []
    for step in FD_STEPS_RAD:
        difference = model.jacobian_fd(q, step)
        error = tuple(a - b for row, test in zip(jacobian, difference) for a, b in zip(row, test))
        fd.append(dict(h=step, error_jv=hypot(*error[:18]), error_jw=hypot(*error[18:])))
    return dict(jv=linear, jw=angular, sigma=sigma, sigma_min=sigma[-1],
                kappa=sigma[0] / sigma[-1] if rank == len(sigma) else None,
                rank=rank, L=LENGTH_M, rank_tol=tol, fd=fd,
                weakest_direction_scaled=left_vectors[-1],
                inverse_gain=1 / sigma[-1] if rank == len(sigma) else None)
