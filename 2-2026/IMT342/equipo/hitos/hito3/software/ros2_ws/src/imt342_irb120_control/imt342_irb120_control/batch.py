"""Consume one approved trajectory by dispatching each discrete leg once."""

from math import isfinite

from .adapters import ARRIVAL_TOLERANCE_RAD
from .state import Result


class BatchRun:
    def __init__(self, trajectory, now, timeout, *, backend="rws"):
        self.trajectory = trajectory
        self.started = now
        self.backend = backend
        self.deadline = now + timeout
        self.timeout = timeout
        self.index = 0
        self.waiting = False
        self.motion_generation = None
        self.prior_sequence = None
        self.state = "executing"
        self.reason = ""
        self.steps = []

    @property
    def active(self):
        return self.state == "executing"

    def stop(self, state, reason):
        self.state, self.reason = state, reason

    def owns_latest_motion(self, supervisor):
        return self.waiting and self.motion_generation == supervisor.motion_generation

    def tick(self, supervisor, fresh, now):
        if not self.active:
            return
        if supervisor.selected != self.backend:
            self.stop("unknown" if self.waiting else "failed", "Cambió el backend del lote")
            return
        if not fresh or not supervisor.connected:
            self.stop("unknown" if self.waiting else "failed", "Telemetría vencida o desconectada")
            return
        if now >= self.deadline:
            self.stop("unknown" if self.waiting else "failed", "Venció el plazo del tramo")
            return
        if self.waiting:
            if not self.owns_latest_motion(supervisor):
                self.stop("unknown", "Cambió el propietario del tramo")
                return
            observation = supervisor.motion_observation
            if observation is None:
                if supervisor.owner is None:
                    self.stop("failed", "El tramo quedó sin propietario ni confirmación")
                return
            result, evidence = observation
            evidence = evidence or {}
            self.steps[-1].update(result=result.value, motion=evidence)
            if result is Result.UNKNOWN or evidence.get("uncertainty_observed"):
                self.stop("unknown", "Envío o resultado RAPID incierto")
                return
            if result in (Result.FAILED, Result.CANCELLED):
                self.stop(result.value, "RAPID no verificó el destino")
                return
            if result is not Result.VERIFIED:
                return
            sequence = evidence.get("sequence")
            reply = evidence.get("reply")
            previous = self.prior_sequence
            if (not evidence.get("submit_confirmed") or type(sequence) is not int
                    or tuple(reply or ()) != (sequence, 3)
                    or (previous is not None and sequence != previous + 1)):
                self.stop("unknown", "Respuesta RAPID sin correlación inequívoca")
                return
            goal = self.trajectory.segments[self.index].goal
            target = evidence.get("target_rad")
            final = evidence.get("final_feedback_rad")
            # Correlate the actual command and final measurement, not just a
            # success code from a potentially unrelated historical order.
            def matches(values, tolerance):
                return (isinstance(values, (list, tuple)) and len(values) == len(goal)
                        and all(isinstance(v, (int, float)) and isfinite(v)
                                and abs(v - q) <= tolerance for v, q in zip(values, goal)))
            if not matches(target, 1e-7) or not matches(final, ARRIVAL_TOLERANCE_RAD):
                self.stop("unknown", "El acuse no acredita el destino y la postura final de este tramo")
                return
            self.steps[-1]["verified_elapsed_s"] = now - self.started
            self.prior_sequence = sequence
            self._advance(now)
            return
        if not supervisor.can_arm:
            return
        segment = self.trajectory.segments[self.index]
        step = dict(index=self.index, start_q_rad=segment.start, target_q_rad=segment.goal,
                    duration_s=segment.duration)
        if getattr(segment, "profile", "lspb") == "lspb":
            step["lspb_s"] = segment.duration
        if segment.start == segment.goal:
            actual = supervisor.feedback.positions
            errors = tuple(measured - target for measured, target in zip(actual, segment.goal))
            maximum = max(map(abs, errors))
            if maximum > ARRIVAL_TOLERANCE_RAD:
                self.stop("failed", "Postura medida fuera de la banda del destino sin desplazamiento")
                return
            self.steps.append(dict(step, result="already_satisfied", checked_elapsed_s=now - self.started,
                                   motion={"target_rad": segment.goal, "final_feedback_rad": actual,
                                           "final_error_by_joint_rad": errors, "final_error_rad": maximum,
                                           "motion_sent": False}))
            self._advance(now)
            return
        # The backend checks recent position against segment.start again under mastership.
        if supervisor.execute_batch_leg(segment.goal, segment, lambda: self.active, self.prior_sequence):
            self.waiting = True
            self.motion_generation = supervisor.motion_generation
            self.steps.append(dict(step, request_elapsed_s=now - self.started, result="requested"))

    def _advance(self, now):
        self.index += 1
        self.waiting = False
        self.deadline = now + self.timeout
        if self.index == len(self.trajectory.segments):
            self.stop("verified", "Lote verificado")
