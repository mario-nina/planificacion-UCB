"""Serialize RWS command ownership and keep HTTP off the ROS thread."""

from concurrent.futures import ThreadPoolExecutor
from threading import Event
from time import monotonic

from .state import Result


class BackendUnavailable(Exception):
    """The client failed to initialize before taking command ownership."""


FRESHNESS = 1.0
ARM_TIMEOUT = 15.0

TERMINAL = {Result.VERIFIED, Result.FAILED, Result.CANCELLED}
RWS_BACKENDS = {"rws", "rws_trajectory"}


class Supervisor:
    def __init__(self, factories, selected):
        if selected not in RWS_BACKENDS | {"external"}:
            raise ValueError("Backend inválido")
        self.factories = factories
        self.selected = selected
        self.mode = "offline"
        self.generation = 0
        self.ready = False
        self.arm_deadline = 0.0
        self.dispatch = None
        self.owner = None
        self.pending = None
        self.cancel_requested = False
        self.motion_generation = 0
        self.motion_observation = None
        self._motion_dispatched = False
        self.feedback = None
        self.detail = self._connection_detail(selected)
        self.clients = {}
        self.next_detail = 0.0
        self.jobs = {}
        self.next_poll = {}
        # Both RWS adapters use the same Digest HTTP session. A read still
        # finishing after selection must not overlap the new adapter's requests.
        self.pool = ThreadPoolExecutor(max_workers=1)
        self.rapid_preparation = {"state": "idle", "detail": "", "http_status": None}
        self._prepare_pending = None

    @property
    def preparing_rapid(self):
        return self.rapid_preparation["state"] == "preparing"

    def request_prepare_rapid(self, module_source=None, recover_missing=False):
        if (
            self.mode != "observe" or self.selected not in RWS_BACKENDS or not self.connected
            or self.owner is not None or self.preparing_rapid or self.selected not in self.factories
        ):
            return False
        self.disarm()
        self.rapid_preparation = {
            "state": "preparing", "detail": "Consultando y preparando el servidor RAPID",
            "http_status": None,
        }
        self._prepare_pending = (module_source, recover_missing)
        return True

    def _connection_detail(self, name):
        if name == "external":
            return "Telemetría ROS externa"
        if name not in self.factories:
            return f"{name} · backend sin configurar"
        return f"{name} · esperando lectura"

    def set_mode(self, mode):
        if mode not in {"offline", "observe", "control"}:
            return False
        if mode == self.mode:
            return True
        self.disarm()
        if self.owner is not None or self.preparing_rapid:
            return False
        self.mode = mode
        self._invalidate_feedback()
        return True

    def _invalidate_feedback(self):
        self.generation += 1
        self.feedback = None
        self.ready = False
        self.detail = self._connection_detail(self.selected)

    def select(self, name):
        supported = RWS_BACKENDS | {"external"} if self.mode == "offline" else self.factories.keys() | {"external"}
        if name not in supported:
            return False
        if name != self.selected:
            self.disarm()
            if self.owner is not None or self.preparing_rapid:
                return False
            self.selected = name
            self._invalidate_feedback()
        return True

    @staticmethod
    def _fresh(feedback):
        return feedback is not None and 0 <= monotonic() - feedback.received_at <= FRESHNESS

    @property
    def connected(self):
        return self.mode != "offline" and self._fresh(self.feedback)

    @property
    def can_arm(self):
        return (
            self.mode == "control" and self.owner is None and self.selected in self.factories
            and self.ready and self.connected and self.feedback.source == self.selected
        )

    @property
    def armed(self):
        return self.can_arm and monotonic() < self.arm_deadline

    def arm(self):
        if not self.can_arm:
            return False
        self.arm_deadline = monotonic() + ARM_TIMEOUT
        return True

    def disarm(self):
        self.arm_deadline = 0.0
        if self.dispatch is not None:
            self.dispatch.clear()
        if self.pending is not None:
            self.pending = self.owner = None
            self.detail = "Habilitación revocada antes del envío"

    def execute(self, goal, trajectory):
        if self.owner is not None:
            return False
        if not self.armed:
            self.disarm()
            return False
        expires = self.arm_deadline
        self.arm_deadline = 0.0
        return self._queue_motion(goal, trajectory, lambda: monotonic() < expires)

    def execute_batch_leg(self, goal, trajectory, authorized, prior_sequence):
        if self.selected not in RWS_BACKENDS or not self.can_arm or not authorized():
            return False
        self.arm_deadline = 0.0
        return self._queue_motion(goal, trajectory, authorized, (prior_sequence,))

    def _queue_motion(self, goal, trajectory, scope_authorized, extra=()):
        self.motion_generation += 1
        self.motion_observation = None
        self._motion_dispatched = False
        permission = Event()
        permission.set()
        self.dispatch = permission

        def authorized():
            return (permission.is_set() and self.mode == "control" and self.connected
                    and scope_authorized())

        self.owner = self.selected
        self.pending = (goal, trajectory, authorized, *extra)
        self.detail = "Solicitud en curso"
        return True

    def cancel(self):
        pending = self.pending is not None
        self.disarm()
        if pending:
            self.detail = "Solicitud cancelada antes del envío"
        elif self.owner is not None:
            self.cancel_requested = True

    def _run(self, name, action, payload, check_ready):
        if name not in self.clients:
            try:
                self.clients[name] = self.factories[name]()
            except Exception as exc:
                raise BackendUnavailable(type(exc).__name__) from exc
        client = self.clients[name]
        if action == "execute":
            client.execute(*payload)
        elif action == "cancel":
            client.cancel()
        elif action == "prepare":
            from .rws_backend import RwsError
            try:
                state = client.start_server(*payload)
            except RwsError as error:
                return {"state": error.state, "detail": str(error), "http_status": error.status,
                        "controller": getattr(client, "controller_metadata", {})}
            detail = ("Servidor RAPID iniciado y verificado" if state == "started"
                      else "Servidor RAPID ya en ejecución")
            return {"state": state, "detail": detail, "http_status": None,
                    "controller": getattr(client, "controller_metadata", {})}
        feedback, result, detail = client.poll(check_ready=check_ready)
        motion = getattr(client, "motion_evidence", None)
        return feedback, result, detail, client.ready, motion

    def tick(self, online):
        online = online and self.mode != "offline"
        if not online or not self.connected:
            self.disarm()
        for name, (job, action, generation) in tuple(self.jobs.items()):
            if not job.done():
                continue
            del self.jobs[name]
            if action == "prepare":
                try:
                    self.rapid_preparation = job.result()
                except Exception as exc:
                    self.rapid_preparation = {
                        "state": "failed", "http_status": None,
                        "detail": f"Preparación RAPID sin confirmar ({type(exc).__name__})",
                    }
                continue
            try:
                feedback, result, detail, ready, motion = job.result()
            except Exception as exc:
                if name == self.selected and generation == self.generation:
                    self.feedback = None
                    self.ready = False
                    self.disarm()
                    error = str(exc) if isinstance(exc, BackendUnavailable) else type(exc).__name__
                    self.detail = f"{name} · error {error}"
                if isinstance(exc, BackendUnavailable) and action == "execute":
                    self.owner = None
                    self.cancel_requested = False
                # A command may have reached the controller before the error.
                continue
            released = False
            if name == self.owner and self._motion_dispatched:
                if result in TERMINAL:
                    released = True
                    released_at = monotonic()
                    if motion and motion.get("sequence") is not None:
                        motion = {**motion, "host_release_monotonic_s": released_at}
                        sent = motion.get("host_submit_monotonic_s")
                        if sent is not None:
                            motion["submit_to_release_s"] = max(0.0, released_at - sent)
                    self.owner = None
                    self.dispatch = None
                    self.cancel_requested = False
                    self.next_poll[name] = 0.0
                self.motion_observation = (result, motion)
            if generation != self.generation or not online:
                continue
            fresh = self._fresh(feedback)
            old_ready = self.ready
            if name == self.selected:
                self.ready = ready and fresh and result is not Result.UNKNOWN
                if not fresh and self.feedback is not None and self.feedback.source == name:
                    self.feedback = None
                if not self.ready:
                    self.disarm()
            if fresh and name == self.selected:
                if self.feedback is None or feedback.received_at >= self.feedback.received_at:
                    self.feedback = feedback
            if name == self.selected and (action != "poll" or released or self.ready != old_ready
                                           or monotonic() >= self.next_detail):
                self.detail = f"{name} · {detail}"
                self.next_detail = monotonic() + 0.25

        active = {self.owner} if self.owner else set()
        if online:
            active.add(self.selected)
        for name in active & self.factories.keys():
            if name in self.jobs:
                continue
            action, payload = "poll", ()
            if name == self.selected and self._prepare_pending is not None:
                preparation = self._prepare_pending
                self._prepare_pending = None
                generation = self.generation

                def prepare_authorized(selected=name):
                    return (self.preparing_rapid and self.generation == generation
                            and self.mode == "observe" and self.selected == selected)

                if not self.connected:
                    self.rapid_preparation = {
                        "state": "failed", "detail": "La lectura RWS venció antes de preparar RAPID",
                        "http_status": None,
                    }
                    continue
                action, payload = "prepare", (prepare_authorized, *preparation)
            if name == self.owner:
                if self.pending is not None:
                    if not self.pending[2]():
                        self.disarm()
                        continue
                    action, payload = "execute", self.pending
                    self.pending = None
                    self._motion_dispatched = True
                elif self.cancel_requested:
                    action = "cancel"
                    self.cancel_requested = False
            now = monotonic()
            if action == "poll" and now < self.next_poll.get(name, 0):
                continue
            # During candidate v2 motion only, sample at up to 10 Hz instead of 6.7 Hz.
            # One RWS worker remains shared; slow requests are never overlapped.
            # If an observation is uncertain, use the original cadence.
            moving_v2 = (name == "rws_trajectory" and name == self.owner
                         and self.motion_observation is not None
                         and self.motion_observation[0] in {Result.ACCEPTED, Result.EXECUTING}
                         and (self.motion_observation[1] or {}).get("module_revision") in {"lspb_candidate", "continuous_candidate"})
            self.next_poll[name] = now + (0.10 if moving_v2 else 0.15)
            check_ready = self.mode == "control" and name == self.selected
            job = self.pool.submit(self._run, name, action, payload, check_ready)
            self.jobs[name] = job, action, self.generation

    def close(self):
        self.disarm()
        if self.preparing_rapid:
            self.rapid_preparation = {
                "state": "failed", "detail": "Sesión cerrada durante la preparación RAPID",
                "http_status": None,
            }
        self.pool.shutdown(wait=True, cancel_futures=True)
        if self.owner is not None and self.owner in self.clients:
            try:
                self.clients[self.owner].cancel()
            except Exception:
                pass
        for client in self.clients.values():
            try:
                client.close()
            except RuntimeError:
                continue
