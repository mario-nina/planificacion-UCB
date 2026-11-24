"""Bridge transport-specific lifecycle and ROS coordinator polling."""

import os
from dataclasses import replace
from hashlib import sha256
from math import isfinite, radians
from time import monotonic

from .state import Result

# Equal to the existing v2 module's TrjAt band; no RAPID source change.
ARRIVAL_TOLERANCE_RAD = radians(0.30)
PLAN_ORIGIN_TOLERANCE_RAD = radians(0.05)
ACTIVE_RESULTS = {Result.TRANSMITTED, Result.ACCEPTED, Result.EXECUTING}


def transient_read_error(error):
    return error.state == "transport_error" or error.status in {500, 502, 503, 504}


class RwsAdapter:
    def __init__(self, backend, transition=None, close_session=None):
        self.backend = backend
        self.transition = transition
        self.close_session = close_session
        self.next_diagnostic = 0.0
        self.diagnostic = {}
        self.error = None
        self.ready = False
        self.next_preflight = 0.0
        self.preflight_error = None
        self.preflight_state = None
        self.preflight_transient = False
        self.arrival_observation = {}
        self.last_terminal_diagnostic_sequence = None
        self.read_failures = 0
        self.next_read = 0.0
        self.first_transport_error = None
        self.last_read_error = None
        self.transient_failure = False

    @property
    def motion_evidence(self):
        final_error = self.backend.final_error_rad
        joint_errors = self.arrival_observation.get("final_error_by_joint_rad")
        if final_error is None and joint_errors is not None:
            final_error = max(map(abs, joint_errors))
        return {**self.backend.motion_evidence, **self.arrival_observation,
                "final_error_rad": final_error,
                "final_q_rad": (self.backend.final_feedback.positions
                                if self.backend.final_feedback is not None else None)}

    @property
    def controller_metadata(self):
        return dict(self.backend.controller_metadata)

    def start_server(self, authorized, module_source=None, recover_missing=False):
        try:
            if not authorized():
                from .rws_backend import RwsError
                raise RwsError("Preparación RAPID revocada")
            # A prepare click authorizes adding the pinned own module if absent.
            # Existing modules are verified, never overwritten.
            recover_missing = recover_missing or (module_source is not None and not self.backend._loaded_module())
            if self.transition is not None:
                return self.transition().prepare_normal(authorized, module_source, recover_missing)
            return self.backend.start_server(authorized=authorized, module_source=module_source,
                                             recover_missing=recover_missing)
        finally:
            self.next_diagnostic = 0.0
            self.next_preflight = 0.0
            self.next_read = 0.0

    def execute(self, goal, trajectory, authorized=lambda: False, expected_prior_sequence=None):
        from .rws_backend import RwsError
        self.arrival_observation = {}
        try:
            self.backend.execute(goal, authorized=authorized,
                                 expected_start=trajectory.sample(0.0).positions,
                                 expected_prior_sequence=expected_prior_sequence,
                                 duration=getattr(trajectory, "duration", None),
                                 origin_tolerance_rad=self._origin_tolerance(expected_prior_sequence))
        except (ValueError, RwsError) as exc:
            self.error = str(exc)
        else:
            self.error = None

    def _origin_tolerance(self, expected_prior_sequence):
        # The first command uses the plan-origin band. The next segment can
        # start from its own correlated predecessor within the arrival band.
        limit = ARRIVAL_TOLERANCE_RAD if expected_prior_sequence is not None else PLAN_ORIGIN_TOLERANCE_RAD
        return min(self.backend.config.tolerance_rad, limit)

    def poll(self, *, check_ready=False):
        from .rws_backend import RwsError
        # Snapshot the local send clock before result() clears it on a terminal reply.
        sent_at = self.backend._sent_at
        result = self.backend.result()
        feedback = None
        read_error = None
        self.transient_failure = False
        # Only throttle failed idle reads. Never defer reconciliation of an
        # in-flight order, cancel or dispatch; no automatic retry of writes.
        if result not in ACTIVE_RESULTS | {Result.UNKNOWN} and monotonic() < self.next_read:
            self.ready = False
            diagnostic = {**self.diagnostic, "result": result.value,
                          "motion_evidence": self.motion_evidence,
                          "error": self.last_read_error,
                          "first_transport_error": self.first_transport_error,
                          "read_retry_in_s": max(0.0, self.next_read-monotonic())}
            return None, result, self._describe(diagnostic)
        try:
            final = (self.backend.final_feedback
                     if sent_at is not None and result in {Result.VERIFIED, Result.FAILED, Result.CANCELLED}
                     else None)
            feedback = final if final is not None else self.backend.read_state()
        except RwsError as exc:
            read_error = str(exc)
            self.last_read_error = read_error
            self.transient_failure = transient_read_error(exc)
            if self.transient_failure:
                self.first_transport_error = self.first_transport_error or read_error
                self.read_failures += 1
                self.next_read = monotonic() + min(4.0, 0.5 * 2 ** min(self.read_failures-1, 3))
            self.diagnostic = {}
        else:
            self.read_failures = 0
            self.next_read = 0.0
            self.last_read_error = None
            if self._diagnostic_due(result):
                self.next_diagnostic = monotonic() + 4.0
                try:
                    self.diagnostic = self.backend.diagnostic()
                except RwsError as exc:
                    read_error = str(exc)
                    self.transient_failure = transient_read_error(exc)
                    if self.transient_failure:
                        self.first_transport_error = self.first_transport_error or read_error
                    self.diagnostic = {}
        if not self._can_preflight(check_ready, result, read_error):
            self.ready = False
            self.next_preflight = 0.0
            self.preflight_error = self.preflight_state = None
            self.preflight_transient = False
        elif monotonic() >= self.next_preflight:
            self.next_preflight = monotonic() + 1.0
            try:
                self.backend.prepare()
            except RwsError as exc:
                self.ready = False
                self.preflight_error = str(exc)
                self.preflight_state = exc.state
                self.preflight_transient = self.transient_failure = transient_read_error(exc)
                if self.transient_failure:
                    self.first_transport_error = self.first_transport_error or str(exc)
                if exc.state == "server_returning":
                    self.next_preflight = self.next_diagnostic = 0.0
            else:
                self.ready = True
                self.preflight_error = self.preflight_state = None
                self.preflight_transient = False
                self._prepared()
        self._after_read(feedback, result)
        self._record_arrival(feedback, result, sent_at)
        diagnostic = {
            **self.diagnostic,
            "result": result.value,
            "first_transport_error": self.first_transport_error,
            "motion_evidence": self.motion_evidence,
            "final_error_rad": self.motion_evidence["final_error_rad"],
            "error": read_error or self.error or self.backend.last_error or self.preflight_error,
        }
        return feedback, result, self._describe(diagnostic)

    def _record_arrival(self, feedback, result, sent_at):
        evidence = self.backend.motion_evidence
        goal, sequence = evidence.get("target_rad"), evidence.get("sequence")
        if self.arrival_observation.get("arrival_sequence") != sequence:
            self.arrival_observation = {}
        if goal is None or sequence is None:
            return
        observation = self.arrival_observation
        observation["arrival_sequence"] = sequence
        if sent_at is not None:
            observation.setdefault("host_submit_monotonic_s", sent_at)
        sent = observation.get("host_submit_monotonic_s", evidence.get("host_submit_monotonic_s"))
        tolerance = self.backend.config.tolerance_rad
        initial = evidence.get("start_feedback_rad")
        if initial is not None:
            observation.setdefault("start_within_arrival_band", all(
                abs(measured - target) <= tolerance for measured, target in zip(initial, goal)
            ))

        reply = evidence.get("reply") or ()
        has_ack = (len(reply) == 2 and reply[0] == sequence and reply[1] in (3, 4, 5))
        ack = evidence.get("submit_to_terminal_reply_s") if has_ack else None
        if sent is not None and ack is not None:
            observation.setdefault("terminal_ack_host_monotonic_s", sent + ack)
        active = result in ACTIVE_RESULTS or result is Result.UNKNOWN
        observation["awaiting_terminal_reply"] = active and not has_ack
        final = self.backend.final_feedback
        observed = final if has_ack or not active else feedback
        if observed is None or observation.get("arrival_frozen"):
            return
        signed_errors = tuple(measured - target for measured, target in zip(observed.positions, goal))
        maximum = max(map(abs, signed_errors))
        in_band = maximum <= tolerance
        observation.update(arrival_tolerance_rad=tolerance, latest_goal_error_rad=maximum,
                           within_arrival_band=in_band)
        if in_band and sent is not None:
            # A final read can be the first sampled in-band posture at RWS rate.
            elapsed = max(0.0, observed.received_at - sent)
            observation.setdefault("first_within_band_monotonic_s", observed.received_at)
            observation.setdefault("submit_to_first_observed_band_s", elapsed)
            observation.setdefault("band_first_seen_at_terminal", has_ack)
        if not active:
            observation["arrival_frozen"] = True
            observation["final_error_by_joint_rad"] = signed_errors
            if result is Result.VERIFIED and has_ack and reply[1] == 3 and in_band:
                observation["stable_arrival_host_monotonic_s"] = observed.received_at
                observation["stable_arrival_source"] = "rapid_phase_3_and_rws_final_pose"
            first = observation.get("submit_to_first_observed_band_s")
            if first is not None and ack is not None and not observation.get("band_first_seen_at_terminal"):
                observation["observed_band_to_terminal_s"] = max(0.0, ack - first)

    def _can_preflight(self, check_ready, result, read_error):
        return (check_ready and not read_error and result is not Result.UNKNOWN
                and result not in ACTIVE_RESULTS)

    def _prepared(self):
        pass

    def _after_read(self, feedback, result):
        pass

    def _diagnostic_due(self, result):
        # Refresh closed-sequence state immediately, instead of leaving the
        # panel blocked by a cached pending_sequence for the diagnostic period.
        reply = self.backend.motion_evidence.get("reply") or ()
        closed = len(reply) == 2 and reply[1] in (3, 4, 5)
        changed = closed and reply[0] != self.last_terminal_diagnostic_sequence
        if changed:
            self.last_terminal_diagnostic_sequence = reply[0]
        return changed or monotonic() >= self.next_diagnostic

    def _describe(self, diagnostic):
        return str(diagnostic)

    def cancel(self):
        self.backend.cancel()

    def close(self):
        (self.close_session or self.backend.close)()


class RwsTrajectoryAdapter(RwsAdapter):
    def __init__(self, backend, motion_enabled=False, transition=None, close_session=None):
        super().__init__(backend, transition, close_session)
        self.motion_enabled = motion_enabled
        self.local_rejection = None
        self.management_result = None

    def start_server(self, authorized, module_source=None, recover_missing=False):
        from .rws_backend import RwsError
        self.disable_session_motion()
        if self.transition is None:
            raise RwsError("La preparación v2 requiere el adaptador RWS configurado")
        try:
            transition = self.transition()
            if not authorized():
                raise RwsError("Preparación RAPID revocada")
            missing = not self.backend._loaded_module()
            if missing and not transition.normal._loaded_module():
                # The existing additive v2 installer requires a verified Normal.
                # Bootstrap its idle server, then perform the normal handover.
                transition.prepare_normal(authorized, transition.normal_source, recover_missing=True)
            state = transition.prepare_v2(authorized, recover_missing=recover_missing or missing)
        except RwsError as error:
            self.management_result = {"state": error.state, "reason": str(error)}
            raise
        finally:
            self.next_diagnostic = self.next_preflight = self.next_read = 0.0
        self.motion_enabled = authorized()
        self.management_result = {"state": state}
        return state

    def disable_session_motion(self):
        self.motion_enabled = self.ready = False
        reset = getattr(self.backend, "reset_shape_calibration", None)
        if reset is not None:
            reset()

    @property
    def motion_evidence(self):
        return {} if self.local_rejection else super().motion_evidence

    def execute(self, goal, trajectory, authorized=lambda: False, expected_prior_sequence=None):
        from .rws_backend import RwsError
        if not self.motion_enabled or len(getattr(trajectory, "segments", (trajectory,))) != 1:
            self.local_rejection = "Prepara el servidor v2 y envía un destino por tramo"
            return
        self.local_rejection = None
        self.arrival_observation = {}
        try:
            segment = getattr(trajectory, "segments", (trajectory,))[0]
            options = {"linear_cartesian": True} if getattr(segment, "linear_cartesian", False) else {}
            self.backend.execute(goal, duration=trajectory.duration, authorized=authorized,
                                 expected_start=trajectory.sample(0.0).positions,
                                 expected_prior_sequence=expected_prior_sequence,
                                 origin_tolerance_rad=self._origin_tolerance(expected_prior_sequence),
                                 reference_blend=(getattr(segment, "blend_fraction", None)
                                                  if getattr(segment, "profile", None) == "lspb" else None), **options)
        except (ValueError, RwsError) as exc:
            self.error = str(exc)
        else:
            self.error = None
        finally:
            self.next_diagnostic = 0.0

    def poll(self, *, check_ready=False):
        feedback, result, detail = super().poll(check_ready=check_ready)
        return feedback, Result.FAILED if self.local_rejection else result, detail

    def _can_preflight(self, check_ready, result, read_error):
        return self.motion_enabled and super()._can_preflight(check_ready, result, read_error)

    def _after_read(self, feedback, result):
        if result is Result.UNKNOWN:
            self.disable_session_motion()
        elif (feedback is None or self.preflight_error and self.preflight_state != "server_returning"):
            # An idle HTTP failure is not a change of RAPID program. Keep the
            # prepared session, not the motion permission; next fresh preflight
            # must succeed. A protocol/ownership failure still revokes it.
            if (self.transient_failure or self.preflight_transient) and result not in ACTIVE_RESULTS:
                self.ready = False
            else:
                self.disable_session_motion()

    def _prepared(self):
        self.diagnostic.update(self.backend.ready_diagnostic)

    def _describe(self, diagnostic):
        if self.local_rejection:
            diagnostic.update(result=Result.FAILED.value, motion_evidence={})
        diagnostic["trajectory_motion_enabled"] = self.motion_enabled
        diagnostic["rapid_management"] = self.management_result
        diagnostic["trajectory_status"] = (
            self.local_rejection or diagnostic["error"] or diagnostic.get("rapid_diagnostic")
            or ("Servidor v2 verificado · tiempo solicitado a ABB" if self.motion_enabled
                else "Prepara el servidor de trayectorias v2")
        )
        return str(diagnostic)


def configured_backends(node, *, trajectory_source=None, normal_source=None):
    def parameter(name, default):
        return node.declare_parameter(name, default).value

    url = parameter("rws_url", "")
    task = parameter("rws_task", "")
    module = parameter("rws_module", "")
    mechunit = parameter("rws_mechunit", "")
    tolerance = parameter("rws_tolerance_rad", 0.0)
    timeout = parameter("rws_timeout_sec", 2.0)
    ack_timeout = parameter("rws_ack_timeout_sec", 5.0)

    if not all((url, task, module, mechunit, isfinite(tolerance) and tolerance > 0, timeout > 0)):
        return {}
    # The external setting still determines whether RWS is configured. Use a
    # common bounded position band for dispatch/arrival, matching the pinned v2.
    tolerance = ARRIVAL_TOLERANCE_RAD
    from .rws_backend import MODULE_NAME as NORMAL
    from .rws_trajectory import MODULE_NAME as V2, MODULE_SHA256

    # An absent or incorrect v2 file must not interfere with the Normal factory.
    with_v2 = (isinstance(trajectory_source, bytes)
               and sha256(trajectory_source).hexdigest() == MODULE_SHA256
               and (task, module, mechunit) == ("T_ROB1", NORMAL, "ROB_1"))
    normal = transition = None

    def normal_backend():
        from .rws_backend import RwsConfig, RwsRapidBackend
        nonlocal normal
        if normal is None:
            config = RwsConfig(url, os.environ["IMT342_RWS_USERNAME"],
                               os.environ["IMT342_RWS_PASSWORD"], task, module, mechunit,
                               tolerance, timeout, ack_timeout)
            normal = RwsRapidBackend(config)
        return normal

    def trajectory_transition():
        from .rws_trajectory import RwsTrajectoryBackend
        from .rws_transition import RwsTrajectoryTransition
        nonlocal transition
        if transition is None:
            backend = normal_backend()
            trajectory = RwsTrajectoryBackend(replace(backend.config, module=V2), trajectory_source,
                                              http=backend.http, allow_candidate=True)
            transition = RwsTrajectoryTransition(backend, trajectory, normal_source)
        return transition

    def close_session():
        nonlocal normal
        if normal is not None:
            normal.close()
            normal = None

    factories = {"rws": lambda: RwsAdapter(normal_backend(), trajectory_transition if with_v2 else None,
                                           close_session)}
    if with_v2:
        factories["rws_trajectory"] = lambda: RwsTrajectoryAdapter(
            trajectory_transition().trajectory, transition=trajectory_transition, close_session=close_session,
        )
    return factories
