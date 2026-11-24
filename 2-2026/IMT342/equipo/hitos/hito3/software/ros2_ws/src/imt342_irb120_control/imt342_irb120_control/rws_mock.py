"""In-process HTTP-shaped RWS double and discrete RAPID execution contract."""

from json import dumps, loads
from math import degrees, radians

from .state import Feedback, Result, finite_positions, validate_target

JOINTTARGET = "/rw/motionsystem/mechunits/ROB_1/jointtarget?json=1"
# Only these private mock routes describe goal/result/cancel. They model the
# future contract with a RAPID consumer rather than actual RobotWare endpoints.
GOAL = "/mock/rapid/goal"
RESULT = "/mock/rapid/result"


class FakeHttp:
    """Simple fake HTTP server with separately advanced simulated completion."""

    def __init__(self):
        self.positions_deg = (0.0,) * 6
        self.target_deg = None
        self.state = Result.IDLE
        self.requests = []
        self.forced_status = None

    def request(self, method, path, body=None):
        self.requests.append((method, path, body))
        if self.forced_status is not None:
            return self.forced_status, ""
        if (method, path) == ("GET", JOINTTARGET):
            axes = {f"rax_{i}": str(axis) for i, axis in enumerate(self.positions_deg, 1)}
            return 200, dumps({"_embedded": {"_state": [axes]}})
        if (method, path) == ("POST", GOAL):
            if self.state is Result.ACCEPTED:
                return 409, ""
            self.target_deg = finite_positions(loads(body)["position_deg"])
            self.state = Result.ACCEPTED
            return 204, ""
        if (method, path) == ("GET", RESULT):
            return 200, dumps({"result": self.state.value})
        if (method, path) == ("DELETE", GOAL) and self.state is Result.ACCEPTED:
            self.target_deg = None
            self.state = Result.CANCELLED
            return 204, ""
        return 404, ""

    def complete(self):
        """Advance the mock controller after a separate simulated event."""
        if self.state is not Result.ACCEPTED:
            raise RuntimeError("El simulador RAPID no tiene un destino activo")
        self.positions_deg = self.target_deg
        self.target_deg = None
        self.state = Result.SIMULATED_COMPLETE


class RwsRapidMock:
    """RWS read, discrete acceptance, cancellation and independent result."""

    def __init__(self, http, clock):
        if not isinstance(http, FakeHttp):
            raise TypeError("Se requiere FakeHttp")
        self.http = http
        self.clock = clock

    def _request(self, method, path, expected, body=None):
        code, response = self.http.request(method, path, body)
        if code != expected:
            raise RuntimeError(f"RWS simulado devolvió HTTP {code} para {method} {path}")
        return response

    def read_state(self):
        raw = self._request("GET", JOINTTARGET, 200)
        axes = loads(raw)["_embedded"]["_state"][0]
        degrees_by_axis = finite_positions(axes[f"rax_{i}"] for i in range(1, 7))
        return Feedback(tuple(map(radians, degrees_by_axis)), self.clock(), "rws_mock")

    def execute(self, goal_rad):
        goal_deg = tuple(map(degrees, validate_target(goal_rad)))
        self._request("POST", GOAL, 204, dumps({"position_deg": goal_deg}))
        return Result.ACCEPTED

    def cancel(self):
        self._request("DELETE", GOAL, 204)
        return Result.CANCELLED

    def result(self):
        return Result(loads(self._request("GET", RESULT, 200))["result"])
