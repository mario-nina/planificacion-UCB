#!/usr/bin/env python3
"""Summarize a live ROS session without making controller requests."""
import argparse
import json
from math import isfinite
from os import environ
from sys import float_info
from time import monotonic

FRESHNESS = 1.0
MODES = {"offline": "Offline · simulación", "observe": "Observe · solo lectura",
         "control": "Control"}
BACKENDS = {"external": "ROS externo", "rws": "RWS/RAPID — Normal",
            "rws_trajectory": "RWS/RAPID — Trayectorias v2"}
RWS_BACKENDS = ("rws", "rws_trajectory")
CONTROLLER_STATES = {
    "mode": {"AUTO", "MANR", "MANF", "INIT", "UNDEF"},
    "motors": {"motoron", "motoroff", "guardstop", "emergencystop", "emergencystopreset",
               "init", "sysfail"},
    "execution": {"running", "stopped", "uninitiated", "unknown"},
}


def number(value):
    return type(value) in (float, int) and abs(value) <= float_info.max and isfinite(value)


def fresh_packet(data, started_ns, now_ns):
    """Ignore latched or queued reports from before this invocation."""
    if isinstance(data, dict):
        stamp = data.get("timestamp_ns")
        if type(stamp) is int and started_ns <= stamp <= now_ns:
            if now_ns - stamp <= FRESHNESS * 1e9:
                return data
    return None


def telemetry(data, now_ns):
    age = data.get("feedback_age_s")
    stamp = data.get("actual_stamp_ns")
    q = data.get("actual")
    valid_stamp = type(stamp) is int and 0 < stamp <= data["timestamp_ns"]
    if number(age) and age >= 0:
        age += max(0, now_ns - data["timestamp_ns"]) / 1e9
    else:
        age = None
    backend = data.get("backend")
    fresh = (
        data.get("mode") in ("observe", "control") and backend in RWS_BACKENDS
        and data.get("source") == backend and data.get("backend_connected") is True
        and data.get("stale") is False and age is not None and age <= FRESHNESS
        and isinstance(q, list) and len(q) == 6 and all(number(joint) for joint in q)
        and valid_stamp
    )
    return age, stamp if valid_stamp else None, fresh


def observe_ready(data, now_ns):
    return (data.get("mode") == "observe" and telemetry(data, now_ns)[2]
            and data.get("physical_armed") is False)


def describe(data, now_ns):
    age, stamp, fresh = telemetry(data, now_ns)
    mode = data.get("mode")
    backend = data.get("backend")
    print("Modo · " + next((label for key, label in MODES.items() if key == mode), "desconocido"))
    print("Backend · " + next((label for key, label in BACKENDS.items() if key == backend), "desconocido"))
    transport = data.get("transport")
    if not isinstance(transport, dict):
        transport = {}
    status = transport.get("http_status")
    if fresh:
        print("RWS · lectura autenticada · telemetría física reciente de J1–J6")
    elif mode == "offline":
        print("Feedback · postura virtual")
    elif backend not in RWS_BACKENDS:
        print("Feedback RWS · sin medición por este backend")
    elif status == 401:
        print("RWS · HTTP 401 · vuelve a ingresar las credenciales autorizadas")
    elif status == 403:
        print("RWS · HTTP 403 · confirma permisos de lectura de la cuenta")
    elif type(status) is int and 400 <= status <= 599:
        print(f"RWS · HTTP {status} · ejecuta irc5_demo.sh diagnose")
    else:
        print("RWS · sin medición vigente · revisa cable, ruta y servicio con diagnose")
    if mode != "offline" and age is not None:
        print(f"Antigüedad · {age:.3f} s" + (" · caducada" if age > FRESHNESS else ""))
    if mode != "offline" and stamp is not None:
        print(f"Sello de recepción ROS · {stamp} ns")
    values = []
    for key, allowed in CONTROLLER_STATES.items():
        value = transport.get(key)
        values.append(value if isinstance(value, str) and value in allowed else "sin dato")
    print("IRC5 · modo {} · motores {} · RAPID {}".format(*values))
    print("Habilitación física · " + ("activa" if data.get("physical_armed") is True else "bloqueada"))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--expect-observe", action="store_true")
    parser.add_argument("--wait", type=float, default=3.0, metavar="SEG")
    args = parser.parse_args()
    if not 0.1 <= args.wait <= 15:
        parser.error("--wait requiere entre 0.1 y 15 segundos")

    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import String

    environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    rclpy.init(args=[])
    node = None
    try:
        node = rclpy.create_node("imt342_diagnostico")
        clock = node.get_clock()
        started_ns = clock.now().nanoseconds
        sample = None

        def receive(msg):
            nonlocal sample
            try:
                data = json.loads(msg.data)
            except (ValueError, RecursionError):
                return
            candidate = fresh_packet(data, started_ns, clock.now().nanoseconds)
            if candidate and (sample is None or candidate["timestamp_ns"] > sample["timestamp_ns"]):
                sample = candidate

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE)
        node.create_subscription(String, "/imt342/analysis", receive, qos)
        deadline = monotonic() + args.wait
        while monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=min(0.1, max(0, deadline - monotonic())))
            now_ns = clock.now().nanoseconds
            if sample and (not args.expect_observe or observe_ready(sample, now_ns)):
                break
        now_ns = clock.now().nanoseconds
        if not fresh_packet(sample, started_ns, now_ns):
            print("Sesión ROS · sin diagnóstico reciente · revisa el lanzamiento y el dominio ROS")
            return 1
        describe(sample, now_ns)
        if args.expect_observe and not observe_ready(sample, now_ns):
            print("Observe · preparación pendiente")
            return 2
        return 0
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
