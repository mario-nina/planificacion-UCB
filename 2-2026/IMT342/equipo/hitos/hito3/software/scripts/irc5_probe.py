#!/usr/bin/env python3
"""Local configuration validation and read-only RWS probe for the lab launcher."""

import argparse
import sys
from datetime import datetime, timezone
from ipaddress import IPv4Address
from math import isfinite
from pathlib import Path
from re import fullmatch
from time import monotonic
from urllib.parse import urlsplit
from xml.etree import ElementTree

import yaml

OPTIONAL_DEFAULTS = {"rws_timeout_sec": 2.0, "rws_ack_timeout_sec": 5.0}
REQUIRED = {"rws_url", "rws_task", "rws_module", "rws_mechunit", "rws_tolerance_rad"}


class UniqueKeysLoader(yaml.SafeLoader):
    def construct_mapping(self, node, deep=False):
        keys = [self.construct_object(key, deep=deep) for key, _ in node.value]
        if len(keys) != len(set(keys)):
            raise ValueError
        return super().construct_mapping(node, deep=deep)


def load_config(path):
    root = yaml.load(Path(path).read_text(encoding="utf-8"), Loader=UniqueKeysLoader)
    if not isinstance(root, dict) or set(root) != {"imt342_coordinator"}:
        raise ValueError
    node = root["imt342_coordinator"]
    if not isinstance(node, dict) or set(node) != {"ros__parameters"}:
        raise ValueError
    params = node["ros__parameters"]
    if not isinstance(params, dict) or not REQUIRED <= params.keys():
        raise ValueError
    allowed_rws = REQUIRED | OPTIONAL_DEFAULTS.keys()
    if any(not isinstance(key, str) or key not in allowed_rws for key in params):
        raise ValueError
    for key in ("rws_task", "rws_module", "rws_mechunit"):
        if not isinstance(params[key], str) or not fullmatch(r"[A-Za-z][A-Za-z0-9_]*", params[key]):
            raise ValueError
    for key in ("rws_tolerance_rad", *OPTIONAL_DEFAULTS):
        value = params.get(key, OPTIONAL_DEFAULTS.get(key))
        if type(value) is not float or not isfinite(value) or value <= 0:
            raise ValueError
    url = params["rws_url"]
    if not isinstance(url, str) or (origin := urlsplit(url)).scheme not in ("http", "https"):
        raise ValueError
    host = IPv4Address(origin.hostname)
    port = origin.port if origin.port is not None else (443 if origin.scheme == "https" else 80)
    authority = str(host) if origin.port is None else f"{host}:{port}"
    canonical = f"{origin.scheme}://{authority}"
    if not port or url not in {canonical, canonical + "/"}:
        raise ValueError
    return {**OPTIONAL_DEFAULTS, **params}, str(host), port


class ReadOnlyHttp:
    def __init__(self, transport):
        self.transport = transport

    def request(self, method, path, body=None):
        if method != "GET" or body is not None:
            raise ValueError("La sonda admite únicamente lecturas GET")
        return self.transport.request(method, path)

    def close(self):
        self.transport.close()


def read_credentials(stream):
    username, password, end = stream.read(8193).split(b"\0")
    if end or not username or not password or len(username) + len(password) > 8190:
        raise ValueError
    return username.decode("utf-8"), password.decode("utf-8")


def explain(exc):
    if exc.status == 401:
        return "HTTP 401 · autenticación rechazada. Comprueba usuario, contraseña y cuenta."
    if exc.status == 403:
        return "HTTP 403 · acceso denegado. Verifica los permisos RWS de lectura de la cuenta."
    if exc.status == 404:
        return "HTTP 404 · recurso ausente. Coteja RobotWare, servicio RWS y nombres en el YAML."
    if exc.status is not None:
        return f"HTTP {exc.status} · verifica el servicio RWS y consulta los eventos del IRC5."
    if str(exc) in {"RWS GET: TimeoutError", "RWS GET: ConnectTimeout", "RWS GET: ReadTimeout"}:
        return "Tiempo de espera agotado. Revisa enlace, ruta, puerto RWS y red de Podman."
    if str(exc) in {"RWS GET: ConnectionError", "RWS GET: SSLError"}:
        return "Fallo TCP o TLS desde el contenedor. Revisa ruta, puerto y certificado del IRC5."
    return "Respuesta RWS incompleta o inválida. Coteja RobotWare y la unidad mecánica en el IRC5."


def resource_names(http, path, css_class):
    from imt342_irb120_control.rws_backend import RwsError

    try:
        status, raw = http.request("GET", path)
    except OSError as exc:
        raise RwsError(f"RWS GET: {type(exc).__name__}") from None
    if status != 200:
        raise RwsError("Inventario RAPID no disponible", status)
    try:
        root = ElementTree.fromstring(raw)
        if root.tag.split("}")[-1] != "html":
            raise ValueError
        rows = root.findall(".//{*}li")
        return {
            value.text for row in rows if css_class in row.get("class", "").split()
            for value in row.findall("{*}span") if value.get("class") == "name"
        }
    except (ValueError, ElementTree.ParseError):
        raise RwsError("Inventario RAPID inválido") from None


def check_rapid(config, http, controller, backend):
    from imt342_irb120_control.rws_backend import RwsError

    try:
        tasks = resource_names(http, "/rw/rapid/tasks", "rap-task-li")
        if config.task not in tasks:
            print("Tarea RAPID ausente · coteja rws_task con el FlexPendant.")
            return
        modules = resource_names(http, f"/rw/rapid/modules?task={config.task}", "rap-module-info-li")
        if config.module not in modules:
            print("Módulo RAPID ausente · coteja rws_module y consulta al responsable de la celda.")
            return
        print("Tarea y módulo RAPID localizados.")
        if controller is None or (controller.mode, controller.motors, controller.execution) != (
            "AUTO", "motoron", "running",
        ):
            print("Estado incompatible con Control · compruébalo con el responsable de la celda.")
            return
        backend.prepare()
    except RwsError as exc:
        print("RAPID sin comprobar · revisa puntero, versión y última orden. " + explain(exc))
    else:
        print("Precomprobación RAPID correcta · se requiere autorización explícita en RViz.")


def probe(config, transport_factory=None):
    from imt342_irb120_control.rws_backend import DigestHttp, RwsError, RwsRapidBackend
    from imt342_irb120_control.supervisor import FRESHNESS

    http = ReadOnlyHttp((transport_factory or DigestHttp)(config))
    backend = RwsRapidBackend(config, http=http)
    try:
        try:
            backend.read_state()
        except RwsError as exc:
            print("Telemetría RWS no disponible · " + explain(exc))
            return 3 if exc.status in (401, 403) else 1
        print("RWS alcanzable desde el contenedor · autenticación y lectura articular correctas.")
        controller = None
        try:
            controller = backend.connect()
        except RwsError as exc:
            print("Estado IRC5 sin comprobar · " + explain(exc))
        else:
            mode = controller.mode if controller.mode in {"AUTO", "MANR", "MANF"} else "N/D"
            motors = controller.motors if controller.motors in {"motoron", "motoroff"} else "N/D"
            execution = controller.execution if controller.execution in {"running", "stopped"} else "N/D"
            print(f"IRC5 · modo {mode} · motores {motors} · RAPID {execution}.")
        check_rapid(config, http, controller, backend)
        try:
            feedback = backend.read_state()
            age = monotonic() - feedback.received_at
            if not 0 <= age <= FRESHNESS:
                print("Feedback RWS caducado · comprueba el enlace y repite la lectura.")
                return 1
        except RwsError as exc:
            print("Último feedback RWS no disponible · " + explain(exc))
            return 3 if exc.status in (401, 403) else 1
        print(f"J1–J6 recibidas por RWS · antigüedad local {age:.3f} s · "
              f"recepción UTC {datetime.now(timezone.utc).isoformat(timespec='milliseconds')}.")
        print("Preparado para Observe · ejecución física sin habilitar.")
        return 0
    finally:
        backend.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("config", "probe"))
    parser.add_argument("yaml")
    args = parser.parse_args()
    try:
        params, host, port = load_config(args.yaml)
    except (OSError, TypeError, ValueError, yaml.YAMLError):
        print("YAML RWS inválido · usa solo parámetros RWS admitidos y una URL IPv4 sin secretos.",
              file=sys.stderr)
        return 2
    if args.action == "config":
        print(host)
        print(port)
        return 0
    try:
        username, password = read_credentials(sys.stdin.buffer)
    except (ValueError, OSError):
        print("Credenciales RWS ausentes o inválidas. Usa el lanzador de demostración.", file=sys.stderr)
        return 2
    from imt342_irb120_control.rws_backend import RwsConfig

    return probe(RwsConfig(params["rws_url"], username, password, params["rws_task"],
                           params["rws_module"], params["rws_mechunit"],
                           params["rws_tolerance_rad"], params["rws_timeout_sec"],
                           params["rws_ack_timeout_sec"]))


if __name__ == "__main__":
    raise SystemExit(main())
