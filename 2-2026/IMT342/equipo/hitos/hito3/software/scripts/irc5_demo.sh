#!/usr/bin/env bash
set +x
set -euo pipefail

username="${IMT342_RWS_USERNAME:-}"
password="${IMT342_RWS_PASSWORD:-}"
export -n username password
unset IMT342_RWS_USERNAME IMT342_RWS_PASSWORD

project="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
launcher="$project/scripts/rviz_fedora.sh"
probe="$project/scripts/irc5_probe.py"
image="${IMT342_JAZZY_IMAGE:-localhost/ros-jazzy-imt342-rws:latest}"
config="${IMT342_IRC5_CONFIG:-$project/../irc5_observe.yaml}"
network="${IMT342_IRC5_NETWORK:-podman}"
replace_offline=""
action="${1:-}"
if (($#)); then shift; fi

usage() {
    cat <<'HELP'
Uso: irc5_demo.sh {preflight|observe|status|diagnose|stop|offline|dry-run|credentials}
     irc5_demo.sh {preflight|observe|diagnose|dry-run|credentials} [--network RED] [--config YAML]
     irc5_demo.sh observe --replace-offline ID

preflight y dry-run comprueban recursos locales, interfaz y ruta sin contactar al IRC5.
observe recupera las credenciales, comprueba RWS mediante lecturas e inicia RViz Solo lectura.
diagnose comprueba TCP y RWS de solo lectura, sin iniciar RViz.
offline inicia la simulación con red none.
credentials guarda la cuenta en el llavero de Fedora sin contactar al IRC5.
HELP
}

fail() {
    printf 'Demo IRC5 · %s\n' "$*" >&2
    exit 1
}

case "$action" in
    -h|--help|help) usage; exit 0 ;;
    status|stop|offline) (($# == 0)) || fail "$action no recibe opciones." ;;
    preflight|observe|diagnose|dry-run|credentials)
        while (($#)); do
            (($# >= 2)) && [[ -n "$2" ]] || fail "Falta el valor de $1."
            case "$1" in
                --config) config="$2" ;;
                --network) network="$2" ;;
                --replace-offline)
                    [[ "$action" == observe && "$2" =~ ^[0-9a-f]{64}$ ]] ||
                        fail "--replace-offline requiere observe y un ID válido."
                    replace_offline="$2"
                    ;;
                *) fail "Opción desconocida: $1." ;;
            esac
            shift 2
        done
        ;;
    *) usage >&2; exit 2 ;;
esac

read_endpoint() {
    local endpoint
    local -a fields
    config="$(realpath -e -- "$config")" || fail "YAML no encontrado. Verifica --config."
    [[ -f "$config" && -r "$config" ]] || fail "El YAML debe ser un archivo legible."
    endpoint="$(python3 "$probe" config "$config")" || fail "Corrige la configuración local antes de continuar."
    mapfile -t fields <<<"$endpoint"
    [[ ${#fields[@]} -eq 2 && "${fields[0]}" =~ ^[0-9.]+$ && "${fields[1]}" =~ ^[0-9]+$ ]] ||
        fail "Destino inválido en el YAML."
    host="${fields[0]}"
    port="${fields[1]}"
}

check_probe_inputs() {
    local rootless
    for command in podman python3 ip timeout; do
        command -v "$command" >/dev/null || fail "Falta $command en Fedora."
    done
    python3 -c 'import yaml' 2>/dev/null ||
        fail "Falta PyYAML en Fedora. Verifica el paquete python3-pyyaml antes del laboratorio."
    rootless="$(podman info --format '{{.Host.Security.Rootless}}')" ||
        fail "Podman no inició. Revisa el entorno rootless desde la terminal del operador."
    [[ "$rootless" == true ]] || fail "Usa Podman rootless desde el usuario de Fedora."
    podman image exists "$image" || fail "Falta la imagen local $image."
    [[ "$network" =~ ^[a-zA-Z0-9][a-zA-Z0-9_.-]*$ && "$network" != host && "$network" != none ]] ||
        fail "Indica una red Podman nombrada del laboratorio."
    podman network exists "$network" ||
        fail "La red Podman $network no existe o no se puede consultar. Confirma la red de la celda."
    read_endpoint
    printf 'Destino RWS · %s:%s · red Podman %s\n' "$host" "$port" "$network"
}

check_route() {
    python3 - "$host" <<'PY'
import json
import os
from pathlib import Path
import subprocess
import sys

def ip(*args):
    result = subprocess.run(["ip", "-j", *args], capture_output=True, text=True,
                            env=dict(os.environ, LC_ALL="C"))
    if result.returncode:
        error = result.stderr
        if "Operation not permitted" in error or "Permission denied" in error:
            raise ValueError("Fedora no permite consultar interfaces o rutas en este entorno.")
        raise ValueError("Ruta o interfaz no disponible. Revisa el cable y la configuración IP de la celda.")
    return json.loads(result.stdout)

try:
    route, *_ = ip("-4", "route", "get", sys.argv[1])
    interface = route.get("dev", "")
    source = route.get("prefsrc") or route.get("src")
    if not interface or "/" in interface or interface == "lo":
        raise ValueError("La ruta apunta a una interfaz local o inválida. Verifica la ruta de la celda.")
    link, *_ = ip("link", "show", "dev", interface)
    sysfs = Path("/sys/class/net") / interface
    if link.get("link_type") != "ether" or any((sysfs / key).exists() for key in ("wireless", "phy80211")):
        raise ValueError(f"La ruta sale por {interface}, que no es Ethernet de laboratorio.")
    flags = link.get("flags", ())
    if "UP" not in flags:
        raise ValueError(f"{interface} está desactivada. Revisa el perfil autorizado de Fedora.")
    if "LOWER_UP" not in flags:
        raise ValueError(f"{interface} no tiene enlace. Revisa el cable y el puerto de la celda.")
    address, *_ = ip("-4", "address", "show", "dev", interface)
    if not source or not any(record.get("local") == source for record in address.get("addr_info", ())):
        raise ValueError(f"{interface} carece de una IPv4 de origen válida. Revisa el perfil de la celda.")
    gateway = f" mediante {route['gateway']}" if route.get("gateway") else ""
    print(f"Fedora · Ethernet {interface} · origen {source} · ruta a {sys.argv[1]}{gateway}")
except (IndexError, ValueError) as exc:
    print(f"Preflight de red · {exc}", file=sys.stderr)
    sys.exit(1)
PY
}

check_launcher() {
    local package=imt342_irb120_bringup
    local installed="$project/ros2_ws/install/$package/lib/$package/diagnostico.py"
    local source="$project/ros2_ws/src/$package/scripts/diagnostico.py"
    [[ -x "$installed" ]] ||
        fail "Falta el diagnóstico instalado. Ejecuta bash scripts/build_jazzy_offline.sh."
    if [[ -f "$source" ]] && ! cmp -s -- "$source" "$installed"; then
        fail "El diagnóstico instalado difiere de la fuente. Ejecuta bash scripts/build_jazzy_offline.sh."
    fi
    local -a replacement=()
    [[ -z "$replace_offline" ]] || replacement=(--replace-offline "$replace_offline")
    "$launcher" observe --backend rws --config "$config" --network "$network" --check "${replacement[@]}"
}

check_host_tcp() {
    python3 - "$host" "$port" <<'PY'
import socket
import sys

try:
    with socket.create_connection((sys.argv[1], int(sys.argv[2])), timeout=2.5):
        print("Fedora · puerto TCP RWS accesible")
except OSError as exc:
    print(f"Fedora · puerto TCP RWS inaccesible ({type(exc).__name__}). "
          "Verifica enlace, ruta, dirección y servicio de la celda.", file=sys.stderr)
    sys.exit(1)
PY
}

prompt_credentials() {
    IFS= read -rs -p "Usuario RWS (oculto): " username </dev/tty ||
        fail "No se pudo leer el usuario. Abre una terminal del operador."
    printf '\n' >&2
    IFS= read -rs -p "Contraseña RWS (oculta): " password </dev/tty ||
        fail "No se pudo leer la contraseña. Abre una terminal del operador."
    printf '\n' >&2
    [[ -n "$username" && -n "$password" ]] || fail "Credenciales RWS vacías."
}

credentials() {
    if [[ -n "$username" || -n "$password" ]]; then
        [[ -n "$username" && -n "$password" ]] || fail "Define ambas variables RWS o retíralas para usar el llavero."
        return
    fi
    local account
    if command -v secret-tool >/dev/null &&
        account="$(secret-tool lookup application imt342-rws controller "$host:$port" 2>/dev/null)" &&
        [[ "$account" == *$'\t'* ]]; then
        username="${account%%$'\t'*}"
        password="${account#*$'\t'}"
        if [[ -n "$username" && -n "$password" && "$password" != *$'\n'* ]]; then
            return
        fi
    fi
    if [[ "${IMT342_RWS_KEYRING_ONLY:-0}" == 1 ]]; then
        fail "Llavero RWS bloqueado o sin cuenta para esta celda. Desbloquéalo o configura la cuenta con ./scripts/irc5_demo.sh credentials."
    fi
    printf 'Llavero RWS bloqueado o sin cuenta para esta celda. Desbloquéalo o usa la acción credentials.\n' >&2
    prompt_credentials
}

store_credentials() {
    command -v secret-tool >/dev/null || fail "Falta secret-tool. Usa las variables RWS desde una sesión local."
    read_endpoint
    prompt_credentials
    [[ "$username" != *$'\t'* ]] || fail "El usuario RWS debe escribirse sin tabulaciones."
    if ! printf '%s\t%s' "$username" "$password" | secret-tool store \
        --label='IMT342 IRC5 · cuenta RWS' application imt342-rws controller "$host:$port"; then
        fail "No se pudo guardar la cuenta. Desbloquea el llavero y repite credentials."
    fi
    printf 'Cuenta RWS guardada en el llavero de Fedora.\n'
}

credential_input() {
    printf '%s\0%s\0' "$username" "$password"
}

read_controller() {
    local result=0
    timeout 25s podman run --rm --pull=never --interactive \
        --network "$network" --userns=keep-id --user "$(id -u):$(id -g)" \
        --security-opt label=disable \
        --volume "$project/scripts:/opt/imt342-demo:ro" \
        --volume "$project/ros2_ws/src:/opt/imt342-src:ro" \
        --volume "$config:/tmp/imt342-backends.yaml:ro" \
        --env PYTHONPATH=/opt/imt342-src/imt342_irb120_control:/opt/imt342-src/imt342_irb120_planning \
        "$image" python3 /opt/imt342-demo/irc5_probe.py probe /tmp/imt342-backends.yaml \
        < <(credential_input) || result=$?
    case "$result" in
        0) return 0 ;;
        124|137) fail "Tiempo límite del diagnóstico. Revisa la red del contenedor y el servicio RWS." ;;
        125|126|127) fail "No se pudo iniciar la sonda en Podman. Revisa imagen, montajes y red." ;;
        3) fail "Autenticación o permisos RWS rechazados. Revisa la cuenta autorizada y actualízala con credentials." ;;
        *) fail "La sonda de lectura no confirmó telemetría. Corrige el diagnóstico indicado." ;;
    esac
}

print_session() {
    local container="$1"
    shift
    timeout 12s podman exec "$container" bash -ec '
        source /opt/ros/jazzy/setup.bash
        source /work/install/setup.bash
        exec ros2 run imt342_irb120_bringup diagnostico.py "$@"
    ' imt342 "$@"
}

status() {
    local container result=0
    container="$("$launcher" status --id)" || result=$?
    case "$result" in
        0) ;;
        3) printf 'Demo IRC5 · sin sesión RViz.\n'; return 0 ;;
        *) return "$result" ;;
    esac
    podman inspect --type container \
        --format 'Contenedor · {{.State.Status}} · red {{.HostConfig.NetworkMode}}' "$container"
    print_session "$container" ||
        fail "No se recibió estado ROS de la sesión. Revisa el arranque en la terminal de RViz."
}

watch_startup() {
    local deadline=$((SECONDS + 30)) container="" summary=""
    while ((SECONDS < deadline)); do
        if container="$("$launcher" status --id 2>/dev/null)"; then
            if summary="$(print_session "$container" --expect-observe --wait 5)"; then
                printf '\nDemo IRC5 · Observe listo con feedback RWS vigente.\n%s\n' "$summary"
                return
            fi
        fi
        sleep .5
    done
    if [[ -n "$summary" ]]; then printf '%s\n' "$summary"; fi
    printf 'Demo IRC5 · Arranque sin confirmación de telemetría vigente. Usa status o diagnose.\n' >&2
    return 1
}

watcher_running() {
    local pid
    while IFS= read -r pid; do
        if [[ "$pid" == "$watcher" ]]; then return 0; fi
    done <<<"$(jobs -pr)"
    return 1
}

stop_watcher() {
    if watcher_running; then kill "$watcher" 2>/dev/null || true; fi
}

case "$action" in
    credentials) store_credentials ;;
    stop) exec "$launcher" stop ;;
    status) status ;;
    offline)
        unset username password
        printf 'Demo IRC5 · SIMULACIÓN OFFLINE · red none\n'
        exec "$launcher" offline
        ;;
    preflight|dry-run)
        if [[ "$action" == dry-run ]]; then
            printf 'Plan · comprobar Fedora → TCP → RWS GET desde Podman → RViz Solo lectura → feedback ROS\n'
        fi
        check_probe_inputs
        check_launcher
        check_route
        printf 'Preparación local correcta. Alcance TCP, autenticación y mediciones quedan para la celda.\n'
        ;;
    diagnose)
        check_probe_inputs
        check_route
        host_tcp=0
        check_host_tcp || host_tcp=$?
        credentials
        read_controller
        exit "$host_tcp"
        ;;
    observe)
        check_probe_inputs
        check_launcher
        check_route
        check_host_tcp || fail "Observación no iniciada. Usa diagnose para comparar el acceso desde Podman."
        credentials
        read_controller
        if [[ -n "$replace_offline" ]]; then
            "$launcher" stop --offline-id "$replace_offline"
        fi
        printf 'Iniciando RViz en Solo lectura. Esperando feedback de la sesión ROS…\n'
        (unset username password; watch_startup) &
        watcher=$!
        trap stop_watcher EXIT
        exec {rws_input}< <(credential_input)
        unset username password
        if ! "$launcher" observe --backend rws --config "$config" --network "$network" --rws-stdin \
            <&"$rws_input"; then
            fail "RViz terminó con error. Revisa la salida de ROS y ejecuta status."
        fi
        exec {rws_input}<&-
        if watcher_running || ! wait "$watcher"; then
            fail "RViz cerró sin confirmar una lectura vigente en la sesión. Usa diagnose."
        fi
        ;;
esac
