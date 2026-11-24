#!/usr/bin/env bash
set -euo pipefail

usage() {
    cat <<'HELP'
Uso: rviz_fedora.sh [offline|observe|control]
     rviz_fedora.sh status [--id|--session]
     rviz_fedora.sh stop [--offline-id ID]
     rviz_fedora.sh {observe|control} --backend {rws|rws_trajectory} --config ARCHIVO --network RED

Offline es el modo predeterminado. Observe y control sin opciones usan ROS externo
para pruebas con dobles. Los tres arranques de prueba usan Podman sin red.
El arranque con RWS queda reservado a una sesión de laboratorio supervisada.
HELP
}

fail() {
    printf 'RViz IMT-342: %s\n' "$*" >&2
    exit 1
}

mode="${1:-offline}"
if [[ $# -gt 0 ]]; then shift; fi
case "$mode" in
    -h|--help|help) usage; exit 0 ;;
    offline|observe|control|status|stop) ;;
    *) usage >&2; fail "modo desconocido: $mode" ;;
esac

name="imt342-ec34-rviz"
label="org.imt342.project"
label_value="eci34-irb120"
mode_label="org.imt342.mode"
command -v podman >/dev/null || fail "Podman no está disponible en PATH."
[[ $(podman info --format '{{.Host.Security.Rootless}}') == true ]] ||
    fail "Abre una terminal de tu usuario de Fedora con Podman rootless."

if [[ "$mode" == status || "$mode" == stop ]]; then
    detail=""
    offline_id=""
    case "$mode:${1:-}:$#" in
        status::0|stop::0) ;;
        status:--id:1|status:--session:1) detail="$1" ;;
        stop:--offline-id:2)
            offline_id="$2"
            [[ "$offline_id" =~ ^[0-9a-f]{64}$ ]] || fail "Identificador Offline inválido."
            ;;
        *) fail "$mode no recibe esos argumentos." ;;
    esac
    if podman container exists "$name"; then
        read -r container_id owner < <(podman inspect --type container \
            --format '{{.Id}} {{index .Config.Labels "org.imt342.project"}}' "$name") ||
            fail "No se pudo identificar el contenedor $name."
        [[ "$owner" == "$label_value" && "$container_id" =~ ^[0-9a-f]{64}$ ]] ||
            fail "El contenedor $name pertenece a otra aplicación."
    else
        result=$?
        [[ "$result" == 1 ]] || fail "No se pudo consultar el estado de Podman."
        if [[ -n "$detail" ]]; then exit 3; fi
        [[ -z "$offline_id" ]] || fail "La sesión Offline original ya no existe."
        printf 'RViz IMT-342 no está en ejecución.\n'
        exit 0
    fi
    if [[ "$detail" == --id ]]; then
        printf '%s\n' "$container_id"
    elif [[ "$detail" == --session ]]; then
        session_info="$(podman inspect --type container \
            --format '{{index .Config.Labels "org.imt342.mode"}} {{.HostConfig.NetworkMode}}' "$container_id")" ||
            fail "No se pudo consultar el modo de la sesión."
        session_mode="${session_info%% *}"
        case "$session_mode" in
            offline) [[ "$session_info" == "offline none" ]] || session_mode=unknown ;;
            observe|control) ;;
            *) session_mode=unknown ;;
        esac
        printf '%s %s\n' "$container_id" "$session_mode"
    elif [[ "$mode" == stop ]]; then
        if [[ -n "$offline_id" ]]; then
            [[ "$container_id" == "$offline_id" ]] || fail "La sesión Offline original cambió."
            session_info="$(podman inspect --type container \
                --format '{{index .Config.Labels "org.imt342.mode"}} {{.HostConfig.NetworkMode}}' "$container_id")" ||
                fail "No se pudo comprobar el modo de la sesión."
            [[ "$session_info" == "offline none" ]] ||
                fail "La sesión original no está identificada como Offline aislada."
        fi
        # Target the validated ID, even if a new session acquires the name.
        podman kill --signal INT "$container_id" >/dev/null ||
            fail "No se pudo solicitar el cierre de la sesión."
        for ((attempt = 0; attempt < 50; attempt++)); do
            if podman container exists "$container_id"; then
                sleep .2
            else
                result=$?
                [[ "$result" == 1 ]] || fail "No se pudo verificar el cierre en Podman."
                printf 'RViz IMT-342 se cerró.\n'
                exit 0
            fi
        done
        exec podman stop --time 3 "$container_id"
    else
        podman inspect --type container \
            --format 'RViz IMT-342: {{.State.Status}} · red: {{.HostConfig.NetworkMode}}' "$container_id"
    fi
    exit 0
fi

check_only=false
replace_offline=""
rws_stdin=false
backend="external"
config=""
network=""
while [[ $# -gt 0 ]]; do
    case "$1" in
        --check) check_only=true; shift; continue ;;
        --rws-stdin) rws_stdin=true; shift; continue ;;
    esac
    [[ $# -ge 2 && -n "$2" ]] || fail "Falta el valor de $1."
    case "$1" in
        --backend) backend="$2" ;;
        --config) config="$2" ;;
        --network) network="$2" ;;
        --replace-offline)
            replace_offline="$2"
            [[ "$replace_offline" =~ ^[0-9a-f]{64}$ ]] || fail "Identificador Offline inválido."
            ;;
        *) fail "Opción desconocida: $1" ;;
    esac
    shift 2
done

if [[ -n "$replace_offline" ]] && { ! $check_only || [[ "$mode" != observe || "$backend" != rws ]]; }; then
    fail "--replace-offline solo se admite en la comprobación de Observe."
fi
backend_args=()
launch_args=("mode:=$mode" "backend:=$backend" "gui:=true")
case "$backend" in
    external)
        [[ -z "$config$network" ]] || fail "La sesión con backend externo se ejecuta sin red y usa su configuración instalada."
        network="none"
        if [[ "$mode" == offline ]]; then
            launch_args[1]="backend:=rws"
        fi
        ;;
    rws|rws_trajectory)
        [[ "$mode" != offline ]] || fail "Para Offline usa el backend externo sin opciones."
        [[ -n "$config" && -n "$network" ]] ||
            fail "Para $backend especifica el YAML de la sesión y la red Podman del laboratorio."
        [[ "$network" =~ ^[a-zA-Z0-9][a-zA-Z0-9_.-]*$ && "$network" != host && "$network" != none ]] ||
            fail "Usa el nombre de una red Podman de laboratorio."
        podman network exists "$network" || fail "No existe la red Podman $network."
        config="$(realpath -e -- "$config")" || fail "El YAML de la sesión no existe."
        [[ -f "$config" && -r "$config" ]] || fail "El YAML de la sesión no se puede leer."
        backend_args+=(--volume "$config:/tmp/imt342-backends.yaml:ro")
        launch_args+=("backend_config:=/tmp/imt342-backends.yaml")
        if ! $check_only && ! $rws_stdin; then
            [[ -n "${IMT342_RWS_USERNAME:-}" && -n "${IMT342_RWS_PASSWORD:-}" ]] ||
                fail "Exporta IMT342_RWS_USERNAME e IMT342_RWS_PASSWORD desde la terminal del operador."
            backend_args+=(--env IMT342_RWS_USERNAME --env IMT342_RWS_PASSWORD)
        fi
        ;;
    *) fail "Backend desconocido: $backend" ;;
esac
if $rws_stdin; then
    [[ "$backend" == rws || "$backend" == rws_trajectory ]] ||
        fail "--rws-stdin requiere un backend del IRC5."
    backend_args+=(--interactive)
fi

project="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
workspace="$project/ros2_ws"
evidence="$project/evidence"
image="${IMT342_JAZZY_IMAGE:-localhost/ros-jazzy-imt342-rws:latest}"
domain="${IMT342_ROS_DOMAIN_ID:-142}"
[[ "$domain" =~ ^[0-9]{1,3}$ ]] && (( 10#$domain <= 232 )) ||
    fail "IMT342_ROS_DOMAIN_ID debe estar entre 0 y 232."
domain="$((10#$domain))"
[[ -r "$workspace/install/setup.bash" ]] ||
    fail "Falta el workspace compilado. Ejecuta scripts/build_jazzy_offline.sh con IMT342_JAZZY_IMAGE=$image."
podman image exists "$image" || fail "No existe la imagen local $image."
if [[ -n "$replace_offline" ]]; then
    current="$("$0" status --session)" || fail "No se pudo comprobar la sesión Offline original."
    [[ "$current" == "$replace_offline offline" ]] ||
        fail "La sesión Offline original cambió."
elif podman container exists "$name"; then
    fail "Ya existe $name. Consulta 'status' o cierra la sesión con 'stop'."
else
    [[ "$?" == 1 ]] || fail "No se pudo consultar el estado de Podman."
fi
[[ "${DISPLAY:-}" =~ ^(:|unix:)([0-9]+)(\.[0-9]+)?$ ]] ||
    fail "DISPLAY no señala a XWayland local. Abre este comando en una terminal gráfica de Fedora."
display_number="${BASH_REMATCH[2]}"
display=":${display_number}${BASH_REMATCH[3]}"
[[ -S "/tmp/.X11-unix/X$display_number" ]] ||
    fail "No se encuentra el socket X11 de DISPLAY=$display."

command -v xprop >/dev/null || fail "Se requiere xprop para verificar la sesión X11."
shopt -s nullglob
authorities=("${XAUTHORITY:-}")
if [[ -n "${XDG_RUNTIME_DIR:-}" ]]; then
    authorities+=("$XDG_RUNTIME_DIR"/.mutter-Xwaylandauth.* "$XDG_RUNTIME_DIR"/xauth_*)
fi
authorities+=("$HOME/.Xauthority")
authority=""
for candidate in "${authorities[@]}"; do
    [[ -n "$candidate" && -f "$candidate" && -r "$candidate" ]] || continue
    if DISPLAY="$display" XAUTHORITY="$candidate" xprop -root _NET_SUPPORTING_WM_CHECK >/dev/null 2>&1; then
        authority="$(realpath -e -- "$candidate")"
        break
    fi
done
[[ -n "$authority" ]] ||
    fail "No se pudo autenticar a XWayland. Revisa XAUTHORITY desde la terminal de tu sesión gráfica."

if $check_only; then
    printf 'RViz IMT-342 · imagen, workspace, montajes y XWayland listos.\n'
    exit 0
fi
mkdir -p -- "$evidence" || fail "No se pudo crear el directorio de evidencias: $evidence."
[[ -w "$evidence" ]] || fail "El directorio de evidencias no admite escritura: $evidence."
printf 'RViz IMT-342 · modo %s · backend %s · red %s\n' "$mode" "$backend" "$network"
exec podman run --rm --pull=never --name "$name" --label "$label=$label_value" --label "$mode_label=$mode" \
    --network "$network" --userns=keep-id --user "$(id -u):$(id -g)" \
    --security-opt label=disable --hostname "$(hostname)" \
    --volume "$workspace:/work:ro" --volume "$evidence:/evidencia:rw" \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:ro --volume "$authority:/tmp/imt342-xauth:ro" \
    --env "DISPLAY=$display" --env XAUTHORITY=/tmp/imt342-xauth --env QT_QPA_PLATFORM=xcb \
    --env QT_X11_NO_MITSHM=1 --env LIBGL_ALWAYS_SOFTWARE=1 --env "ROS_DOMAIN_ID=$domain" \
    "${backend_args[@]}" --workdir /work "$image" bash -ec '
        if [[ "$1" == true ]]; then
            set +x
            IFS= read -r -d "" IMT342_RWS_USERNAME &&
                IFS= read -r -d "" IMT342_RWS_PASSWORD &&
                [[ -n "$IMT342_RWS_USERNAME" && -n "$IMT342_RWS_PASSWORD" ]] ||
                { echo "Credenciales RWS de entrada incompletas." >&2; exit 1; }
            export IMT342_RWS_USERNAME IMT342_RWS_PASSWORD
        fi
        shift
        source /opt/ros/jazzy/setup.bash
        source /work/install/setup.bash
        exec ros2 launch imt342_irb120_bringup session.launch.py "$@"
    ' imt342 "$rws_stdin" "${launch_args[@]}"
