#!/usr/bin/env bash
set +x
set -euo pipefail

project="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
demo="$project/scripts/irc5_demo.sh"
launch="$project/scripts/rviz_fedora.sh"
title="IMT-342 · ABB IRB 120"

case "${1:-}" in
    install)
        (($# == 1)) || exit 2
        python3 - "$project/scripts/imt342_launcher.sh" <<'PY'
import os
from pathlib import Path
import sys
from tempfile import NamedTemporaryFile

path = sys.argv[1]
if any(char in path for char in "\r\n\t"):
    raise SystemExit("La ruta del proyecto contiene caracteres de control.")
# Desktop Entry primero decodifica sus escapes y luego los de Exec.
argument = "".join(("\\" + char) if char in '\\"$\x60' else char for char in path)
argument = argument.replace("%", "%%").replace("\\", "\\\\")
location = Path(os.environ.get("XDG_DATA_HOME") or Path.home() / ".local/share") / "applications"
location.mkdir(parents=True, exist_ok=True)
target = location / "org.imt342.irb120.desktop"
with NamedTemporaryFile(dir=location, mode="w", encoding="utf-8", delete=False) as file:
    file.write(f"""[Desktop Entry]
Type=Application
Name=IMT-342 · ABB IRB 120
Comment=Abre RViz en simulación o Solo lectura con IRC5
Exec="{argument}"
Icon=applications-engineering
Terminal=false
Categories=Education;
""")
    temporary = Path(file.name)
temporary.replace(target)
print(target)
PY
        exit
        ;;
    -h|--help)
        printf 'Uso: %s [install]\n' "$0"
        exit
        ;;
    "") ;;
    *) printf 'Opción desconocida. Usa %s --help.\n' "$0" >&2; exit 2 ;;
esac

command -v zenity >/dev/null || {
    printf 'Zenity no está disponible. Usa ./scripts/irc5_demo.sh offline u observe.\n' >&2
    exit 1
}

message() {
    zenity "$1" --title="$title" --no-markup --width=460 --text="$2"
}

choice="$(zenity --list --title="$title" --text='Elige la sesión de RViz' \
    --width=480 --height=230 --column='Modo' --column='Descripción' \
    'Offline' 'Simulación aislada de la red' \
    'Solo lectura' 'IRC5 · sesión supervisada')" || exit 0

case "$choice" in
    Offline) mode=offline ;;
    'Solo lectura') mode=observe ;;
    *) message --error "Selección de sesión desconocida."; exit 2 ;;
esac

result=0
session="$("$launch" status --session 2>&1)" || result=$?
args=()
case "$result" in
    3) ;;
    0)
        read -r id current <<<"$session"
        if [[ "$mode" == observe && "$current" == offline && "$id" =~ ^[0-9a-f]{64}$ ]]; then
            message --question '¿Cambiar a Solo lectura? Fedora verificará Ethernet, la cuenta y telemetría RWS antes de cerrar la sesión Offline. Se abrirá otra sesión RViz.' || exit 0
            args=(--replace-offline "$id")
        else
            message --info 'Ya hay una sesión RViz gestionada. Usa esa ventana o ciérrala antes de abrir otra.'
            exit 0
        fi
        ;;
    *) message --error "$session"; exit 1 ;;
esac

log="$(mktemp)"
trap 'rm -f -- "$log"' EXIT
result=0
IMT342_RWS_KEYRING_ONLY=1 "$demo" "$mode" "${args[@]}" >"$log" 2>&1 || result=$?
# A closed isolated window can return 130 while the managed replacement starts.
if [[ "$result" == 0 || ("$mode" == offline && "$result" == 130) ]]; then
    exit 0
fi
message --error "$(tail -n 12 -- "$log")"
exit 1
