"""Offline CLI for reviewing P1 drafts."""
import argparse
import json
from pathlib import Path
import sys

PLANNING = Path(__file__).resolve().parents[3] / "ros2_ws/src/imt342_irb120_planning"
if PLANNING.is_dir():
    sys.path.insert(0, str(PLANNING))

from .module import DRAFT_MODULE, SERVER_MODULE, render_module
from .planning import build_plan, load_plan, simulate_completion


def main():
    parser = argparse.ArgumentParser(description="Exporta un borrador RAPID offline en JSON.")
    inputs = parser.add_mutually_exclusive_group(required=True)
    inputs.add_argument("--inicio", type=float, nargs=6, metavar="rad")
    inputs.add_argument("--cargar", type=Path, metavar="ARCHIVO.json")
    parser.add_argument("--destino", type=float, nargs=6, metavar="rad")
    parser.add_argument("--perfil", choices=("lspb", "quintic"))
    parser.add_argument("--duracion", type=float, metavar="s")
    parser.add_argument("--intervalos", type=int)
    parser.add_argument("--velocidad-maxima", type=float, nargs=6, metavar="rad/s")
    parser.add_argument("--aceleracion-maxima", type=float, nargs=6, metavar="rad/s2")
    parser.add_argument("--jerk-maximo", type=float, nargs=6, metavar="rad/s3")
    parser.add_argument("--simular", action="store_true", help="Genera acuses de llegada sintéticos.")
    parser.add_argument("--mod", type=Path, metavar="ARCHIVO.mod",
                        help=f"Escribe el borrador virtual {DRAFT_MODULE}.")
    args = parser.parse_args()
    options = dict(profile=args.perfil, duration=args.duracion, intervals=args.intervalos, max_jerk=args.jerk_maximo)
    if args.velocidad_maxima is not None:
        options["max_velocity"] = args.velocidad_maxima
    if args.aceleracion_maxima is not None:
        options["max_acceleration"] = args.aceleracion_maxima
    try:
        options = {name: value for name, value in options.items() if value is not None}
        if args.cargar is not None:
            if args.destino is not None or options:
                raise ValueError("Use --cargar sin otros argumentos de planificación")
            with args.cargar.open(encoding="utf-8") as handle:
                raw = handle.read(256 * 1024 + 1)
            if len(raw) > 256 * 1024:
                raise ValueError("El registro supera el tamaño máximo admitido")
            draft = load_plan(json.loads(raw))
        else:
            if args.destino is None:
                raise ValueError("Indique --destino junto a --inicio")
            draft = build_plan(args.inicio, args.destino, **options)
        output = draft.record()
        if args.mod is not None:
            if args.mod.suffix.lower() != ".mod":
                raise ValueError("El archivo RAPID debe tener extensión .mod")
            if args.mod.stem.casefold() == SERVER_MODULE.casefold():
                raise ValueError(f"Use {DRAFT_MODULE}.mod para distinguir el borrador del servidor RWS")
            with args.mod.open("x", encoding="ascii", newline="\n") as handle:
                handle.write(render_module(draft))
        if args.simular:
            output["simulation"] = simulate_completion(draft)
    except (OSError, ValueError, OverflowError) as exc:
        parser.error(str(exc))
    print(json.dumps(output, ensure_ascii=False, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
