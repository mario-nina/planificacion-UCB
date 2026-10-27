"""Validación cinemática offline del Hito 3."""

import json
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

HITO3 = Path(__file__).resolve().parents[1]
HITO2 = HITO3.parent / "hito2"

sys.path.insert(0, str(HITO2))

from src.irb120_fk import forward_kinematics_tcp
from src.jacobiano import jacobiano_geometrico
from src.limites_irb120 import violaciones_limites
from src.pieper import solve_pieper


CASOS = {
    "VAL-01": [0.30, -0.40, 0.20, 0.30, -0.20, 0.80],
    "VAL-02": [0.30, -0.35, 0.25, 0.40, 0.10, -0.60],
    "VAL-03": [0.30, -0.35, 0.25, 0.40, 0.01, -0.60],
    "SINGULAR": [0.30, -0.35, 0.25, 0.40, 0.00, -0.60],
}

PASO_FD = 1e-6
LONGITUD_REF = 1.0


def error_pose(T_ref, T_cal):
    ep = np.linalg.norm(T_ref[:3, 3] - T_cal[:3, 3])

    R = T_ref[:3, :3].T @ T_cal[:3, :3]
    coseno = (np.trace(R) - 1.0) / 2.0
    seno = 0.5 * np.linalg.norm([
        R[2, 1] - R[1, 2],
        R[0, 2] - R[2, 0],
        R[1, 0] - R[0, 1],
    ])

    eR = np.arctan2(seno, coseno)

    return float(ep), float(eR)


def jacobiano_fd(q):
    """Jacobiano espacial por diferencias centrales de FK."""
    J = np.zeros((6, 6))
    R0 = forward_kinematics_tcp(q)[:3, :3]

    for i in range(6):
        delta = np.zeros(6)
        delta[i] = PASO_FD

        Tp = forward_kinematics_tcp(q + delta)
        Tm = forward_kinematics_tcp(q - delta)

        J[:3, i] = (
            Tp[:3, 3] - Tm[:3, 3]
        ) / (2.0 * PASO_FD)

        Rdot = (
            Tp[:3, :3] - Tm[:3, :3]
        ) / (2.0 * PASO_FD)

        W = Rdot @ R0.T
        W = (W - W.T) / 2.0

        J[3:, i] = [
            W[2, 1],
            W[0, 2],
            W[1, 0],
        ]

    return J


def normalizar_jacobiano(J):
    """Convención: [v/L_ref, omega]."""
    Jn = J.copy()
    Jn[:3, :] /= LONGITUD_REF
    return Jn


def evaluar(nombre, valores):
    q = np.asarray(valores, dtype=float)

    violaciones = violaciones_limites(q)

    T = forward_kinematics_tcp(q)
    J = jacobiano_geometrico(q, target_frame="tcp")
    Jfd = jacobiano_fd(q)

    Jn = normalizar_jacobiano(J)
    Jfdn = normalizar_jacobiano(Jfd)

    eJ = np.linalg.norm(Jn - Jfdn, ord="fro")
    eJ /= max(1.0, np.linalg.norm(Jfdn, ord="fro"))

    s = np.linalg.svd(Jn, compute_uv=False)
    tolerancia_rango = 1e-9 * s[0]

    rango = int(np.count_nonzero(s > tolerancia_rango))

    condicion = (
        float(s[0] / s[-1])
        if rango == 6
        else None
    )

    datos = {
        "q_rad": q.tolist(),
        "limites_cumplidos": len(violaciones) == 0,
        "violaciones": [str(x) for x in violaciones],
        "marco_objetivo": "tcp",
        "rango_jacobiano": rango,
        "sigma_min_normalizado": float(s[-1]),
        "condicion_normalizada": condicion,
        "eJ_relativo": float(eJ),
    }

    if nombre in ("VAL-01", "VAL-02"):
        resultado = solve_pieper(T, target_frame="tcp")

        admisibles = [
            sol for sol in resultado.solutions
            if not violaciones_limites(sol.q)
        ]

        datos["soluciones_ik"] = len(resultado.solutions)
        datos["soluciones_admisibles"] = len(admisibles)

        if admisibles:
            errores = [
                error_pose(
                    T,
                    forward_kinematics_tcp(
                        np.asarray(sol.q, dtype=float)
                    ),
                )
                for sol in admisibles
            ]

            datos["errores_soluciones_admisibles"] = [
                {"ep_m": ep, "eR_rad": eR}
                for ep, eR in errores
            ]
            datos["ep_max_m"] = max(x[0] for x in errores)
            datos["eR_max_rad"] = max(x[1] for x in errores)
        else:
            datos["errores_soluciones_admisibles"] = []
            datos["ep_max_m"] = None
            datos["eR_max_rad"] = None

    return datos


def main():
    resultados = {
        "fecha_utc": datetime.now(timezone.utc).isoformat(),
        "modo": "offline",
        "robot_modelo": "ABB IRB 120 academico",
        "python": sys.version.split()[0],
        "numpy": np.__version__,
        "paso_diferencias_finitas_rad": PASO_FD,
        "longitud_referencia_m": LONGITUD_REF,
        "criterio_rango_relativo": 1e-9,
        "casos": {},
    }

    for nombre, q in CASOS.items():
        datos = evaluar(nombre, q)
        resultados["casos"][nombre] = datos

        print(f"\n========== {nombre} ==========")
        print("Límites:", datos["limites_cumplidos"])
        print("Rango:", datos["rango_jacobiano"])
        print("Sigma mínimo:", datos["sigma_min_normalizado"])
        print("Condición:", datos["condicion_normalizada"])
        print("eJ:", datos["eJ_relativo"])

        if "soluciones_ik" in datos:
            print("IK totales:", datos["soluciones_ik"])
            print("IK admisibles:", datos["soluciones_admisibles"])
            print("ep máximo:", datos["ep_max_m"])
            print("eR máximo:", datos["eR_max_rad"])

    salida = HITO3 / "evidencias" / "validacion_cinematica.json"
    salida.write_text(
        json.dumps(
            resultados,
            indent=2,
            ensure_ascii=False,
            allow_nan=False,
        ) + "\n"
    )

    print("\nResultados guardados en:", salida)


if __name__ == "__main__":
    main()
