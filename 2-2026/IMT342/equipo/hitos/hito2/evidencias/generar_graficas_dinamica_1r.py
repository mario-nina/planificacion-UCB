"""Produce figuras y CSV reproducibles del ejemplo docente de dinámica 1R.

Ejecutar desde raíz hito2:
    python evidencias/generar_graficas_dinamica_1r.py

Fuente: S10_material_oficial.pdf, W10-S01 solucionario pp. 17 y
hito_2_guia.pdf p. 31 (páginas del PDF consolidado).
Datos didácticos; los torques NO son del ABB IRB 120 real.
"""
from __future__ import annotations

import csv
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

RAIZ = Path(__file__).resolve().parents[1]
if str(RAIZ) not in sys.path:
    sys.path.insert(0, str(RAIZ))

from src.lspb import planificar_lspb  # noqa: E402
from src.dinamica_1r import ParametrosDinamica1R, evaluar_dinamica_1r  # noqa: E402

DESTINO = Path(__file__).resolve().parent / "figuras"


def _fases(ax, tc, tf):
    for frontera in (tc, tf - tc):
        ax.axvline(frontera, linestyle="--", linewidth=1, alpha=0.6)
    ax.grid(True, alpha=0.3)


def _guardar(fig, ruta):
    fig.savefig(ruta, dpi=180, bbox_inches="tight")
    plt.close(fig)
    print(f"  {ruta.relative_to(RAIZ)} ({ruta.stat().st_size:,} bytes)")


def main():
    DESTINO.mkdir(parents=True, exist_ok=True)

    # Ejemplo OFICIAL W10-S01; no propiedades del ABB IRB 120.
    perfil = planificar_lspb([0.0], [2.0], 3.0, [1.0])
    parametros = ParametrosDinamica1R(
        inercia_kg_m2=0.25,
        masa_kg=1.0,
        longitud_cm_m=0.5,
        gravedad_m_s2=9.81,
    )
    tiempo = np.linspace(0.0, perfil.tf, 601)
    resultado = evaluar_dinamica_1r(perfil, tiempo, parametros)
    tc = float(perfil.tc[0])

    # Validación matemática independiente del ejemplo impreso del docente.
    muestra = evaluar_dinamica_1r(perfil, 2.5, parametros)
    esperado = -0.25 + 4.905 * np.sin(1.875)
    assert np.isclose(muestra.q_rad, 1.875, atol=1e-12)
    assert np.isclose(muestra.qdot_rad_s, 0.5, atol=1e-12)
    assert np.isclose(muestra.qddot_rad_s2, -1.0, atol=1e-12)
    assert np.isclose(muestra.tau_total_nm, esperado, atol=1e-12)
    assert np.isclose(resultado.qdot_rad_s[0], 0.0, atol=1e-12)
    assert np.isclose(resultado.qdot_rad_s[-1], 0.0, atol=1e-12)
    assert np.allclose(
        resultado.tau_total_nm,
        resultado.tau_inercial_nm + resultado.tau_gravitatorio_nm,
        atol=1e-12,
    )

    # Figura 1: los tres perfiles q, qdot, qddot, alineados en tiempo.
    fig, ejes = plt.subplots(3, 1, figsize=(9.2, 9), sharex=True)
    for ax, valores, etiqueta in zip(
        ejes,
        (resultado.q_rad, resultado.qdot_rad_s, resultado.qddot_rad_s2),
        ("Posición q (rad)", "Velocidad q̇ (rad/s)", "Aceleración q̈ (rad/s²)"),
    ):
        ax.plot(tiempo, valores, linewidth=1.7)
        ax.set_ylabel(etiqueta)
        _fases(ax, tc, perfil.tf)
    ejes[0].set_title("LSPB 1R — ejemplo docente W10-S01")
    ejes[-1].set_xlabel("Tiempo (s)")
    fig.tight_layout()
    _guardar(fig, DESTINO / "01_perfiles_lspb_1r.png")

    # Figura 2: contribuciones inercial y gravitatoria por separado.
    fig, ax = plt.subplots(figsize=(9.2, 5))
    ax.plot(tiempo, resultado.tau_inercial_nm, label="Inercial", linewidth=1.7)
    ax.plot(tiempo, resultado.tau_gravitatorio_nm, label="Gravitatoria", linewidth=1.7)
    _fases(ax, tc, perfil.tf)
    ax.set(xlabel="Tiempo (s)", ylabel="Torque (N·m)",
           title="Torque inercial y gravitatorio — modelo reducido 1R")
    ax.legend()
    fig.tight_layout()
    _guardar(fig, DESTINO / "02_torques_componentes_1r.png")

    # Figura 3: torque total, fases y referencia del solucionario.
    fig, ax = plt.subplots(figsize=(9.2, 5))
    ax.plot(tiempo, resultado.tau_total_nm, label="Torque total", linewidth=1.7)
    ax.scatter([2.5], [muestra.tau_total_nm], zorder=3,
               label=f"t = 2.5 s; τ = {muestra.tau_total_nm:.4f} N·m")
    _fases(ax, tc, perfil.tf)
    ax.set(xlabel="Tiempo (s)", ylabel="Torque total (N·m)",
           title="Torque total y transiciones LSPB — modelo docente 1R")
    ax.legend()
    fig.tight_layout()
    _guardar(fig, DESTINO / "03_torque_total_1r.png")

    # CSV generado desde el modelo, en LF: evita errores de git diff --check.
    ruta_csv = DESTINO / "dinamica_1r_datos.csv"
    with ruta_csv.open("w", newline="", encoding="utf-8") as archivo:
        escritor = csv.writer(archivo, lineterminator="\n")
        escritor.writerow([
            "tiempo_s", "q_rad", "qdot_rad_s", "qddot_rad_s2",
            "tau_inercial_Nm", "tau_gravitatorio_Nm", "tau_total_Nm",
        ])
        escritor.writerows(
            (f"{v:.12g}" for v in fila)
            for fila in zip(
                tiempo, resultado.q_rad, resultado.qdot_rad_s,
                resultado.qddot_rad_s2, resultado.tau_inercial_nm,
                resultado.tau_gravitatorio_nm, resultado.tau_total_nm,
            )
        )
    print(f"  {ruta_csv.relative_to(RAIZ)} ({ruta_csv.stat().st_size:,} bytes)")
    print(f"Comprobación docente t=2.5 s: τ={muestra.tau_total_nm:.6f} N·m")
    print(f"LSPB: tc={tc:g} s, tf={perfil.tf:g} s")
    crucero = evaluar_dinamica_1r(perfil, 1.5, parametros)
    assert np.isclose(crucero.tau_inercial_nm, 0.0, atol=1e-12)
    assert not np.isclose(crucero.tau_gravitatorio_nm, 0.0, atol=1e-12)
    print(f"Crucero t=1.5 s: tau_inercial={crucero.tau_inercial_nm:.6f}, "
          f"tau_gravitatorio={crucero.tau_gravitatorio_nm:.6f} N·m")
    indice_pico = int(np.argmax(np.abs(resultado.tau_total_nm)))
    print(f"Mayor |torque total| en la malla: "
          f"{np.abs(resultado.tau_total_nm[indice_pico]):.6f} N·m "
          f"(t={tiempo[indice_pico]:.3f} s; salto LSPB puede alterar el extremo)")
    print("NOTA: valores del modelo didáctico 1R; no torques reales del ABB IRB 120.")
    print("NOTA: la aceleración y el torque inercial pueden saltar en las fronteras LSPB.")


if __name__ == "__main__":
    main()
