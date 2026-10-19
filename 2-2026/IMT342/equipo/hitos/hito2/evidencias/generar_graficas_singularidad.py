"""Genera evidencia reproducible del ensayo de singularidad de muñeca, IMT-342.

Ejecutar con Python + NumPy + Matplotlib, desde cualquier directorio:
    python evidencias/generar_graficas_singularidad.py

Los resultados proceden exclusivamente del modelo local src/singularidades.py.
El ensayo es matemático: las velocidades calculadas no son consignas de robot.
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

from src.singularidades import experimento_singularidad_muneca  # noqa: E402

DESTINO = Path(__file__).resolve().parent / "figuras"


def _acabado(ax, q5, titulo, ylabel):
    indices = np.arange(len(q5))
    ax.set_title(titulo)
    ax.set_xlabel("Secuencia de aproximación a la singularidad (q5 en rad)")
    ax.set_ylabel(ylabel)
    ax.set_xticks(indices, [f"{v:g}" for v in q5])
    ax.grid(True, alpha=0.35)
    return indices


def _guardar(fig, nombre):
    fig.text(
        0.5, 0.015,
        "Modelo académico ABB IRB 120 · q5 → 0 · Jacobiano de brida · L = 1 m · tol. relativa = 1e-9",
        ha="center", fontsize=8,
    )
    fig.savefig(DESTINO / nombre, dpi=190, bbox_inches="tight", pad_inches=0.20)
    plt.close(fig)


def generar():
    DESTINO.mkdir(parents=True, exist_ok=True)
    e = experimento_singularidad_muneca(
        target_frame="flange",
        tolerancia_relativa=1e-9,
        longitud_referencia_m=1.0,
        magnitud_twist_normalizado=0.01,
    )
    q5 = e.angulos_q5_rad
    xs = np.arange(q5.size)
    if not np.array_equal(e.rangos, np.array([6] * (len(xs) - 1) + [5])):
        raise RuntimeError("El ensayo no produjo la secuencia de rangos esperada; revisar el modelo.")
    if not np.all(np.isfinite(e.normas_qdot[:-1])) or not np.isnan(e.normas_qdot[-1]):
        raise RuntimeError("El cálculo de velocidades no coincide con la condición singular esperada.")

    # Guardar la tabla de resultados computados, sin números introducidos manualmente.
    with (DESTINO / "singularidad_muneca_datos.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f, lineterminator="\n")
        writer.writerow(["q5_rad", "rango_J", "sigma_min_J_normalizado", "condicion_J_normalizado",
                         "norma_qdot_rad_s", "error_consigna"])
        for vals in zip(q5, e.rangos, e.sigma_min, e.condiciones, e.normas_qdot, e.errores_consigna):
            writer.writerow([f"{float(v):.12g}" if np.isfinite(v) else "NaN" if np.isnan(v) else "Inf"
                             for v in vals])

    # Figura 1: rango y clasificación de la singularidad en el último punto.
    fig, ax = plt.subplots(figsize=(9, 4.6))
    ax.plot(xs, e.rangos, marker="o", linewidth=1.8)
    ax.set_ylim(4.7, 6.3)
    ax.set_yticks([5, 6])
    _acabado(ax, q5, "Pérdida de rango del Jacobiano geométrico", "Rango de J")
    ax.annotate("Singular: rango 5", (xs[-1], e.rangos[-1]),
                xytext=(-112, -26), textcoords="offset points",
                arrowprops={"arrowstyle": "->"}, fontsize=9)
    fig.tight_layout(rect=[0, 0.035, 1, 1])
    _guardar(fig, "01_rango_jacobiano.png")

    # Figura 2: sigma_min se grafica sólo para las posturas regulares, y se
    # señala el caso singular por separado para no representar como positivo
    # un cero numérico que depende del redondeo.
    fig, ax = plt.subplots(figsize=(9, 4.6))
    ax.semilogy(xs[:-1], e.sigma_min[:-1], marker="o", linewidth=1.8)
    ax.axvline(xs[-1], linestyle="--", linewidth=1.2)
    _acabado(ax, q5, "Pérdida progresiva de independencia (SVD)", "Menor valor singular de J normalizado")
    ax.annotate("q5 = 0: σmín ≈ 0 (rango 5)", xy=(xs[-1], e.sigma_min[-2]),
                xytext=(-165, 32), textcoords="offset points",
                arrowprops={"arrowstyle": "->"}, fontsize=9)
    fig.tight_layout(rect=[0, 0.035, 1, 1])
    _guardar(fig, "02_valor_singular_minimo.png")

    # Figura 3: sin punto de norma qdot en q5 = 0, ya que la consigna no
    # pertenece a la imagen del Jacobiano y no hay solución exacta.
    fig, ax = plt.subplots(figsize=(9, 4.6))
    ax.semilogy(xs[:-1], e.normas_qdot[:-1], marker="o", linewidth=1.8)
    ax.axvline(xs[-1], linestyle="--", linewidth=1.2)
    _acabado(ax, q5, "Amplificación de velocidad articular requerida", "Norma de q̇ [rad/s]")
    ax.annotate("Sin solución exacta en q5 = 0", xy=(xs[-1], e.normas_qdot[-2]),
                xytext=(-183, -40), textcoords="offset points",
                arrowprops={"arrowstyle": "->"}, fontsize=9)
    fig.tight_layout(rect=[0, 0.035, 1, 1])
    _guardar(fig, "03_amplificacion_velocidades.png")

    print("Evidencia reproducible generada:")
    for f in sorted(DESTINO.iterdir()):
        print(f"  {f.relative_to(RAIZ)} ({f.stat().st_size:,} bytes)")
    print("Advertencia: las velocidades calculadas NO son comandos admisibles para el robot.")


if __name__ == "__main__":
    generar()
