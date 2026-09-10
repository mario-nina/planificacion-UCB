#!/usr/bin/env python3

from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np


HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from generar_marcos import (
    calcular_frames_dh,
    calcular_frames_urdf,
)


# ============================================================
# Configuraciones ya utilizadas en la validación del Hito 1
# ============================================================

QA = np.radians([
    0, 0, 0, 0, 0, 0
])

QB = np.radians([
    20, -30, 25, 15, -20, 35
])


# ============================================================
# Cálculo de la transformación relativa
# ============================================================

def calcular_c(q):
    """
    Deduce C sin introducirla manualmente:

        0T_DH6 = 0T_link6 * C

    por tanto:

        C = inv(0T_link6) * 0T_DH6
    """

    frames_dh = calcular_frames_dh(q)
    frames_urdf = calcular_frames_urdf(q)

    T_0_dh6 = frames_dh[-1]
    T_0_link6 = frames_urdf["link_6"]

    C = np.linalg.inv(T_0_link6) @ T_0_dh6

    return C, T_0_link6, T_0_dh6, frames_urdf


def validar_constancia():
    C_A, T_L_A, T_D_A, frames_A = calcular_c(QA)
    C_B, T_L_B, T_D_B, frames_B = calcular_c(QB)

    print("========================================")
    print("TRANSFORMACION RELATIVA EN q(A)")
    print("========================================")
    print(np.round(C_A, 12))

    print()
    print("========================================")
    print("TRANSFORMACION RELATIVA EN q(B)")
    print("========================================")
    print(np.round(C_B, 12))

    diferencia = np.max(np.abs(C_A - C_B))

    print()
    print("max|C_A - C_B| =", diferencia)

    if not np.allclose(C_A, C_B, atol=1e-12):
        raise RuntimeError(
            "C no permanece constante entre A y B."
        )

    # Verificación adicional:
    # el dh_frame_6 del URDF debe implementar exactamente C.
    C_urdf_A = (
        np.linalg.inv(T_L_A)
        @ frames_A["dh_frame_6"]
    )

    C_urdf_B = (
        np.linalg.inv(T_L_B)
        @ frames_B["dh_frame_6"]
    )

    if not np.allclose(C_A, C_urdf_A, atol=1e-12):
        raise RuntimeError(
            "El joint fijo link_6 -> dh_frame_6 "
            "no implementa la C deducida en A."
        )

    if not np.allclose(C_B, C_urdf_B, atol=1e-12):
        raise RuntimeError(
            "El joint fijo link_6 -> dh_frame_6 "
            "no implementa la C deducida en B."
        )

    R = C_A[:3, :3]
    p = C_A[:3, 3]

    if not np.allclose(R.T @ R, np.eye(3), atol=1e-12):
        raise RuntimeError("R_C no es ortogonal.")

    if not np.isclose(np.linalg.det(R), 1.0, atol=1e-12):
        raise RuntimeError("det(R_C) != +1.")

    print()
    print("CONSTANCIA DE C: PASS")
    print("IMPLEMENTACION URDF DE C: PASS")
    print("R_C^T R_C = I: PASS")
    print("det(R_C) = +1: PASS")

    print()
    print("Traslacion relativa:")
    print(np.round(p, 12))

    return C_A


# ============================================================
# Figura geométrica
# ============================================================

COLOR_X = "#D62828"
COLOR_Y = "#2A9D3F"
COLOR_Z = "#2463C7"
COLOR_TEXT = "#20252B"
COLOR_SECONDARY = "#626A73"


def dibujar_frame(ax, R, labels):
    """
    Dibuja un triedro centrado en el origen.

    La separación de los dos gráficos es puramente visual:
    físicamente link_6 y {6}_DH comparten origen.
    """

    origen = np.zeros(3)
    longitud = 0.78

    colores = (
        COLOR_X,
        COLOR_Y,
        COLOR_Z,
    )

    for j, (label, color) in enumerate(
        zip(labels, colores)
    ):
        direccion = R[:, j]
        v = longitud * direccion

        ax.quiver(
            0, 0, 0,
            v[0], v[1], v[2],
            color=color,
            linewidth=3.0,
            arrow_length_ratio=0.13,
        )

        punta = 1.10 * v

        ax.text(
            punta[0],
            punta[1],
            punta[2],
            label,
            fontsize=13,
            weight="bold",
            color=color,
            ha="center",
            va="center",
        )

    ax.scatter(
        [0], [0], [0],
        s=35,
        color=COLOR_TEXT,
    )

    ax.set_xlim(-1.0, 1.0)
    ax.set_ylim(-1.0, 1.0)
    ax.set_zlim(-1.0, 1.0)

    ax.set_box_aspect((1, 1, 1))
    ax.set_proj_type("ortho")

    ax.view_init(
        elev=22,
        azim=-58,
    )

    ax.set_axis_off()


def generar_figura(C, salida_svg, salida_png):
    """
    Compara la orientación local de link_6 con el frame D-H {6}.
    """

    R_L = np.eye(3)
    R_DH = C[:3, :3]

    fig = plt.figure(
        figsize=(13.2, 7.2),
        facecolor="white",
    )

    ax_l = fig.add_subplot(
        121,
        projection="3d",
    )

    ax_dh = fig.add_subplot(
        122,
        projection="3d",
    )

    dibujar_frame(
        ax_l,
        R_L,
        (
            r"$x_L$",
            r"$y_L$",
            r"$z_L$",
        ),
    )

    dibujar_frame(
        ax_dh,
        R_DH,
        (
            r"$x_6$",
            r"$y_6$",
            r"$z_6$",
        ),
    )

    ax_l.set_title(
        r"`link_6` — frame URDF",
        fontsize=15,
        weight="bold",
        color=COLOR_TEXT,
        pad=14,
    )

    ax_dh.set_title(
        r"Frame $\{6\}$ — D-H",
        fontsize=15,
        weight="bold",
        color=COLOR_TEXT,
        pad=14,
    )

    fig.text(
        0.055,
        0.94,
        "Relación geométrica entre los frames terminales",
        fontsize=18,
        weight="bold",
        color=COLOR_TEXT,
        ha="left",
    )

    fig.text(
        0.055,
        0.895,
        r"Misma brida y mismo origen físico · configuración de referencia $q=0$",
        fontsize=11,
        color=COLOR_SECONDARY,
        ha="left",
    )

    fig.text(
        0.5,
        0.155,
        r"$x_6=z_L \qquad y_6=-y_L \qquad z_6=x_L$",
        fontsize=16,
        color=COLOR_TEXT,
        ha="center",
    )

    fig.text(
        0.5,
        0.095,
        r"$^{0}T_{6}^{DH}=\,^{0}T_{link_6}^{URDF}\,C$",
        fontsize=14,
        color=COLOR_TEXT,
        ha="center",
    )

    fig.text(
        0.5,
        0.045,
        "Los dos triedros se muestran separados únicamente para facilitar la lectura.",
        fontsize=9,
        color=COLOR_SECONDARY,
        ha="center",
    )

    fig.subplots_adjust(
        left=0.04,
        right=0.96,
        top=0.82,
        bottom=0.20,
        wspace=0.06,
    )

    fig.savefig(
        salida_svg,
        format="svg",
        bbox_inches="tight",
        pad_inches=0.10,
    )

    fig.savefig(
        salida_png,
        dpi=220,
        bbox_inches="tight",
        pad_inches=0.10,
        facecolor="white",
    )

    plt.close(fig)

    print()
    print("Figura SVG:", salida_svg)
    print("Figura PNG:", salida_png)


def main():
    C = validar_constancia()

    generar_figura(
        C,
        HERE / "relacion_link6_dh6.svg",
        HERE / "relacion_link6_dh6.png",
    )


if __name__ == "__main__":
    main()
