#!/usr/bin/env python3

from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import matplotlib.pyplot as plt
import numpy as np


# ============================================================
# Rutas
# ============================================================

HERE = Path(__file__).resolve().parent
KIN_DIR = HERE.parent / "cinematica_directa"
HITO1_DIR = HERE.parent.parent
URDF_PATH = HITO1_DIR / "irb120_description" / "urdf" / "irb120.urdf"

sys.path.insert(0, str(KIN_DIR))

from dh import dh_matrix
from irb120_fk import forward_kinematics, irb120_dh_params


# ============================================================
# Configuración articular de referencia
# ============================================================

Q0 = np.zeros(6)

EXPECTED_T06_Q0 = np.array([
    [0.0,  0.0, 1.0, 0.374],
    [0.0, -1.0, 0.0, 0.000],
    [1.0,  0.0, 0.0, 0.630],
    [0.0,  0.0, 0.0, 1.000],
])

EXPECTED_DH_ORIGINS_Q0 = np.array([
    [0.000, 0.000, 0.000],
    [0.000, 0.000, 0.290],
    [0.000, 0.000, 0.560],
    [0.000, 0.000, 0.630],
    [0.302, 0.000, 0.630],
    [0.302, 0.000, 0.630],
    [0.374, 0.000, 0.630],
])

EXPECTED_URDF_ORIGINS_Q0 = {
    "base_link": np.array([0.000, 0.000, 0.000]),
    "link_1":    np.array([0.000, 0.000, 0.000]),
    "link_2":    np.array([0.000, 0.000, 0.290]),
    "link_3":    np.array([0.000, 0.000, 0.560]),
    "link_4":    np.array([0.000, 0.000, 0.630]),
    "link_5":    np.array([0.302, 0.000, 0.630]),
    "link_6":    np.array([0.374, 0.000, 0.630]),
}


# ============================================================
# Estilo común
# ============================================================

COLOR_X = "#D62828"
COLOR_Y = "#2A9D3F"
COLOR_Z = "#2463C7"
COLOR_CHAIN = "#343A40"
COLOR_TEXT = "#20252B"
COLOR_SECONDARY = "#626A73"

AXIS_LENGTH = 0.047


# ============================================================
# D-H
# ============================================================

def calcular_frames_dh(q):
    """Calcula [0T0, 0T1, ..., 0T6] usando el modelo auditado."""

    T = np.eye(4)
    frames = [T.copy()]

    for params in irb120_dh_params(q):
        T = T @ dh_matrix(*params)
        frames.append(T.copy())

    return frames


def validar_dh(frames):
    """Valida la cadena antes de generar evidencia gráfica."""

    for i, T in enumerate(frames):
        R = T[:3, :3]

        if not np.allclose(R.T @ R, np.eye(3), atol=1e-12):
            raise RuntimeError(f"D-H frame {{{i}}}: R^T R != I")

        if not np.isclose(np.linalg.det(R), 1.0, atol=1e-12):
            raise RuntimeError(f"D-H frame {{{i}}}: det(R) != 1")

    if not np.allclose(
        frames[-1],
        forward_kinematics(Q0),
        atol=1e-12,
    ):
        raise RuntimeError(
            "La cadena D-H dibujada no coincide con forward_kinematics()."
        )

    if not np.allclose(
        frames[-1],
        EXPECTED_T06_Q0,
        atol=1e-12,
    ):
        raise RuntimeError(
            "0T6(q=0) no coincide con la pose de referencia."
        )

    origins = np.array([T[:3, 3] for T in frames])

    if not np.allclose(
        origins,
        EXPECTED_DH_ORIGINS_Q0,
        atol=1e-12,
    ):
        raise RuntimeError(
            "Los orígenes D-H no coinciden con la referencia."
        )

    print("VALIDACION D-H: PASS")


# ============================================================
# URDF
# ============================================================

def vector_xml(text, default):
    if text is None:
        return np.array(default, dtype=float)

    return np.array(
        [float(v) for v in text.split()],
        dtype=float,
    )


def matriz_rpy(rpy):
    """
    Convención URDF:
        R = Rz(yaw) Ry(pitch) Rx(roll)
    """

    roll, pitch, yaw = rpy

    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    Rx = np.array([
        [1.0, 0.0, 0.0],
        [0.0, cr, -sr],
        [0.0, sr,  cr],
    ])

    Ry = np.array([
        [ cp, 0.0, sp],
        [0.0, 1.0, 0.0],
        [-sp, 0.0, cp],
    ])

    Rz = np.array([
        [cy, -sy, 0.0],
        [sy,  cy, 0.0],
        [0.0, 0.0, 1.0],
    ])

    return Rz @ Ry @ Rx


def matriz_origen(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = matriz_rpy(rpy)
    T[:3, 3] = xyz
    return T


def matriz_eje_angulo(axis, theta):
    """Rotación de Rodrigues alrededor del axis local de una junta."""

    axis = np.asarray(axis, dtype=float)
    norma = np.linalg.norm(axis)

    if norma == 0:
        raise ValueError("El axis URDF no puede tener norma cero.")

    x, y, z = axis / norma

    c = np.cos(theta)
    s = np.sin(theta)
    C = 1.0 - c

    R = np.array([
        [x*x*C + c,   x*y*C - z*s, x*z*C + y*s],
        [y*x*C + z*s, y*y*C + c,   y*z*C - x*s],
        [z*x*C - y*s, z*y*C + x*s, z*z*C + c],
    ])

    T = np.eye(4)
    T[:3, :3] = R

    return T


def calcular_frames_urdf(q):
    """
    Calcula las poses globales de los links directamente desde
    irb120.urdf.

    No utiliza la FK D-H.
    """

    q = np.asarray(q, dtype=float)

    if q.shape != (6,):
        raise ValueError("q debe contener 6 valores.")

    q_joint = {
        f"joint_{i + 1}": q[i]
        for i in range(6)
    }

    root = ET.parse(URDF_PATH).getroot()

    joints = []

    for joint in root.findall("joint"):
        name = joint.attrib["name"]
        joint_type = joint.attrib["type"]

        parent = joint.find("parent").attrib["link"]
        child = joint.find("child").attrib["link"]

        origin = joint.find("origin")

        if origin is None:
            xyz = np.zeros(3)
            rpy = np.zeros(3)
        else:
            xyz = vector_xml(
                origin.attrib.get("xyz"),
                [0.0, 0.0, 0.0],
            )
            rpy = vector_xml(
                origin.attrib.get("rpy"),
                [0.0, 0.0, 0.0],
            )

        axis_tag = joint.find("axis")

        if axis_tag is None:
            axis = np.array([1.0, 0.0, 0.0])
        else:
            axis = vector_xml(
                axis_tag.attrib.get("xyz"),
                [1.0, 0.0, 0.0],
            )

        joints.append({
            "name": name,
            "type": joint_type,
            "parent": parent,
            "child": child,
            "xyz": xyz,
            "rpy": rpy,
            "axis": axis,
        })

    frames = {
        "base_link": np.eye(4)
    }

    pendientes = joints.copy()

    while pendientes:
        progreso = False

        for joint in pendientes[:]:
            if joint["parent"] not in frames:
                continue

            T_parent = frames[joint["parent"]]

            T_origin = matriz_origen(
                joint["xyz"],
                joint["rpy"],
            )

            T_mov = np.eye(4)

            if joint["type"] in ("revolute", "continuous"):
                theta = q_joint.get(joint["name"], 0.0)

                T_mov = matriz_eje_angulo(
                    joint["axis"],
                    theta,
                )

            elif joint["type"] == "prismatic":
                raise NotImplementedError(
                    "Este modelo no contiene juntas prismáticas."
                )

            frames[joint["child"]] = (
                T_parent
                @ T_origin
                @ T_mov
            )

            pendientes.remove(joint)
            progreso = True

        if not progreso:
            nombres = [j["name"] for j in pendientes]

            raise RuntimeError(
                "No se pudo resolver completamente el árbol URDF: "
                + ", ".join(nombres)
            )

    return frames


def validar_urdf(frames_urdf, frames_dh):
    """
    Verifica la configuración q=0 directamente desde el URDF.
    """

    for nombre, esperado in EXPECTED_URDF_ORIGINS_Q0.items():
        obtenido = frames_urdf[nombre][:3, 3]

        if not np.allclose(obtenido, esperado, atol=1e-12):
            raise RuntimeError(
                f"{nombre}: posición inesperada en q=0."
            )

    # En el URDF actual, todos los origin rpy de joint_1...joint_6
    # son cero. Por tanto, en q=0 los frames link_1...link_6
    # conservan la orientación de base_link.
    for i in range(1, 7):
        R = frames_urdf[f"link_{i}"][:3, :3]

        if not np.allclose(R, np.eye(3), atol=1e-12):
            raise RuntimeError(
                f"link_{i}: orientación inesperada en q=0."
            )

    # dh_frame_6 debe reproducir exactamente el frame {6} D-H.
    if not np.allclose(
        frames_urdf["dh_frame_6"],
        frames_dh[-1],
        atol=1e-12,
    ):
        raise RuntimeError(
            "dh_frame_6 no coincide con el frame {6} D-H."
        )

    print("VALIDACION URDF q=0: PASS")
    print("VALIDACION dh_frame_6 <-> {6} D-H: PASS")


# ============================================================
# Componentes gráficos
# ============================================================

def dibujar_triedro(ax, T, longitud=AXIS_LENGTH):
    origen = T[:3, 3]
    R = T[:3, :3]

    for direccion, color in zip(
        (R[:, 0], R[:, 1], R[:, 2]),
        (COLOR_X, COLOR_Y, COLOR_Z),
    ):
        v = longitud * direccion

        ax.quiver(
            origen[0],
            origen[1],
            origen[2],
            v[0],
            v[1],
            v[2],
            color=color,
            linewidth=2.0,
            arrow_length_ratio=0.22,
            normalize=False,
        )


def etiqueta(ax, posicion, texto, fontsize=10):
    ax.text(
        posicion[0],
        posicion[1],
        posicion[2],
        texto,
        ha="center",
        va="center",
        fontsize=fontsize,
        color=COLOR_TEXT,
        bbox=dict(
            boxstyle="round,pad=0.25",
            facecolor="white",
            edgecolor="#C7CCD1",
            linewidth=0.7,
            alpha=0.96,
        ),
        zorder=20,
    )


def configurar_vista(ax):
    """
    Misma cámara y escala para todas las figuras.
    """

    ax.set_proj_type("ortho")

    ax.view_init(
        elev=21,
        azim=-63,
    )

    ax.set_xlim(-0.09, 0.47)
    ax.set_ylim(-0.18, 0.18)
    ax.set_zlim(-0.045, 0.71)

    ax.set_box_aspect((
        0.56,
        0.36,
        0.755,
    ))

    ax.set_axis_off()


def encabezado(fig, titulo):
    fig.text(
        0.075,
        0.925,
        titulo,
        fontsize=17,
        weight="bold",
        color=COLOR_TEXT,
        ha="left",
    )

    fig.text(
        0.075,
        0.882,
        r"Configuración articular de referencia: "
        r"$q=[0,0,0,0,0,0]^T$ rad",
        fontsize=11,
        color=COLOR_SECONDARY,
        ha="left",
    )

    fig.text(
        0.785,
        0.925,
        "x",
        fontsize=11,
        weight="bold",
        color=COLOR_X,
    )

    fig.text(
        0.815,
        0.925,
        "y",
        fontsize=11,
        weight="bold",
        color=COLOR_Y,
    )

    fig.text(
        0.845,
        0.925,
        "z",
        fontsize=11,
        weight="bold",
        color=COLOR_Z,
    )

    fig.text(
        0.875,
        0.925,
        "ejes locales",
        fontsize=9,
        color=COLOR_SECONDARY,
    )


def exportar(fig, svg, png):
    fig.subplots_adjust(
        left=0.025,
        right=0.98,
        top=0.86,
        bottom=0.05,
    )

    fig.savefig(
        svg,
        format="svg",
        bbox_inches="tight",
        pad_inches=0.10,
    )

    fig.savefig(
        png,
        dpi=220,
        bbox_inches="tight",
        pad_inches=0.10,
        facecolor="white",
    )

    plt.close(fig)


# ============================================================
# Figura D-H
# ============================================================

def graficar_dh(frames, salida_svg, salida_png):
    fig = plt.figure(
        figsize=(12.8, 7.2),
        facecolor="white",
    )

    ax = fig.add_subplot(
        111,
        projection="3d",
    )

    origins = np.array([
        T[:3, 3]
        for T in frames
    ])

    ax.plot(
        origins[:, 0],
        origins[:, 1],
        origins[:, 2],
        color=COLOR_CHAIN,
        linewidth=3.0,
        solid_capstyle="round",
        zorder=2,
    )

    ax.scatter(
        origins[:, 0],
        origins[:, 1],
        origins[:, 2],
        s=24,
        color=COLOR_CHAIN,
        depthshade=False,
        zorder=5,
    )

    for T in frames:
        dibujar_triedro(ax, T)

    offsets = {
        0: np.array([-0.030, -0.025, -0.035]),
        1: np.array([-0.045, -0.020,  0.012]),
        2: np.array([-0.050, -0.020, -0.010]),
        3: np.array([-0.040, -0.022,  0.040]),
        4: np.array([-0.025, -0.045,  0.040]),
        5: np.array([ 0.020, -0.050, -0.040]),
        6: np.array([ 0.035, -0.020,  0.035]),
    }

    for i, T in enumerate(frames):
        etiqueta(
            ax,
            T[:3, 3] + offsets[i],
            rf"$\{{{i}\}}$",
        )

    configurar_vista(ax)
    encabezado(fig, "Asignación de marcos D-H")

    fig.text(
        0.74,
        0.115,
        r"$O_4 = O_5$",
        fontsize=10,
        weight="bold",
        color=COLOR_TEXT,
        ha="left",
    )

    fig.text(
        0.74,
        0.082,
        "Los frames {4} y {5} comparten origen.",
        fontsize=9,
        color=COLOR_SECONDARY,
        ha="left",
    )

    exportar(
        fig,
        salida_svg,
        salida_png,
    )


# ============================================================
# Figura URDF
# ============================================================

def graficar_urdf(frames, salida_svg, salida_png):
    """
    Muestra únicamente los frames estructurales base_link,
    link_1 ... link_6.

    dh_frame_6 se reserva para la explicación posterior de C.
    """

    nombres = [
        "base_link",
        "link_1",
        "link_2",
        "link_3",
        "link_4",
        "link_5",
        "link_6",
    ]

    frames_lista = [
        frames[nombre]
        for nombre in nombres
    ]

    origins = np.array([
        T[:3, 3]
        for T in frames_lista
    ])

    fig = plt.figure(
        figsize=(12.8, 7.2),
        facecolor="white",
    )

    ax = fig.add_subplot(
        111,
        projection="3d",
    )

    ax.plot(
        origins[:, 0],
        origins[:, 1],
        origins[:, 2],
        color=COLOR_CHAIN,
        linewidth=3.0,
        solid_capstyle="round",
        zorder=2,
    )

    ax.scatter(
        origins[:, 0],
        origins[:, 1],
        origins[:, 2],
        s=24,
        color=COLOR_CHAIN,
        depthshade=False,
        zorder=5,
    )

    for T in frames_lista:
        dibujar_triedro(ax, T)

    labels = [
        "base",
        r"$L_1$",
        r"$L_2$",
        r"$L_3$",
        r"$L_4$",
        r"$L_5$",
        r"$L_6$",
    ]

    offsets = {
        0: np.array([-0.045, -0.025, -0.040]),
        1: np.array([ 0.035, -0.025, -0.010]),
        2: np.array([-0.045, -0.020,  0.018]),
        3: np.array([-0.050, -0.020,  0.015]),
        4: np.array([-0.040, -0.020,  0.038]),
        5: np.array([-0.020, -0.040,  0.038]),
        6: np.array([ 0.035, -0.020,  0.035]),
    }

    for i, (T, texto) in enumerate(
        zip(frames_lista, labels)
    ):
        etiqueta(
            ax,
            T[:3, 3] + offsets[i],
            texto,
        )

    configurar_vista(ax)
    encabezado(fig, "Frames estructurales del URDF")

    fig.text(
        0.71,
        0.115,
        r"$L_i \equiv$ `link_i`",
        fontsize=10,
        weight="bold",
        color=COLOR_TEXT,
        ha="left",
    )

    fig.text(
        0.71,
        0.082,
        "`base_link` y `link_1` comparten origen en q = 0.",
        fontsize=9,
        color=COLOR_SECONDARY,
        ha="left",
    )

    exportar(
        fig,
        salida_svg,
        salida_png,
    )


# ============================================================
# Comparación D-H ↔ URDF
# ============================================================

def graficar_comparacion(
    frames_dh,
    frames_urdf,
    salida_svg,
    salida_png,
):
    fig = plt.figure(
        figsize=(14.4, 7.2),
        facecolor="white",
    )

    ax_dh = fig.add_subplot(
        121,
        projection="3d",
    )

    ax_urdf = fig.add_subplot(
        122,
        projection="3d",
    )

    # -------------------- D-H --------------------

    origins_dh = np.array([
        T[:3, 3]
        for T in frames_dh
    ])

    ax_dh.plot(
        origins_dh[:, 0],
        origins_dh[:, 1],
        origins_dh[:, 2],
        color=COLOR_CHAIN,
        linewidth=2.7,
    )

    ax_dh.scatter(
        origins_dh[:, 0],
        origins_dh[:, 1],
        origins_dh[:, 2],
        s=20,
        color=COLOR_CHAIN,
        depthshade=False,
    )

    for T in frames_dh:
        dibujar_triedro(
            ax_dh,
            T,
            longitud=0.043,
        )

    ax_dh.set_title(
        "Denavit–Hartenberg",
        fontsize=14,
        weight="bold",
        color=COLOR_TEXT,
        pad=8,
    )

    configurar_vista(ax_dh)

    # -------------------- URDF --------------------

    nombres_urdf = [
        "base_link",
        "link_1",
        "link_2",
        "link_3",
        "link_4",
        "link_5",
        "link_6",
    ]

    lista_urdf = [
        frames_urdf[n]
        for n in nombres_urdf
    ]

    origins_urdf = np.array([
        T[:3, 3]
        for T in lista_urdf
    ])

    ax_urdf.plot(
        origins_urdf[:, 0],
        origins_urdf[:, 1],
        origins_urdf[:, 2],
        color=COLOR_CHAIN,
        linewidth=2.7,
    )

    ax_urdf.scatter(
        origins_urdf[:, 0],
        origins_urdf[:, 1],
        origins_urdf[:, 2],
        s=20,
        color=COLOR_CHAIN,
        depthshade=False,
    )

    for T in lista_urdf:
        dibujar_triedro(
            ax_urdf,
            T,
            longitud=0.043,
        )

    ax_urdf.set_title(
        "URDF — frames de links",
        fontsize=14,
        weight="bold",
        color=COLOR_TEXT,
        pad=8,
    )

    configurar_vista(ax_urdf)

    # -------------------- Encabezado --------------------

    fig.text(
        0.055,
        0.935,
        "Misma configuración física, distintas convenciones de frames",
        fontsize=17,
        weight="bold",
        color=COLOR_TEXT,
        ha="left",
    )

    fig.text(
        0.055,
        0.895,
        r"Configuración articular de referencia: "
        r"$q=[0,0,0,0,0,0]^T$ rad",
        fontsize=11,
        color=COLOR_SECONDARY,
        ha="left",
    )

    fig.text(
        0.735,
        0.935,
        "x",
        fontsize=11,
        weight="bold",
        color=COLOR_X,
    )

    fig.text(
        0.765,
        0.935,
        "y",
        fontsize=11,
        weight="bold",
        color=COLOR_Y,
    )

    fig.text(
        0.795,
        0.935,
        "z",
        fontsize=11,
        weight="bold",
        color=COLOR_Z,
    )

    fig.text(
        0.825,
        0.935,
        "ejes locales",
        fontsize=9,
        color=COLOR_SECONDARY,
    )

    fig.text(
        0.50,
        0.055,
        "D-H orienta sus marcos según la convención geométrica; "
        "URDF expresa por separado la pose del frame y el eje de cada junta.",
        fontsize=10,
        color=COLOR_SECONDARY,
        ha="center",
    )

    fig.subplots_adjust(
        left=0.02,
        right=0.98,
        top=0.84,
        bottom=0.10,
        wspace=0.02,
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


# ============================================================
# Main
# ============================================================

def main():
    frames_dh = calcular_frames_dh(Q0)
    validar_dh(frames_dh)

    frames_urdf = calcular_frames_urdf(Q0)
    validar_urdf(frames_urdf, frames_dh)

    graficar_dh(
        frames_dh,
        HERE / "marcos_dh_q0.svg",
        HERE / "marcos_dh_q0.png",
    )

    graficar_urdf(
        frames_urdf,
        HERE / "marcos_urdf_q0.svg",
        HERE / "marcos_urdf_q0.png",
    )

    graficar_comparacion(
        frames_dh,
        frames_urdf,
        HERE / "comparacion_dh_urdf_q0.svg",
        HERE / "comparacion_dh_urdf_q0.png",
    )

    print()
    print("FIGURAS GENERADAS:")
    print("  marcos_dh_q0.svg / .png")
    print("  marcos_urdf_q0.svg / .png")
    print("  comparacion_dh_urdf_q0.svg / .png")


if __name__ == "__main__":
    main()
