# Gate 0 — Marcos y TCP del modelo propio

## Fuente verificada

Archivo:
hito1/irb120_description/urdf/irb120.urdf

Último commit que modificó el URDF:
ae2cf35 (2026-09-09)

## Cadena cinemática

base_link -> link_1 -> link_2 -> link_3 ->
link_4 -> link_5 -> link_6 -> dh_frame_6 ->
gripper_base -> tcp

Las primeras seis juntas son revolutas.
Las tres últimas juntas son fijas.

## Transformaciones terminales

| Origen | Destino | xyz (m) | rpy (rad) |
|---|---|---|---|
| link_6 | dh_frame_6 | 0, 0, 0 | 0, -pi/2, pi |
| dh_frame_6 | gripper_base | 0, 0, 0 | 0, 0, 0 |
| gripper_base | tcp | 0, 0, 0.120 | 0, 0, 0 |

## Interpretación geométrica

dh_frame_6 y link_6 comparten origen, pero no orientación.

La traslación del TCP en dh_frame_6 es:

p = [0, 0, 0.120] m

Expresada en el marco link_6:

p = [0.120, 0, 0] m

La orientación del TCP coincide con dh_frame_6
en el modelo actual.

El desplazamiento visual de 0.030 m de la pinza
no modifica el origen de gripper_base.

## Estado de verificación

CONFIRMADO:
- Marcos y juntas declarados en el URDF.
- Transformaciones fijas del modelo.
- TCP provisional de 120 mm.
- Commit fuente del URDF.

PENDIENTE:
- Confirmar herramienta física instalada.
- Confirmar TCP configurado en el controlador ABB.
- Validar correspondencia entre modelo propio y robot físico.
- Comprobar comparaciones FK con marcos equivalentes.

Este documento no certifica compatibilidad física
ni constituye autorización de movimiento.
