# Hoja de Definición de Juntas — ABB IRB 120

Esta hoja documenta las seis juntas del ABB IRB 120 utilizadas en el modelo URDF del Hito 1 y los frames fijos añadidos en la cadena terminal.

Los valores `xyz` se expresan en metros, `rpy` en radianes y `axis` corresponde al eje de rotación expresado en el marco local de la junta.

## Juntas móviles J1–J6

| Junta | Parent | Child | xyz [m] | rpy [rad] | axis | Límites [rad] | Límites [°] |
|---|---|---|---|---|---|---|---|
| J1 | `base_link` | `link_1` | `0 0 0` | `0 0 0` | `0 0 1` | `[-2.879793, 2.879793]` | `[-165, 165]` |
| J2 | `link_1` | `link_2` | `0 0 0.290` | `0 0 0` | `0 1 0` | `[-1.919862, 1.919862]` | `[-110, 110]` |
| J3 | `link_2` | `link_3` | `0 0 0.270` | `0 0 0` | `0 1 0` | `[-1.919862, 1.221730]` | `[-110, 70]` |
| J4 | `link_3` | `link_4` | `0 0 0.070` | `0 0 0` | `1 0 0` | `[-2.792527, 2.792527]` | `[-160, 160]` |
| J5 | `link_4` | `link_5` | `0.302 0 0` | `0 0 0` | `0 1 0` | `[-2.094395, 2.094395]` | `[-120, 120]` |
| J6 | `link_5` | `link_6` | `0.072 0 0` | `0 0 0` | `1 0 0` | `[-6.981317, 6.981317]` | `[-400, 400]` |

Las seis articulaciones son de tipo `revolute` y constituyen los seis grados de libertad del manipulador.

`parent` y `child` definen la cadena cinemática:

`base_link -> link_1 -> link_2 -> link_3 -> link_4 -> link_5 -> link_6`

El atributo `origin` define la pose fija del marco de la junta respecto al eslabón `parent`.

El atributo `axis` define el eje local alrededor del cual gira la articulación.

Los parámetros D-H no se copian directamente a `origin`, ya que D-H y URDF describen la misma cadena física mediante convenciones diferentes.

## Cadena terminal fija

Después de `link_6`, el modelo incorpora tres joints de tipo `fixed`:

| Joint fijo | Parent | Child | xyz [m] | rpy [rad] | Función |
|---|---|---|---|---|---|
| `link6_to_dh_frame_6` | `link_6` | `dh_frame_6` | `0 0 0` | `0 -1.5707963267948966 3.141592653589793` | Alinear el frame terminal ROS con el frame D-H `{6}` |
| `dh6_to_gripper` | `dh_frame_6` | `gripper_base` | `0 0 0` | `0 0 0` | Acoplar rígidamente la pinza al frame `{6}` |
| `gripper_to_tcp` | `gripper_base` | `tcp` | `0 0 0.120` | `0 0 0` | Definir el TCP a 120 mm sobre `Z_6` |

Estos joints son fijos y no añaden grados de libertad al manipulador.

### Frame `dh_frame_6`

`link_6` y el frame D-H `{6}` comparten el mismo origen físico, pero utilizan orientaciones distintas.

Para disponer en ROS del mismo frame terminal utilizado por la cinemática D-H se añadió `dh_frame_6`, relacionado con `link_6` mediante una transformación fija.

La relación entre sus ejes es:

- `x_DH = z_link6`
- `y_DH = -y_link6`
- `z_DH = x_link6`

Esta transformación permite que la pose de `dh_frame_6` pueda compararse directamente con la transformación homogénea `0T6` obtenida mediante la cinemática directa D-H.

### Pinza y TCP

`gripper_base` representa el acoplamiento rígido del efector final.

La geometría actual de la pinza es provisional y puede sustituirse posteriormente por una malla STL sin modificar la definición cinemática del TCP.

El TCP se encuentra a 0.120 m sobre `Z_6`, sin desfase de rotación respecto al frame `{6}`:

```math
{}^{6}T_{TCP} =
\begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 \\
0 & 0 & 1 & 0.120 \\
0 & 0 & 0 & 1
\end{bmatrix}
```

Por tanto, la pose del TCP respecto a la base se obtiene mediante:

```math
{}^{0}T_{TCP}(q) = {}^{0}T_{6}(q)\,{}^{6}T_{TCP}
```

La cadena terminal completa del modelo queda:

`link_6 -> dh_frame_6 -> gripper_base -> tcp`
