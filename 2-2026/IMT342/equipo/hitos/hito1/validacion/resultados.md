# Validación cruzada FK ↔ URDF — ABB IRB 120

Esta validación compara la cinemática directa calculada a partir del modelo D-H con las transformaciones publicadas por ROS a partir del modelo URDF.

Para cada configuración articular se siguen dos rutas independientes:

- FK D-H: `q -> forward_kinematics(q) -> T_FK`
- URDF/ROS: `q -> /joint_states -> robot_state_publisher -> TF -> T_ROS`

El criterio posicional de aceptación es:

```math
\Delta E = \left\|p_{FK} - p_{ROS}\right\| < 10^{-6}\ \mathrm{m}
```

La orientación se verifica comparando directamente las matrices de rotación mediante el máximo error elemento a elemento:

```math
e_R = \max\left|R_{FK} - R_{ROS}\right|
```

## Configuraciones de prueba

| Configuración | q [°] |
|---|---|
| A | `[0, 0, 0, 0, 0, 0]` |
| B | `[20, -30, 25, 15, -20, 35]` |

## Validación de la brida `{6}`

La comparación principal utiliza:

- FK matemática: `forward_kinematics(q)` → `0T6`
- ROS/URDF: TF `base_link -> dh_frame_6`

| Conf. | Posición FK [m] | Posición ROS [m] | Delta E [m] | Error máx. orientación | Resultado |
|---|---|---|---:|---:|---|
| A | `[0.374000000, 0.000000000, 0.630000000]` | `[0.374000000, 0.000000000, 0.630000000]` | `1.110418207246720e-16` | `2.832769448823990e-16` | PASS |
| B | `[0.213683251, 0.070991768, 0.649474138]` | `[0.213683251, 0.070991768, 0.649474138]` | `6.938893903907228e-17` | `2.220446049250313e-16` | PASS |

Ambas configuraciones cumplen ampliamente:

```math
\Delta E < 10^{-6}\ \mathrm{m}
```

## Validación complementaria del TCP

También se verificó la cadena terminal hasta el TCP situado a 120 mm sobre `Z_6`.

La comparación utiliza:

- FK matemática: `forward_kinematics_tcp(q)` → `0T_TCP`
- ROS/URDF: TF `base_link -> tcp`

| Conf. | Posición FK [m] | Posición ROS [m] | Delta E [m] | Error máx. orientación | Resultado |
|---|---|---|---:|---:|---|
| A | `[0.494000000, 0.000000000, 0.630000000]` | `[0.494000000, 0.000000000, 0.630000000]` | `1.136133505277067e-16` | `2.832769448823990e-16` | PASS |
| B | `[0.319629005, 0.098248577, 0.698795165]` | `[0.319629005, 0.098248577, 0.698795165]` | `1.716587549445839e-16` | `2.220446049250313e-16` | PASS |

## Reproducción

La validación no depende de RViz ni del Joint State Publisher GUI.

En un entorno Ubuntu con ROS 2 Jazzy y el paquete `irb120_description` disponible en un workspace de `colcon`, iniciar primero:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select irb120_description
source install/setup.bash
ros2 launch irb120_description validacion_fk.launch.py
```

En otra terminal, con ROS 2 cargado y desde el directorio `hito1`:

```bash
source /opt/ros/jazzy/setup.bash
python3 validacion/validar_fk_ros.py
```

El script publica exactamente las configuraciones A y B, obtiene las transformaciones de ROS/TF y realiza automáticamente las comparaciones de posición y orientación.

## Resultado

Las configuraciones A y B producen coincidencia numérica entre la cinemática directa D-H y el modelo URDF/ROS.

El error posicional medido es del orden de `1e-16 m`, muy inferior al límite de aceptación de `1e-6 m`.

**Resultado global: PASS**
