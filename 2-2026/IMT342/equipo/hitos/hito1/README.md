# Hito 1 — ABB IRB 120

Implementación del modelo cinemático y del gemelo digital del manipulador **ABB IRB 120**, con validación cruzada entre la cinemática directa D-H y las transformaciones publicadas por ROS 2.

## Estructura

```text
hito1/
├── docs/                  # diagramas y fuentes externas
├── evidencias/            # desarrollo manuscrito
├── irb120_description/    # paquete ROS 2 del robot
├── modelo_cinematico/     # modelo D-H y cinemática directa
└── validacion/            # comparación FK ↔ ROS/TF
```

## Contenido principal

- `modelo_cinematico/`: parámetros D-H, cinemática directa en Python y documentación de la correspondencia D-H ↔ URDF.
- `irb120_description/`: URDF, mallas, launch files y configuración de RViz.
- `validacion/`: validación automática usando las mismas configuraciones articulares en el modelo D-H y en ROS/TF.
- `evidencias/`: desarrollo manuscrito utilizado durante la construcción del modelo.
- `docs/`: diagramas de flujo y registro de fuentes externas.

## Visualización en RViz

Con ROS 2 Jazzy y el paquete disponible en un workspace de `colcon`:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select irb120_description
source install/setup.bash
ros2 launch irb120_description display.launch.py
```

## Validación

En una terminal:

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch irb120_description validacion_fk.launch.py
```

En otra terminal, desde `hito1/`:

```bash
source /opt/ros/jazzy/setup.bash
python3 validacion/validar_fk_ros.py
```

Las configuraciones de prueba A y B cumplen el criterio posicional definido para el Hito:

```text
Delta E < 1e-6 m
```

Los resultados completos se encuentran en [`validacion/resultados.md`](validacion/resultados.md).

## Documentación

- [`docs/diagramas_flujo.md`](docs/diagramas_flujo.md)
- [`docs/FUENTES_EXTERNAS.md`](docs/FUENTES_EXTERNAS.md)
- [`modelo_cinematico/hoja_definicion_juntas.md`](modelo_cinematico/hoja_definicion_juntas.md)
- [`modelo_cinematico/sustentacion_dh_urdf.md`](modelo_cinematico/sustentacion_dh_urdf.md)
