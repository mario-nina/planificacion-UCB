# Panel de operación RViz

El plugin `imt342_irb120_rviz/JointPanel` usa el nodo ROS de RViz. El coordinador
del arranque gestiona simulación, telemetría y animación. La escena usa RobotModel
para la postura actual y otro RobotModel con TF prefijada para el fantasma.

- Los seis objetivos conservan radianes internamente y se actualizan con los
  controles numéricos o deslizadores. «Copiar postura actual» y «Poner objetivo
  a cero» cambian las seis juntas en una única publicación.
- La preferencia de grados/radianes se guarda con la configuración RViz y
  afecta la postura, los objetivos y los campos de orientación TCP.
- El análisis desplegable calcula IK para el TCP expresado en `base_link`.
  Los campos XYZ están en metros. La orientación se expresa en roll, pitch y
  yaw usando la preferencia angular. Una solución que corresponda a la última
  solicitud puede copiarse al objetivo con su botón.
- En Solo lectura se mantienen editables el objetivo articular y el TCP. El
  coordinador decide si dispone de una postura desde la que previsualizar.
  Aplicar modifica la simulación en Offline.
- El panel indica cuando no recibe actualizaciones de postura durante un
  segundo. El estado del coordinador informa la procedencia y la vigencia de
  la señal.

## Rutas analíticas

En Offline, abra «TCP 4×4 y rutas LSPB» y cargue FK actual o ingrese una matriz
homogénea de cuatro filas, separando los elementos con espacios y usando punto
decimal. «Cargar XYZ + RPY» usa los campos del editor anterior. La alternativa
de cuaternión usa posición XYZ de ese editor y componentes `qx qy qz qw`.
El coordinador valida la orientación y calcula las ramas Pieper admisibles.

«Calcular ramas y rutas» obtiene las rutas desde la postura actual. Los
controles de velocidad son porcentajes del límite del modelo para el
planificador, con escala global y ajuste por junta. Seleccione una rama para
ver sus errores FK, costo, tiempo y estado de colisión. Los botones de ruta
reproducen, pausan, continúan o cancelan la animación del fantasma. «Copiar
rama a objetivo» actualiza los campos articulares. El backend RWS/RAPID recibe
el destino solo al pulsar el control de ejecución existente. El controlador
ABB genera el perfil de su `MoveAbsJ` con el `speeddata` configurado en RAPID.

| Tópico | Tipo | Dirección del panel |
|---|---|---|
| `/imt342/target` | `sensor_msgs/JointState` | Publica cada objetivo editado |
| `/imt342/preview_target` | `sensor_msgs/JointState` | Envía objetivo completo a previsualizar |
| `/imt342/apply_target` | `sensor_msgs/JointState` | Envía objetivo completo a simular |
| `/imt342/preview_command` | `std_msgs/String` | Publica `cancel` |
| `/imt342/mode_command` | `std_msgs/String` | Solicita `offline`, `observe` o `control` |
| `/imt342/mode_state` | `std_msgs/String` | Lee modo confirmado, QoS transient local |
| `/imt342/actual` | `sensor_msgs/JointState` | Lee la postura actual |
| `/imt342/status` | `std_msgs/String` | Lee procedencia y estado de reproducción, QoS transient local |
| `/imt342/tcp_target` | `geometry_msgs/PoseStamped` | Envía solicitud de IK |
| `/imt342/ik_solution` | `sensor_msgs/JointState` | Lee solución, correlacionada con sello temporal |
| `/imt342/pose_plan_request` | `std_msgs/String` | Publica la matriz, escalas e ID de solicitud en JSON |
| `/imt342/pose_plan_result` | `std_msgs/String` | Lee ramas/rutas en JSON, correlacionadas por ID, QoS transient local |
| `/imt342/pose_plan_preview` | `std_msgs/String` | Publica la rama e ID en JSON o los comandos `pause`, `resume`, `cancel` |
| `/imt342/actual_pose_matrix` | `std_msgs/String` | Lee FK actual y procedencia en JSON, QoS transient local |
| `/imt342/kinematics_status` | `std_msgs/String` | Lee diagnóstico de cinemática, QoS transient local |

La prueba de los valores del panel se puede ejecutar con CTest o con un
compilador C++17 independiente.

```sh
c++ -std=c++17 -Wall -Wextra -Wpedantic -Werror -Iinclude \
  test/panel_values_test.cpp -o /tmp/imt342-panel-values-test
/tmp/imt342-panel-values-test
```
