# Gate 0 — Auditoría del entorno ROS

Fecha: 2026-10-27
Ámbito: computadora personal de desarrollo
Estado: CONFIRMADO para los componentes indicados

## Sistema anfitrión

- Distribución: Fedora Workstation 44.
- Podman: versión 5.8.9.
- Modo rootless: true.
- Contenedores persistentes encontrados: ninguno.

## Imágenes disponibles

- localhost/ros-jazzy-imt342:latest
  - ID: 165548a73969
- docker.io/osrf/ros:jazzy-desktop-full-noble
  - ID: db0ff3f9fbde

## Entorno ROS verificado

Imagen ejecutada:
localhost/ros-jazzy-imt342:latest

- Sistema operativo: Ubuntu 24.04.4 LTS.
- ROS_DISTRO: jazzy.
- Python: 3.12.3.
- rviz2: paquete localizado.
- robot_state_publisher: paquete localizado.
- rclpy: paquete localizado e importación correcta.

## Procedimiento de verificación

En Fedora:
- cat /etc/fedora-release
- podman --version
- podman info
- podman images
- podman ps -a

En contenedor:
- podman run --rm --network=none
- source /opt/ros/jazzy/setup.bash
- ros2 pkg prefix
- python3 -c "import rclpy"

Fuente: resultados observados en terminal el 2026-10-27.

## Límites de la evidencia

CONFIRMADO:
- ROS 2 Jazzy disponible en el contenedor local.
- Paquetes básicos localizados.
- Importación de rclpy funcional.
- Ejecución rootless mediante Podman.

NO VERIFICADO:
- Sistema operativo de la PC del laboratorio.
- Drivers ROS compatibles con el controlador ABB.
- Comunicación real con el ABB.
- Licencias RWS o EGM.
- Funcionamiento gráfico de RViz2 en una sesión actual.
- Movimiento físico del robot.

No se realizó ninguna conexión al robot.
