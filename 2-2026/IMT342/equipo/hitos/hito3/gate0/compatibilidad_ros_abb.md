# Gate 0 — Investigación de compatibilidad ROS–ABB

Fecha: 2026-10-27
Estado: investigación documental
Robot de trabajo: ABB IRB 120

## 1. Fuente académica

S12_material_oficial.pdf
- Ficha Gate 0
- Arquitecturas técnicas como hipótesis
- Guía de trabajo autónomo W12-S01

## 2. Entorno local confirmado

- Fedora 44.
- Podman rootless 5.8.9.
- Ubuntu 24.04.4 LTS en contenedor.
- ROS 2 Jazzy disponible.
- URDF propio del IRB 120.

Evidencias:
- gate0/entorno_ros.md
- gate0/marcos_y_tcp.md

## 3. Tecnologías candidatas

### RWS

Robot Web Services permite consultar recursos del
controlador y realizar operaciones admitidas por su API.

RWS 1.0 corresponde a RobotWare 6.
RWS 2.0 corresponde a RobotWare 7.

Estado en laboratorio: NO VERIFICADO.

### EGM

Externally Guided Motion permite modalidades de
guía y control externo del movimiento.

Para abb_libegm con RobotWare 6 se documentan:
- RobotWare 6.07.01 o posterior.
- Opción/licencia EGM 689-1.

Estado en laboratorio: NO VERIFICADO.

No se autoriza movimiento físico.

### RAPID

Se investigará la posibilidad de aprovechar programas
RAPID y el antecedente académico de ajedrez.

Código histórico: NO RECUPERADO.
Interfaz utilizada: NO VERIFICADA.

## 4. Software candidato

### ros-industrial/abb_robot_driver

Implementación principalmente ROS 1.
Compatibilidad directa con Jazzy: NO VERIFICADA.
Rama de referencia: master.
Commit remoto consultado: d1678957d3792e6aa54582ff894db62b6a51dac0.
No se selecciona para instalación.

### PickNikRobotics/abb_ros2

Implementación ROS 2 con componentes RWS y EGM.

Compatibilidad con ROS 2 Jazzy: NO VERIFICADA.
Configuración específica IRB 120: NO VERIFICADA.
Rama de referencia: rolling.
Commit remoto consultado: e6486f7409073cdb2a10d6878bb50191dd50172b.
Pruebas de compilacion o ejecucion: NO REALIZADAS.

El IRB 1200 de los ejemplos no debe confundirse
con el IRB 120 utilizado en este proyecto.

## 5. Arquitecturas candidatas

### Arquitectura A: ROS 2 + RWS

Objetivo:
- Comunicación y lectura de estados sin movimiento.

Requisitos pendientes:
- Modelo del controlador.
- RobotWare y versión RWS.
- Permisos y credenciales institucionales.
- Cliente compatible con Jazzy.

### Arquitectura B: ROS 2 + RWS/EGM

Objetivo:
- Comunicación, realimentación y posible control
  externo en etapas posteriores.

Requisitos pendientes:
- Controlador y RobotWare compatibles.
- Licencia EGM instalada.
- Driver compatible y configuración validada.
- Condiciones de seguridad y autorización.

## 6. Decisión provisional

Investigar arquitectura A como ruta inicial
para la preparación de Gate 1.

Mantener B como alternativa condicionada a
licencias, compatibilidad y autorizaciones.

NO existe todavía una arquitectura validada
con el controlador físico del laboratorio.

## 7. Información requerida del laboratorio

- Modelo y revisión exactos del controlador.
- Versión y revisión de RobotWare.
- Opciones/licencias instaladas.
- Interfaz de comunicación permitida.
- Disponibilidad de RobotStudio/Virtual Controller.
- Restricciones de red.
- Operador y supervisor autorizados.
- Código y documentación del proyecto de ajedrez.

## 8. Referencias técnicas externas

ABB Robot Web Services:
https://www.abb.com/global/en/areas/robotics/downloads

ROS-Industrial abb_robot_driver:
https://github.com/ros-industrial/abb_robot_driver

ROS-Industrial abb_librws:
https://github.com/ros-industrial/abb_librws

ROS-Industrial abb_libegm:
https://github.com/ros-industrial/abb_libegm

PickNik abb_ros2:
https://github.com/PickNikRobotics/abb_ros2

Estas fuentes describen capacidades y requisitos
generales, no certifican el hardware del laboratorio.

## 9. Hipótesis de compatibilidad

### H1 — RWS disponible en el controlador

Fuente: documentación de RWS y ficha Gate 0.
Qué verificaría: versión de RobotWare, versión RWS y
disponibilidad de consultas autorizadas.
Qué impide concluir: controlador y RobotWare del
laboratorio todavía desconocidos.
Estado: NO VERIFICADO.

### H2 — Opción EGM disponible

Fuente: documentación de abb_libegm y abb_robot_driver.
Qué verificaría: opción EGM instalada, versión de
RobotWare compatible y configuración documentada.
Qué impide concluir: licencias y opciones sin auditar.
Estado: NO VERIFICADO. No probar movimiento.

### H3 — Integración RAPID histórica reutilizable

Fuente: antecedente de ajedrez mencionado en S12.
Qué verificaría: código fuente, interfaz, dependencias
y evidencia de funcionamiento previo.
Qué impide concluir: implementación no recuperada.
Estado: NO VERIFICADO.

### H4 — Driver ROS 2 compatible con Jazzy e IRB 120

Fuente: PickNikRobotics/abb_ros2.
Qué verificaría: dependencias, compilación, interfaces
y configuración específica del IRB 120.
Qué impide concluir: no se ha realizado una compilación
ni validado el controlador del laboratorio.
Estado: NO VERIFICADO.

### H5 — Simulación mediante controlador virtual

Fuente: guía S12 y documentación de RobotStudio.
Qué verificaría: disponibilidad de RobotStudio,
Virtual Controller, licencias y configuración equivalente.
Qué impide concluir: software y licencias no auditados.
Estado: NO VERIFICADO.

## 10. Alcance de las referencias Git

Los commits registrados se obtuvieron mediante git ls-remote.

Demuestran qué revisión anunciaban las ramas consultadas,
pero no constituyen pruebas de compilación ni compatibilidad.

Las pruebas de integración requieren una auditoría posterior
del software y del hardware del laboratorio.
