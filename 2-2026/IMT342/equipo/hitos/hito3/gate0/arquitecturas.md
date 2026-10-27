# Gate 0 — Arquitecturas candidatas ROS–ABB

Fecha: 2026-10-27
Proyecto: Hito 3 progresivo — ECI-3/4
Robot de trabajo: ABB IRB 120

## 1. Objetivo

Comparar dos posibles arquitecturas de integración entre
ROS 2 y el controlador ABB del laboratorio.

Las arquitecturas son hipótesis técnicas.

No se ha demostrado compatibilidad ni se autoriza
comunicación física o movimiento del robot.

## 2. Fuentes

Fuente académica principal:
- S12_material_oficial.pdf
- w12_s01_gate0_ficha.pdf
- w12_s01_guia_autonoma_bitacora.pdf

Evidencia del proyecto:
- gate0/inventario.md
- gate0/entorno_ros.md
- gate0/marcos_y_tcp.md
- gate0/compatibilidad_ros_abb.md

Referencias técnicas:
- https://github.com/PickNikRobotics/abb_ros2
- https://github.com/ros-industrial/abb_robot_driver
- https://github.com/ros-industrial/abb_librws
- https://github.com/ros-industrial/abb_libegm

## 3. Entorno local disponible

CONFIRMADO:
- Fedora Workstation 44.
- Podman 5.8.9 rootless.
- Contenedor Ubuntu 24.04.4 LTS.
- ROS 2 Jazzy.
- Modelo URDF propio del IRB 120.
- Implementaciones propias FK, IK y Jacobiano.

NO VERIFICADO:
- Controlador físico y revisión.
- Versión de RobotWare.
- Licencias y opciones ABB.
- Interfaz permitida por el laboratorio.
- Compatibilidad del driver con Jazzy.
- TCP configurado en el controlador físico.

## 4. Arquitectura A — ROS 2 + RWS

### Descripción

Utilizar un cliente compatible con Robot Web Services
para consultar información del controlador ABB.

La prioridad es preparar Gate 1 mediante lecturas
autorizadas, sin movimiento del robot.

### Diagrama conceptual

```text
ROS 2 Jazzy
    |
    | Solicitudes de consulta
    v
Cliente / adaptador RWS
    |
    | Interfaz RWS compatible
    v
Controlador ABB
    |
    | Respuestas y estados
    v
Cliente / adaptador RWS
    |
    | Datos interpretados
    v
ROS 2 Jazzy
```

El protocolo, los mensajes y las capacidades
concretas deben verificarse.

### Requisitos críticos

- Controlador compatible con RWS.
- Versión de RobotWare identificada.
- Versión RWS compatible.
- Cliente ROS compatible con Jazzy.
- Acceso de red autorizado.
- Credenciales y permisos adecuados.
- Documentación de las consultas admitidas.

### Ventajas potenciales

- Permite priorizar observación sin movimiento.
- Puede servir para obtener estados del controlador.
- Separa la verificación de comunicación de la
  integración posterior del movimiento.

### Limitaciones

- Disponibilidad de RWS no verificada.
- No implica control de trayectorias en tiempo real.
- No garantiza compatibilidad con ROS 2 Jazzy.
- No demuestra disponibilidad de estados articulares
  hasta ejecutar una prueba autorizada.

### Requisito crítico no verificado

Modelo del controlador, RobotWare, interfaz RWS
disponible y autorización institucional.

Estado: PENDIENTE.

## 5. Arquitectura B — ROS 2 + RWS/EGM

### Descripción

Investigar una integración con RWS para funciones
compatibles de gestión/observación y EGM para un
posible canal de guía o control externo.

La implementación de movimiento pertenecería a
etapas posteriores, nunca a Gate 0.

### Diagrama conceptual

```text
ROS 2 Jazzy
    |
    +--> Cliente RWS
    |       |
    |       | Consultas / respuestas
    |       v
    |   Controlador ABB
    |
    +--> Interfaz EGM compatible
            |
            | Canal bidireccional EGM
            | sujeto a configuración
            v
        Controlador ABB
```

Las consignas de movimiento solamente podrían
habilitarse después de verificar compatibilidad,
seguridad y autorización.

### Requisitos críticos

- Modelo de controlador compatible.
- Versión de RobotWare compatible.
- Opción/licencia EGM instalada.
- Driver y dependencias compatibles con Jazzy.
- Configuración EGM documentada.
- Red y permisos institucionales.
- Parámetros articulares y marcos consistentes.
- Protocolo de seguridad aprobado.
- Operador y supervisor autorizados.

### Ventajas potenciales

- Posibilidad de integrar realimentación y
  modalidades de control externo.
- Posible vinculación con ros2_control.
- Permitiría investigar etapas posteriores
  de integración del sistema.

### Limitaciones

- Mayor complejidad de configuración.
- Dependencia de opciones y licencias.
- Requisitos de temporización y comunicación
  por validar.
- Riesgos asociados al control de movimiento.
- No existe todavía evidencia de funcionamiento
  con el ABB físico del laboratorio.

### Requisito crítico no verificado

Disponibilidad de EGM, compatibilidad de versiones
y condiciones de operación autorizada.

Estado: PENDIENTE.

## 6. Comparación

| Criterio | Arquitectura A | Arquitectura B |
|---|---|---|
| Interfaz | RWS | RWS y EGM |
| Objetivo inicial | Observación | Observación y control futuro |
| Movimiento en Gate 0 | No | No |
| Licencia EGM | No requerida para la ruta RWS propuesta | Requiere verificarla |
| Driver ROS 2 Jazzy | Pendiente | Pendiente |
| RobotWare | Pendiente | Pendiente |
| Red institucional | Pendiente | Pendiente |
| Seguridad | Obligatoria | Obligatoria |
| Complejidad prevista | Menor | Mayor |
| Estado | Hipótesis | Hipótesis |

## 7. Flujo bidireccional de señales

### Flujo de solicitudes

```text
Modelo / aplicación ROS
    |
    v
Nodo o adaptador compatible
    |
    v
Interfaz ABB autorizada
    |
    v
Controlador ABB
```

En Gate 1, las solicitudes propuestas serán
únicamente de lectura y sin movimiento.

### Flujo de retorno

```text
Controlador ABB
    |
    v
Interfaz ABB autorizada
    |
    v
Nodo o adaptador compatible
    |
    v
Estados recibidos en ROS
    |
    v
Registro de evidencia y validación
```

Debe demostrarse que los estados proceden
del controlador físico.

Un joint_states simulado no constituye
evidencia de comunicación real.

## 8. Decisión provisional

Prioridad: INVESTIGAR ARQUITECTURA A.

Justificación:
- Permite enfocar las primeras pruebas en lectura.
- Es coherente con la preparación de Gate 1
  sin movimiento.
- Evita asumir desde el inicio la disponibilidad
  de la opción EGM.

Arquitectura B:
Mantener como alternativa condicionada a
la auditoría del controlador y las licencias.

La decisión no constituye una selección
definitiva ni autorización de implementación.

## 9. Información necesaria del laboratorio

1. Modelo y revisión del controlador.
2. Versión y revisión de RobotWare.
3. Opciones y licencias instaladas.
4. Disponibilidad de RWS.
5. Disponibilidad de EGM.
6. Métodos y restricciones de conexión.
7. Credenciales y permisos autorizados.
8. Computador y sistema operativo disponible.
9. Operador y supervisor autorizados.
10. Antecedentes del proyecto de ajedrez.

## 10. Condiciones para avanzar

Antes de Gate 1:
- Identificar una interfaz compatible.
- Confirmar permisos institucionales.
- Definir solicitudes de lectura permitidas.
- Establecer resultados y evidencias esperadas.
- Obtener autorización para la prueba.

Antes de cualquier movimiento físico:
- Completar las verificaciones técnicas.
- Cumplir el checklist de seguridad.
- Disponer de operador y supervisión autorizados.
- Obtener autorización explícita de movimiento.

## 11. Conclusión

ROS 2 Jazzy está disponible en el entorno personal.

Existen interfaces y drivers candidatos, pero
no se ha verificado su compatibilidad con el
controlador ABB físico del laboratorio.

Gate 0 continúa en investigación documental.
