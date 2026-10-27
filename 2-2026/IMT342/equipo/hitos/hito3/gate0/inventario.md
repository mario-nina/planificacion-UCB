# Gate 0 — Inventario local y compatibilidad ROS-ABB

Fuente: S12_material_oficial.pdf
Documento original: w12_s01_gate0_ficha.pdf

Grupo: pendiente
Fecha de auditoría: 2026-10-27 (documental y entorno local; ABB físico no inspeccionado)
Responsables: pendiente
Supervisión: pendiente

## Criterio de auditoría

Un elemento se considera confirmado únicamente cuando
existe evidencia verificable.

Pendiente no significa ausente.

Esta ficha no constituye autorización para conectar,
configurar ni mover el robot físico.

## Inventario oficial

| N.º | Elemento | Dato observado | Estado | Evidencia | Responsable | Próxima verificación |
|---|---|---|---|---|---|---|
| 1 | Manipulador y modelo exacto | Modelo de trabajo IRB 120; equipo físico no verificado | Pendiente | Hitos anteriores | Por asignar | Identificar placa del robot |
| 2 | Controlador, modelo y revisión | No verificado | Pendiente | — | Por asignar | Consultar laboratorio |
| 3 | Versión de RobotWare | No verificada | Pendiente | — | Por asignar | Consultar controlador |
| 4 | Opciones y licencias | No verificadas | Pendiente | — | Por asignar | Consultar configuración |
| 5 | Interfaz permitida | No verificada | Pendiente | — | Por asignar | Investigar RWS, EGM u otra |
| 6 | Estado y método de conexión | No verificado | Pendiente | — | Por asignar | Consultar laboratorio |
| 7 | PC disponible y SO | Fedora 44 confirmado en PC personal; PC del laboratorio no verificada | Parcialmente verificado | gate0/entorno_ros.md | Por asignar | Identificar PC del laboratorio |
| 8 | Distribución ROS | ROS 2 Jazzy en Ubuntu 24.04.4 LTS mediante Podman rootless | Confirmado (entorno local) | gate0/entorno_ros.md | Por asignar | Evaluar compatibilidad ROS-ABB |
| 9 | Driver candidato y versión | abb_ros2 rolling e6486f7 y abb_robot_driver master d167895 identificados; compatibilidad no probada | Parcialmente verificado | gate0/compatibilidad_ros_abb.md | Por asignar | Verificar compatibilidad con Jazzy, RobotWare e IRB 120 |
| 10 | Requisitos y versiones | Requisitos candidatos documentados; sin contraste con controlador físico | Pendiente | gate0/compatibilidad_ros_abb.md; gate0/arquitecturas.md | Por asignar | Completar matriz de compatibilidad real |
| 11 | Red y acceso institucional | No verificados | Pendiente | — | Por asignar | Consultar permisos |
| 12 | Manual y seguridad local | No verificados | Pendiente | — | Por asignar | Consultar protocolo del laboratorio |
| 13 | URDF, SRDF y otros modelos | URDF IRB 120 disponible; FK/IK/J validados offline; SRDF no localizado entre archivos rastreados; correspondencia física pendiente | Parcialmente verificado | gate0/marcos_y_tcp.md; gate0/plan_validacion_cinematica.md; evidencias/validacion_cinematica.json | Por asignar | Verificar marcos, TCP y correspondencia con ABB físico |
| 14 | Código previo de ajedrez | No auditado | Pendiente | — | Por asignar | Localizar antecedente |
| 15 | Operador autorizado | No identificado | Pendiente | — | Por asignar | Confirmar con docente |

## Observaciones

- La disponibilidad de ROS no demuestra integración con ABB.
- El URDF propio no demuestra compatibilidad con el robot físico.
- No se realizarán pruebas físicas sin autorización.

## Próximas actividades

1. Solicitar identificación de controlador y RobotWare.
2. Auditar licencias, interfaces y permisos del laboratorio.
3. Verificar compatibilidad real de los drivers candidatos.
4. Formalizar criterios de aceptación de Gate 2.
5. Asignar responsables principales y suplentes.
6. Mantener la decisión NO-GO para movimiento físico.
