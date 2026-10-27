# Gate 0 — Inventario local y compatibilidad ROS-ABB

Fuente: S12_material_oficial.pdf
Documento original: w12_s01_gate0_ficha.pdf

Grupo: pendiente
Fecha de auditoría: pendiente
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
| 7 | PC disponible y SO | PC del laboratorio no verificada | Pendiente | — | Por asignar | Identificar equipo disponible |
| 8 | Distribución ROS | ROS 2 Jazzy documentado históricamente | Pendiente | Documentación previa del entorno | Por asignar | Revalidar instalación |
| 9 | Driver candidato y versión | No seleccionado | Pendiente | — | Por asignar | Investigar alternativas |
| 10 | Requisitos y versiones | No contrastados | Pendiente | — | Por asignar | Elaborar matriz de compatibilidad |
| 11 | Red y acceso institucional | No verificados | Pendiente | — | Por asignar | Consultar permisos |
| 12 | Manual y seguridad local | No verificados | Pendiente | — | Por asignar | Consultar protocolo del laboratorio |
| 13 | URDF, SRDF y otros modelos | URDF propio localizado; versiones por auditar | Pendiente | Hito 1 | Por asignar | Identificar commit y convenciones |
| 14 | Código previo de ajedrez | No auditado | Pendiente | — | Por asignar | Localizar antecedente |
| 15 | Operador autorizado | No identificado | Pendiente | — | Por asignar | Confirmar con docente |

## Observaciones

- La disponibilidad de ROS no demuestra integración con ABB.
- El URDF propio no demuestra compatibilidad con el robot físico.
- No se realizarán pruebas físicas sin autorización.

## Próximas actividades

1. Auditar versiones y commits de los modelos propios.
2. Verificar el entorno ROS disponible.
3. Investigar controlador, interfaces y restricciones.
4. Elaborar las hipótesis de compatibilidad.
