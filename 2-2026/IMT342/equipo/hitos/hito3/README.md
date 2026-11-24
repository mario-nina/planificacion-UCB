# Hito 3 — Integración ROS 2 con ABB IRB 120

**IMT-342 Robótica · Universidad Católica Boliviana · Gestión 2-2026 · ECI-3/4**

Sistema de modelado, planificación, visualización y comunicación con el manipulador industrial ABB IRB 120 de seis articulaciones revolutas. La aplicación integra ROS 2 Jazzy, RViz, cinemática analítica y diferencial, planificación de trayectorias y comunicación RWS/RAPID con el controlador IRC5.

## Organización

| Ruta | Contenido |
| --- | --- |
| [software/](software/) | Aplicación, paquetes ROS 2, servidores RAPID, scripts y documentación de reproducción |
| [gate0/](gate0/) | Factibilidad, inventario y decisiones iniciales de integración |
| [config/](config/) | Referencia histórica del modelo |
| [validacion/](validacion/) | Validación cinemática inicial |
| [evidencias/](evidencias/) | Registros seleccionados y procedencia experimental |
| [informe/](informe/) | Informe final ECI-3/4, fuentes LaTeX, figuras y resultados |

## Alcance

La aplicación dispone de modos offline, observación y control físico, dos backends RWS/RAPID, previsualización, tareas repetibles, análisis cinemático y gráficas. La entrega conserva el código fuente de la versión cerrada el 24/11/2026. El estado de validación se describe en [validación académica](software/docs/validacion.md). Las pruebas offline y los antecedentes físicos se distinguen por fecha y revisión; no constituyen certificación de seguridad ni calibración independiente.

## Documentación

- [Informe final ECI-3/4 (PDF)](informe/informe.pdf)

- [Arquitectura y componentes](software/docs/arquitectura.md)
- [Instalación y operación](software/docs/operacion.md)
- [Modelo, planificación y tareas](software/docs/modelo_y_tareas.md)
- [Validación y referencias](software/docs/validacion.md)

## Evidencia complementaria

- [Validación matemática histórica (offline)](evidencias/software/matematica_hito2/README.md).
- [Observación RWS del 21/11/2026](evidencias/software/observacion_rws_20261121/README.md).
- [Ensayo físico v2 del 23/11/2026](evidencias/software/ensayo_v2_20261123/README.md).
- [Aplicación virtual del 23/11/2026](evidencias/software/tareas_virtuales_20261123/README.md).
- [Entorno de ejecución e inventario de dependencias](software/docs/entorno/README.md).
- [Compilación y pruebas offline de la copia Git](evidencias/software/compilacion_copia_git/README.md).
