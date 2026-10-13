# Hito 2 — Cinemática Inversa Analítica (Pieper) y Perfiles LSPB

Desarrollo del segundo hito del Proyecto Final Integrador de **IMT-342 Robótica**, utilizando como manipulador de referencia el **ABB IRB 120** de seis grados de libertad.

## Objetivo

Implementar y validar un sistema que permita obtener configuraciones articulares a partir de una pose objetivo, generar trayectorias LSPB y analizar su comportamiento cinemático y dinámico.

## Componentes del proyecto

1. Cinemática inversa analítica mediante el método de Pieper.
2. Verificación de soluciones mediante cinemática directa.
3. Generación de trayectorias LSPB.
4. Construcción y validación del Jacobiano geométrico.
5. Análisis de singularidades.
6. Modelo dinámico y cálculo de torques.
7. Integración y presentación de resultados en Jupyter Notebook.

## Modelo de referencia

Se reutiliza el modelo D-H y la cinemática directa desarrollados y validados en el Hito 1.

Las implementaciones nuevas deben respetar las convenciones articulares, los marcos de referencia y la transformación del TCP establecidas previamente.

## Organización

El proyecto utilizará una estructura modular:

- `docs/`: arquitectura, convenciones, requisitos y diagramas.
- `src/`: algoritmos implementados en Python.
- `notebooks/`: desarrollo matemático, ejecución y resultados integrados.
- `validacion/`: pruebas reproducibles.
- `evidencias/`: evidencias complementarias, incluidos desarrollos manuscritos cuando corresponda.
- `requirements.txt`: dependencias del entorno Python.

Los directorios se incorporarán conforme sean necesarios.

## Fuentes académicas

Se priorizan las instrucciones recientes del docente:

- Semana 9, sesión 1: guía de desarrollo y estructura del Notebook.
- Semana 9, sesión 2: guía final de integración del Hito 2.
- Semana 10, sesión 1: seguimiento de implementación y validación.

Las indicaciones iniciales del PFI sobre ROS y Gazebo permanecen sujetas a confirmación de alcance.

## Estado

**En preparación.** Se ha auditado el repositorio, verificado la base cinemática del Hito 1 y registrado las dependencias Python. Los módulos del Hito 2 todavía no están implementados ni validados.

## Desarrollo y validación

Cada componente se desarrollará mediante fundamento matemático, diseño del algoritmo, implementación, pruebas y documentación de resultados.

El Notebook integrará las explicaciones, las funciones y las evidencias necesarias para la sustentación académica.
