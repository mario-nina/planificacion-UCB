# Requisitos y fuentes — Hito 2

## 1. Propósito

Este documento identifica los requisitos académicos y las evidencias necesarias para desarrollar y validar el Hito 2 del Proyecto Final Integrador de IMT-342 Robótica, utilizando el manipulador ABB IRB 120.

Se priorizan las especificaciones operativas más recientes de las semanas 9 y 10. Las indicaciones anteriores del programa y la Guía PFI permanecen registradas cuando su vigencia no ha sido aclarada.

Este documento **no acredita implementación**: las validaciones deberán respaldarse con código ejecutado y resultados reproducibles.

## 2. Fuentes oficiales y prioridad

| Fuente | Documento original | Aplicación |
|---|---|---|
| `S09_material_oficial.pdf`, p. 49 | `w09_s02_guia_hito_2.pdf` | Guía final de integración y módulos requeridos |
| `S10_material_oficial.pdf`, p. 29 | `w10_s01_avance_hito_2.pdf` | Seguimiento de implementación y validación |
| `S09_material_oficial.pdf`, p. 25 | `w09_s01_guia_hito_2.pdf` | Evidencias detalladas por componente |
| `S09_material_oficial.pdf`, p. 23 | `w09_s01_esqueleto_notebook.pdf` | Estructura oficial del Notebook |
| `Guia_PFI.pdf`, p. 3 | Guía general del PFI | Restricciones y entregables iniciales |
| `00_programa_IMT342.pdf`, p. 18 | Programa de la asignatura | Marco de evaluación |

**Criterio de interpretación:** las guías recientes determinan las prioridades de implementación. No se presumirá que sustituyen requisitos anteriores salvo confirmación del docente.

## 3. Componentes y evidencias exigidas

### 3.1. Cinemática inversa analítica — Pieper

**Fuente:** W09-S01 y W09-S02.

- Desarrollar el método de Pieper analíticamente en Python.
- Contemplar las ocho ramas de la solución general.
- Implementar identificación, filtrado y selección de soluciones.
- Identificar ramas inexistentes, duplicadas o degeneradas cuando corresponda.
- No sustituir el método analítico por MoveIt! ni por solucionadores automáticos.

**Evidencia:** ecuaciones, implementación, tabla de ramas y resultados verificables.

### 3.2. Verificación mediante cinemática directa

**Fuente:** W09-S01 y W09-S02.

- Reutilizar el modelo D-H y la FK validados en el Hito 1.
- Reconstruir mediante FK la pose correspondiente a cada solución aceptada.
- Comparar posición y orientación con la pose objetivo.
- Declarar las tolerancias numéricas empleadas.

**Evidencia:** resultados de comparación y errores numéricos.

### 3.3. Trayectorias LSPB

**Fuente:** W09-S01 y W09-S02.

- Determinar y justificar los parámetros temporales, incluido el tiempo de mezcla `tc`.
- Calcular posición, velocidad y aceleración articulares: `q(t)`, `q̇(t)` y `q̈(t)`.
- Comprobar condiciones iniciales y finales, continuidad y límites.
- Presentar gráficas con ejes y unidades.

**Evidencia:** ecuaciones por tramos, funciones ejecutables, comprobaciones y gráficas.

**Nota técnica:** el LSPB ideal puede presentar discontinuidades de aceleración en las transiciones entre fases; no debe confundirse continuidad de posición y velocidad con continuidad de aceleración.

### 3.4. Jacobiano geométrico

**Fuente:** W09-S01 y W09-S02.

- Implementar el Jacobiano correspondiente al modelo del IRB 120.
- Explicar la interpretación física de sus columnas.
- Realizar **al menos un caso de validación manual**.
- Evaluar el Jacobiano en las configuraciones relevantes de la trayectoria.

**Evidencia:** formulación matemática, código, validación manual y comparación de resultados.

La guía exige validación manual, pero no especifica que todas las derivaciones deban entregarse manuscritas.

### 3.5. Análisis de singularidades

**Fuente:** W09-S01 y W09-S02.

- Definir y documentar un experimento de singularidad.
- Evaluar rango o determinante del Jacobiano, según corresponda.
- Interpretar la pérdida de movilidad instantánea.
- Presentar resultados al aproximarse a una configuración singular.
- Incorporar el análisis a lo largo de la trayectoria cuando sea aplicable.

**Evidencia:** experimento reproducible, resultados numéricos, gráficas e interpretación física.

### 3.6. Dinámica y torque

**Fuente:** W09-S01 y W09-S02.

- Identificar y formular el modelo dinámico utilizado.
- Calcular torque a lo largo de la trayectoria.
- Separar las contribuciones inerciales, de Coriolis/centrífugas cuando apliquen y gravitacionales.
- Declarar parámetros físicos, unidades, supuestos y limitaciones.

**Evidencia:** modelo documentado, cálculo ejecutable, gráficas de torque y discusión técnica.

No se presentarán parámetros físicos supuestos como datos reales del fabricante.

### 3.7. Notebook integrado

**Fuente:** W09-S01, W09-S02 y W10-S01.

La estructura indicada por el docente comprende:

1. Planteamiento del problema y modelo del robot.
2. Cinemática inversa analítica de Pieper.
3. Verificación FK.
4. Jacobiano geométrico.
5. Análisis de singularidades.
6. Generación de trayectoria LSPB.
7. Dinámica y análisis de torque.
8. Gráficas y discusión.
9. Validación y conclusiones.

**Evidencia:** Notebook ejecutable con ecuaciones, código, resultados, unidades, gráficas y argumentación técnica.

Se propone implementar los algoritmos en módulos `.py` importados desde el Notebook. Esta es una decisión de arquitectura del equipo, no una exigencia textual del docente.

## 4. Evidencias complementarias del equipo

Para respaldar el aprendizaje y la defensa se incorporarán, cuando corresponda:

- Diagramas de flujo de los algoritmos.
- Procedimientos matemáticos manuscritos.
- Pruebas automatizadas y resultados reproducibles.
- Figuras y tablas de validación.

Estos elementos complementan las exigencias oficiales. No sustituyen las evidencias computacionales.

## 5. Requisitos anteriores pendientes de aclaración

| Indicación | Fuente | Tratamiento |
|---|---|---|
| Nodo ROS en Python que reciba una pose, ejecute Pieper y genere LSPB | `Guia_PFI.pdf`, p. 3 | Alcance por confirmar; no se considera eliminado |
| Demostración de singularidad en `q5 = 0 rad`, incluyendo velocidades y torques | `Guia_PFI.pdf`, p. 3 | Mantener el caso bajo revisión y distinguir singularidad cinemática de torque elevado |
| Perfiles LSPB en Gazebo | `00_programa_IMT342.pdf`, p. 18 | Alcance por confirmar |
| Fecha y modalidad definitiva de entrega y defensa | Comunicación vigente del docente | Pendiente de verificación |

La singularidad por sí sola no implica necesariamente torque elevado. Toda demostración de crecimiento de torque debe justificarse mediante la trayectoria, el modelo dinámico y las condiciones utilizadas.

## 6. Criterio de validación y cierre

De acuerdo con `w10_s01_avance_hito_2.pdf`, **no se marcará un componente como validado únicamente porque se conozca su fórmula**. Debe existir implementación ejecutada y evidencia demostrable.

Cada componente tendrá uno de los siguientes estados:

- **No iniciado:** no existe implementación formal.
- **En progreso:** existe trabajo parcial, aún sin validación completa.
- **Validado:** existe ejecución reproducible y evidencia suficiente.
- **Bloqueado:** existe una dependencia o problema técnico que impide continuar.

El estado se actualizará cuando exista evidencia real; no se marcarán componentes como completos de forma anticipada.

## 7. Base técnica reutilizada

Se reutilizan exclusivamente los resultados validados del Hito 1, en particular:

- Modelo Denavit–Hartenberg del ABB IRB 120.
- Cinemática directa en Python.
- Convención articular, incluido `θ2 = q2 − π/2`.
- Transformación fija desde el marco `{6}` al TCP académico.
- Documentación y resultados de validación FK–ROS/TF.

**Referencia interna:** `equipo/hitos/hito1/`.

Los módulos nuevos del Hito 2 se desarrollarán progresivamente y serán comprobados contra esta base.
