# Arquitectura y convenciones — Hito 2

## 1. Propósito

Este documento define la arquitectura técnica y las convenciones utilizadas para desarrollar el Hito 2 del Proyecto Final Integrador de IMT-342 Robótica.

El proyecto implementará cinemática inversa analítica mediante Pieper, trayectorias LSPB, Jacobiano, análisis de singularidades y dinámica del manipulador ABB IRB 120.

Los requisitos académicos y sus fuentes se encuentran en `requisitos_y_fuentes.md`.

La organización de software descrita aquí es una decisión del equipo, no una estructura impuesta por el docente.

## 2. Continuidad académica e independencia técnica

El Hito 2 continúa el trabajo realizado en el Hito 1, donde se desarrolló y validó el modelo cinemático del ABB IRB 120.

Sin embargo, el Hito 2 se organizará como una unidad ejecutable independiente.

Esto significa que sus algoritmos, Notebook y pruebas no dependerán de archivos ubicados fuera de `hito2/`.

### Base procedente del Hito 1

Se reutilizarán mediante copia controlada:

- `dh.py`: transformación homogénea D-H y composición de transformaciones.
- `irb120_fk.py`: parámetros D-H, cinemática directa y transformación del TCP.

**Ubicación original:**

`equipo/hitos/hito1/modelo_cinematico/cinematica_directa/`

**Ubicación prevista en Hito 2:**

`hito2/src/`

Los archivos se identificarán como procedentes del Hito 1. No se presentarán como desarrollos originales del Hito 2.

Durante la incorporación se registrará el commit de procedencia y cualquier modificación realizada. Las adaptaciones necesarias para utilizar importaciones relativas no deberán alterar el modelo matemático.

La implementación histórica del Hito 1 permanecerá intacta.

Antes de considerar válida la migración se compararán los resultados de ambas versiones mediante configuraciones articulares conocidas.

## 3. Modelo geométrico

**Robot:** ABB IRB 120.

**Grados de libertad:** seis articulaciones revolutas.

**Convención:** Denavit–Hartenberg estándar.

**Variables articulares:**

`q = [q1, q2, q3, q4, q5, q6]`

Los ángulos articulares se expresan en radianes. Las distancias se expresan en metros.

### Parámetros D-H

| Junta | θ | d (m) | a (m) | α |
|---|---|---:|---:|---|
| J1 | q1 | 0.290 | 0 | −π/2 |
| J2 | q2 − π/2 | 0 | 0.270 | 0 |
| J3 | q3 | 0 | 0.070 | −π/2 |
| J4 | q4 | 0.302 | 0 | π/2 |
| J5 | q5 | 0 | 0 | −π/2 |
| J6 | q6 | 0.072 | 0 | 0 |

El offset de la segunda articulación debe conservarse en todos los módulos:

`θ2 = q2 − π/2`

La transformación homogénea total se calcula mediante:

`T_0_6 = T_0_1 · T_1_2 · T_2_3 · T_3_4 · T_4_5 · T_5_6`

### Marco de herramienta

Se distinguirán el marco terminal D-H `{6}` y el TCP académico.

La transformación fija desde `{6}` hasta el TCP consiste en una traslación de 0.120 m sobre el eje local `z6`, sin rotación adicional:

`T_0_TCP = T_0_6 · T_6_TCP`

Este desplazamiento es diferente del parámetro D-H `d6 = 0.072 m`.

Cuando la pose deseada corresponda al TCP, deberá convertirse al marco apropiado antes de resolver la cinemática inversa.

Las equivalencias entre marcos D-H y URDF se conservarán según la documentación validada en el Hito 1.

## 4. Organización del software

La estructura objetivo será:

```text
hito2/
├── README.md
├── requirements.txt
├── docs/
│   ├── requisitos_y_fuentes.md
│   ├── arquitectura_y_convenciones.md
│   └── diagramas_flujo.md
├── src/
│   ├── __init__.py
│   ├── dh.py
│   ├── irb120_fk.py
│   ├── pieper.py
│   ├── lspb.py
│   ├── jacobiano.py
│   ├── singularidades.py
│   └── dinamica.py
├── notebooks/
│   └── hito2_integracion.ipynb
├── validacion/
└── evidencias/
```

Esta estructura es planificada. Los archivos se incorporarán progresivamente conforme comience su implementación.

### `src/`

Contendrá las funciones matemáticas y los algoritmos reutilizables.

Cada módulo tendrá responsabilidades definidas y evitará duplicar funciones ya existentes.

### `notebooks/`

Contendrá el Notebook integrado para la entrega y sustentación académica.

El Notebook incluirá formulación matemática, explicación de los procedimientos, ejecución de funciones, tablas, gráficas, verificaciones y conclusiones.

Los cálculos reutilizables estarán implementados principalmente en `src/` y serán importados desde Jupyter.

### `validacion/`

Contendrá pruebas reproducibles de cinemática, trayectorias y análisis.

Se emplearán herramientas de prueba disponibles en Python, incorporando nuevas dependencias únicamente cuando sean justificadas.

### `evidencias/`

Contendrá material complementario necesario para sustentar el desarrollo, incluidos procedimientos matemáticos manuscritos y figuras seleccionadas.

No se duplicarán sin necesidad los resultados que ya estén documentados en el Notebook.

### `docs/`

Contendrá requisitos, convenciones, arquitectura y diagramas de flujo.

El historial cronológico de cambios se conservará principalmente mediante Git, sin una bitácora separada.

## 5. Interfaces conceptuales

| Módulo | Entrada | Salida |
|---|---|---|
| Pieper | Pose objetivo y convenciones geométricas | Candidatas articulares, ramas y estado |
| FK | Vector articular | Transformación homogénea |
| LSPB | Estados articulares y parámetros temporales | q(t), velocidad y aceleración |
| Jacobiano | Configuración articular y marco de interés | Jacobiano geométrico |
| Singularidades | Configuración o trayectoria | Indicadores de singularidad |
| Dinámica | q, velocidad, aceleración y parámetros dinámicos | Torques y contribuciones del modelo |

Las firmas definitivas, estructuras de datos y tolerancias se decidirán al estudiar e implementar cada módulo.

## 6. Ejecución y dependencias

La ejecución del Hito 2 deberá poder realizarse utilizando su propia carpeta y un entorno Python que cumpla con las dependencias declaradas en `requirements.txt`.

No se utilizarán rutas absolutas personales para localizar módulos.

Las importaciones del Notebook se configurarán y comprobarán desde un directorio de ejecución documentado.

La reproducibilidad deberá verificarse mediante una ejecución limpia del Notebook y de las pruebas.

El entorno ROS 2, si se requiere posteriormente, se documentará por separado del entorno Python académico.

## 7. Validación y trazabilidad

Cada módulo seguirá el procedimiento:

1. Estudiar el fundamento matemático.
2. Definir entradas, salidas y casos especiales.
3. Elaborar el diagrama de flujo correspondiente.
4. Implementar las funciones.
5. Ejecutar pruebas numéricas.
6. Registrar resultados y limitaciones.
7. Integrar el módulo al Notebook.

Un módulo se considerará validado solamente cuando existan resultados reproducibles y criterios de comprobación explícitos.

Los cambios se registrarán mediante commits Git de alcance lógico, siguiendo `equipo/docs/GIT_WORKFLOW.md`.

## 8. Fuentes de referencia

**Fuentes docentes:**

- `S09_material_oficial.pdf`, páginas 23, 25 y 49.
- `S10_material_oficial.pdf`, página 29.
- `Guia_PFI.pdf`, para los requisitos iniciales aún pendientes de aclaración.

**Base técnica del equipo:**

- `equipo/hitos/hito1/README.md`.
- `equipo/hitos/hito1/modelo_cinematico/`.
- `equipo/hitos/hito1/validacion/resultados.md`.

La reutilización de la FK, la arquitectura modular y la organización de evidencias son decisiones técnicas del equipo. Los requisitos académicos concretos se encuentran documentados por separado en `requisitos_y_fuentes.md`.

## 9. Registro de incorporación de la base cinemática

**Fecha de incorporación:** 2026-10-13.

**Procedencia:** Hito 1, directorio `modelo_cinematico/cinematica_directa/`.

**Referencia Git:** `eb42875`, commit de reorganización del modelo cinemático. Esta referencia identifica la versión consultada de los archivos, no necesariamente su creación original.

### Archivos incorporados

| Archivo | Hash Git del original | Adaptación |
|---|---|---|
| `dh.py` | `c5685190c5ed34d38721a71cce31f459d4873caa` | Ninguna |
| `irb120_fk.py` | `78fcbdb1b58d956fbb8b2e5fee762f9632dc00a4` | Importación relativa |

La modificación en `irb120_fk.py` fue exclusivamente:

```diff
-from dh import fk_chain
+from .dh import fk_chain
```

Se añadió `src/__init__.py` para organizar los archivos como paquete Python.

No se modificaron los parámetros D-H, las funciones matemáticas ni los archivos históricos del Hito 1.

### Validación funcional local

Se ejecutaron cuatro pruebas automatizadas con `unittest`:

- Configuración articular cero.
- Ortogonalidad y determinante de la matriz de rotación.
- Desplazamiento fijo de 0.120 m entre brida y TCP.
- Rechazo de vectores articulares de dimensión inválida.

**Resultado:** 4 pruebas satisfactorias, 0 fallidas.

Desde la raíz de `hito2/`, el comando de ejecución es:

```bash
python -m unittest discover -s validacion -p "test_*.py" -v
```

### Comparación con la implementación original

Se ejecutó una comparación temporal entre las funciones FK y TCP del Hito 1 y las copias locales del Hito 2 para tres configuraciones articulares:

1. `[0, 0, 0, 0, 0, 0]`
2. `[0.2, -0.3, 0.4, 0.5, -0.6, 0.7]`
3. `[-0.5, 0.4, -0.2, 1.0, 0.3, -0.8]`

Se utilizó una tolerancia absoluta de `1e-12`, sin tolerancia relativa, para comparar las matrices homogéneas.

**Resultado observado:** error máximo de `0.000e+00` en FK y TCP para las tres configuraciones.

La comparación con el Hito 1 fue una auditoría puntual de la incorporación. No constituye una dependencia de ejecución del Hito 2.

### Alcance de la validación

Las pruebas demuestran la equivalencia numérica de las implementaciones en los casos examinados y la ejecución correcta del paquete local.

No representan una nueva validación física del ABB IRB 120 ni validan todavía los algoritmos propios del Hito 2.

## 10. Límites articulares oficiales del ABB IRB 120

### 10.1. Fuentes primarias

Los rangos articulares nominales fueron contrastados con dos documentos oficiales del fabricante:

1. **ABB Robotics.** *Especificaciones del producto — IRB 120*. Documento `3HAC035960-005`, revisión W, sección 1.8.1 «Área de trabajo y tipo de movimiento», página impresa 37.
2. **ABB Robotics.** *Manual del producto — IRB 120*. Documento `3HAC035728-005`, revisión W, sección 2.2.2 «Área de trabajo y tipo de movimiento», página impresa 50.

Ambas fuentes coinciden en los siguientes rangos:

| Junta | Mínimo (°) | Máximo (°) |
|---|---:|---:|
| J1 | -165 | +165 |
| J2 | -110 | +110 |
| J3 | -110 | +70 |
| J4 | -160 | +160 |
| J5 | -120 | +120 |
| J6 | -400 | +400 |

Los límites se convertirán a radianes para su uso en Python, manteniendo los valores oficiales en grados como referencia documental.

### 10.2. Correspondencia geométrica

La página impresa 36 de `3HAC035960-005`, revisión W, identifica una configuración con J2 = 0° y J3 = 0°, cuyo centro de muñeca presenta X = 302 mm y Z = 630 mm.

La cinemática directa validada en el Hito 1 produce, para la configuración articular cero, una posición de centro de muñeca de `[0.302, 0.000, 0.630]` m.

Esta coincidencia respalda la correspondencia de la configuración de referencia entre el modelo académico y los datos geométricos publicados por ABB.

### 10.3. Condiciones y restricciones

Los valores corresponden a los rangos nominales publicados para el IRB 120.

ABB indica que el rango predeterminado del eje 6 puede ampliarse mediante configuración de software. No se considerará habilitada ninguna extensión sin evidencia de la configuración efectiva.

El *Manual del producto*, sección 2.4.1, página impresa 73, advierte que el área de trabajo puede necesitar restricciones adicionales por riesgos de colisión y que determinadas limitaciones requieren ajustes de configuración.

Por tanto, las comprobaciones de límites articulares realizadas en Python no equivalen a una certificación de seguridad ni sustituyen las restricciones operativas del controlador y de la instalación real.

### 10.4. Aplicación al método de Pieper

La cinemática inversa generará primero las soluciones matemáticas y después aplicará los rangos articulares respaldados por ABB.

La salida deberá conservar la identificación de rama y las razones del descarte, indicando qué articulación excede sus límites.

Se considerarán las representaciones articulares equivalentes separadas por vueltas completas cuando correspondan y estén dentro de límites; esto es especialmente importante para J6.

La posterior selección de una configuración para LSPB utilizará criterios técnicos declarados por el equipo. No se atribuirá al fabricante un algoritmo de selección que ABB no haya especificado en estas fuentes.

### 10.5. Limitaciones pendientes

No se ha comprobado la configuración de un controlador físico específico ni se dispone de evidencia sobre modificaciones particulares de límites, opciones de giro extendido o restricciones de la celda.

La validación actual contempla el modelo académico del IRB 120 con los rangos nominales de los manuales consultados.

La verificación de colisiones, velocidad, aceleración, carga y condiciones reales de operación pertenece a evaluaciones adicionales y no queda demostrada por el mero cumplimiento de los rangos angulares.


## 11. Integración de límites y selección de soluciones de Pieper

### 11.1. Arquitectura implementada

El Hito 2 utiliza tres módulos independientes:

- `src/pieper.py`: genera soluciones analíticas y comprueba su equivalencia con la pose objetivo mediante cinemática directa.
- `src/limites_irb120.py`: comprueba los rangos articulares nominales del fabricante y genera representaciones equivalentes de J6.
- `src/seleccion_pieper.py`: filtra las soluciones de Pieper y selecciona una configuración utilizando un criterio de desplazamiento articular.

Los límites nominales provienen de los manuales ABB identificados en la sección 10. El criterio de selección es una decisión técnica del equipo.

### 11.2. Interfaz de selección

La función principal es:

```python
filtrar_y_seleccionar_pieper(
    resultado,
    q_inicio,
    *,
    pesos=None,
)
```

Donde:

- `resultado` es un objeto `PieperResult` devuelto por `solve_pieper`.
- `q_inicio` es un vector de seis posiciones articulares iniciales, en radianes, que debe cumplir los límites nominales ABB.
- `pesos` es un vector opcional de seis ponderaciones no negativas, con al menos un valor positivo. Por defecto se utilizan seis ponderaciones iguales a uno.

La función devuelve un objeto `ResultadoSeleccion` con:

- `admisibles`: representaciones articulares dentro de límites, con sus errores FK y costos.
- `descartadas`: ramas rechazadas y violaciones articulares correspondientes.
- `seleccionada`: configuración con menor costo, o `None` si no existe una candidata admisible.
- `diagnosticos`: información de Pieper y advertencias de la selección.

### 11.3. Criterio matemático

La función utiliza un costo de desplazamiento articular ponderado:

\[
C(q)=\sum_{i=1}^{6}w_i(q_i-q_{inicio,i})^2
\]

Se selecciona la configuración admisible con el menor costo.

Este criterio no demuestra ausencia de colisiones, factibilidad temporal, cumplimiento de límites de velocidad/aceleración ni optimización de energía o torque.

### 11.4. Tratamiento de J6

El rango nominal ABB de J6 es de -400° a +400°.

Por ello, el módulo considera representaciones equivalentes:

\[
q_6'=q_6+2k\pi,\qquad k\in\mathbb Z
\]

Sólo conserva aquellas que cumplen los límites del fabricante.

Las representaciones equivalentes producen la misma pose según FK, pero pueden generar desplazamientos articulares diferentes desde la configuración inicial.

Las articulaciones J1–J5 no se modifican al generar estas variantes.

### 11.5. Validación realizada

El 13 de octubre de 2026 se ejecutó la batería completa de pruebas del Hito 2:

- 4 pruebas de FK.
- 7 pruebas de Pieper.
- 6 pruebas de límites ABB.
- 7 pruebas de selección.

**Resultado: 24 pruebas satisfactorias, ninguna fallida.**

La prueba de integración de referencia confirmó ocho ramas matemáticas, seis descartes por límites articulares, dos ramas geométricas admisibles y cuatro representaciones admisibles al considerar las vueltas de J6.

También se verificaron entradas inválidas, conservación de diagnósticos, rechazo de J3 fuera de rango, selección de la vuelta de J6 más cercana y manejo de representantes singulares.

### 11.6. Limitaciones

El tratamiento actual de singularidades utiliza representantes particulares y no explora exhaustivamente las familias continuas de soluciones.

La selección no incluye detección de colisiones, límites de velocidad, límites de aceleración ni una trayectoria temporal completa.

El vector seleccionado constituye una propuesta de `q_final` para LSPB, pendiente de comprobación de factibilidad temporal y de trayectoria.

El cumplimiento de límites nominales no certifica seguridad operativa ni equivalencia con la configuración de un controlador ABB real.

### 11.7. Fuentes y evidencias

**Fuentes oficiales:** manuales ABB referenciados en la sección 10 y guías W09-S01/W09-S02 del docente.

**Implementaciones:** `src/pieper.py`, `src/limites_irb120.py` y `src/seleccion_pieper.py`.

**Pruebas:** `validacion/test_pieper.py`, `validacion/test_limites_irb120.py` y `validacion/test_seleccion_pieper.py`.
