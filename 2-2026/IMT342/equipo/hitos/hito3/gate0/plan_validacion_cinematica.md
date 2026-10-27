# Gate 0 — Plan de validación cinemática FK/IK/J

Fecha: 2026-10-27
Robot: ABB IRB 120
Estado: EJECUTADO OFFLINE / SIN VALIDACION FISICA

## 1. Objetivo

Diseñar tres casos de validación cinemática offline:
dos configuraciones regulares y una cercana a singularidad.

Este plan no implica pruebas sobre el robot físico.

## 2. Fuente académica

S12_material_oficial.pdf
Guía autónoma W12-S01, actividad 4.

Requisitos:
- Tres configuraciones de ensayo.
- Marcos de base {B} y TCP.
- Métricas ep, eR y eJ.
- Distinguir torque externo y dinámica completa.

## 3. Implementaciones disponibles

Origen:
hito2/src/

- irb120_fk.py: FK y transformación del TCP.
- pieper.py: IK analítica.
- jacobiano.py: Jacobiano geométrico.
- singularidades.py: análisis de singularidad.
- limites_irb120.py: comprobación de límites.

No se modificarán estas implementaciones.

## 4. Convenciones

Unidad angular: rad.
Unidad de longitud: m.

Base de cálculo: marco {0} del modelo D-H.
Marco objetivo: TCP académico.

Transformación de brida a TCP:
- Traslación local: [0, 0, 0.120] m.
- Rotación relativa: identidad.

Las equivalencias entre {0}, base_link,
{6}, dh_frame_6 y el TCP del URDF
deberán comprobarse explícitamente.

No se considera verificado el TCP físico del ABB.

## 5. VAL-01 — Configuración regular A

q = [0.30, -0.40, 0.20, 0.30, -0.20, 0.80]

Fuente de configuración:
hito2/validacion/test_pieper.py

Procedimiento previsto:

1. Verificar límites articulares.
2. Verificar rango seis del Jacobiano.
3. Calcular FK del TCP.
4. Utilizar la pose como objetivo de Pieper.
5. Obtener soluciones IK.
6. Comprobar admisibilidad de cada solución.
7. Reconstruir la pose usando FK.
8. Calcular ep y eR.
9. Comparar Jacobiano analítico con diferencias
   finitas para obtener eJ.

Estado: EJECUTADO OFFLINE EN HITO 3.

## 6. VAL-02 — Configuración regular B

q = [0.30, -0.35, 0.25, 0.40, 0.10, -0.60]

Fuente de configuración:
hito2/validacion/test_singularidades.py
y secuencia de experimento q5.

Procedimiento previsto:

1. Verificar límites articulares.
2. Verificar rango seis del Jacobiano.
3. Calcular FK del TCP.
4. Resolver IK para la pose objetivo.
5. Reconstruir las poses admisibles.
6. Evaluar ep y eR.
7. Comparar J analítico y J por
   diferencias finitas.
8. Registrar eJ y valores singulares.

Estado: EJECUTADO OFFLINE EN HITO 3.

## 7. VAL-03 — Proximidad a singularidad

q = [0.30, -0.35, 0.25, 0.40, 0.01, -0.60]

Referencia singular:
q5 = 0 rad.

Fuente:
hito2/validacion/test_singularidades.py

Procedimiento previsto:

1. Verificar límites articulares.
2. Evaluar J para q5 = 0.01 rad.
3. Evaluar J para q5 = 0 rad.
4. Registrar rango numérico.
5. Registrar menor valor singular.
6. Registrar número de condición.
7. Comparar J analítico con diferencias finitas.
8. Registrar eJ.
9. Documentar la sensibilidad al acercarse
   a la singularidad.

No invertir el Jacobiano cuando se
clasifique como singular.

La pérdida de rango y su magnitud deberán
verificarse mediante la ejecución real.

Estado: EJECUTADO OFFLINE EN HITO 3.

## 8. Métricas de error

### Posición

ep = ||p_FK - p_ref||_2

Unidad: m.

### Orientación

eR = acos((tr(R_ref^T R_FK) - 1) / 2)

Unidad: rad.

Se limitará numéricamente el argumento
de acos al intervalo [-1, 1].

### Jacobiano

eJ = ||J_modelo_normalizado - J_FD_normalizado||_F
     / max(1, ||J_FD_normalizado||_F)

J_FD: referencia por diferencias finitas de FK.

Longitud de normalización: 1 m.

Los umbrales numéricos de aceptación
se establecerán tras revisar las pruebas
existentes y ejecutar las verificaciones offline.

No se atribuirán tolerancias inventadas al ABB.

## 9. Criterios de evaluación

Para VAL-01 y VAL-02:

- Configuración dentro de los límites académicos.
- Jacobiano regular según criterio declarado.
- Soluciones IK reconstruyen la pose objetivo
  dentro de tolerancias justificadas.
- ep, eR y eJ registrados.

Para VAL-03:

- Evidencia del comportamiento del rango
  y valores singulares.
- Comparación con la postura q5 = 0.
- eJ documentado.
- Ausencia de inversión del Jacobiano singular.

Todos los resultados deberán indicar el
marco objetivo y la configuración usada.

## 10. Distinción de torque y dinámica

La expresión

tau_ext = J^T F

representa los torques articulares generalizados
asociados a un wrench externo F, según el
Jacobiano y convenciones utilizados.

No representa por sí sola toda la dinámica.

Un modelo dinámico más completo contempla
términos de inercia, Coriolis/centrífugos,
gravedad y, cuando corresponda, fricción.

La implementación reducida 1R del Hito 2
no equivale a una dinámica completa del ABB 6R.

## 11. Evidencia futura

Para cada caso:

- Configuración articular y unidades.
- Marcos utilizados.
- Estado de límites.
- Pose objetivo y pose reconstruida.
- ep y eR.
- Jacobianos comparados.
- eJ.
- Rango y valores singulares cuando aplique.
- Comando de ejecución y versiones.
- Logs y resultados reproducibles.
- Interpretación y limitaciones.

## 12. Estado de ejecución

VAL-01: EJECUTADO OFFLINE.
VAL-02: EJECUTADO OFFLINE.
VAL-03: EJECUTADO OFFLINE.

Se ejecutaron validaciones numéricas offline.
No se realizaron ensayos físicos.

Siguiente acción:
revisar los resultados y formalizar criterios
de aceptación antes de Gate 2.

## 13. Resultados offline observados

Fecha de ejecución: 2026-10-27.

Entorno:
- Python 3.14.8.
- NumPy 2.5.2.
- Entorno virtual académico IMT-342.
- Cálculos sin conexión al robot físico.

Evidencias:
- validacion/evaluar_cinematica.py
- evidencias/validacion_cinematica.json

### VAL-01

- Límites: cumplidos.
- Rango del Jacobiano: 6.
- Número de condición: 74.73577693567445.
- Error relativo eJ: 5.211494677201453e-11.
- Soluciones IK: 8.
- Soluciones admisibles: 2.
- ep máximo: 3.7263863056628965e-16 m.
- eR máximo: 5.204376831846875e-16 rad.

### VAL-02

- Límites: cumplidos.
- Rango del Jacobiano: 6.
- Número de condición: 133.2892475738682.
- Error relativo eJ: 5.121275377282839e-11.
- Soluciones IK: 8.
- Soluciones admisibles: 3.
- ep máximo: 1.2412670766236366e-16 m.
- eR máximo: 5.299864942873025e-16 rad.

### VAL-03

- Límites: cumplidos.
- q5: 0.01 rad.
- Rango del Jacobiano: 6.
- Número de condición: 1327.8915162855872.
- Error relativo eJ: 6.163484612660818e-11.

### Referencia singular

- Límites: cumplidos.
- q5: 0 rad.
- Rango del Jacobiano: 5.
- Valor singular mínimo: 2.9935231113068693e-17.
- Número de condición: no definido como valor
  finito bajo el criterio numérico adoptado.
- Error relativo eJ: 4.451617452227668e-11.

### Interpretación

Las soluciones FK/IK presentan errores pequeños.

El Jacobiano analítico concuerda con la
aproximación por diferencias finitas.

La aproximación a q5 = 0 incrementa
la sensibilidad cinemática y la referencia
singular presenta pérdida de rango.

Estas conclusiones corresponden al modelo
académico implementado y no al controlador
físico ABB.

Los umbrales formales de aceptación de Gate 2
continúan pendientes de establecerse.
