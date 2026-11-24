# Validación y alcance académico

La evaluación se organiza según los Gates 0–4 de W12-S01 y los criterios P1–P7 de W14-S02. Los umbrales sujetos a aprobación docente no se presentan como conformidad ya aprobada.

| Gate | Alcance documentado |
| --- | --- |
| 0 | Factibilidad, inventario y arquitectura inicial en `../../gate0/` |
| 1 | Antecedentes de observación RWS/ROS en laboratorio; la trazabilidad depende de los registros originales |
| 2 | Consistencia matemática y pruebas offline de FK, IK, Jacobiano y singularidades; contraste independiente y calibración física no acreditados por esas pruebas |
| 3 | Antecedentes físicos del backend Normal (21/11) y v2 (23/11), correspondientes a revisiones previas al cierre del 24/11 |
| 4 | Planificación y simulación de tareas repetibles; los ensayos previos no acreditan ejecución física de las tareas finales de muro y triángulo |

## Criterios P1–P7

P1: comparación FK con marco, punto, unidades y referencia comunes. P2: ramas IK y residuos FK. P3: Jacobiano y diferencias finitas. P4: singularidades y subespacio de tarea. P5: decisión por criterio y alcance. P6: checklist de procedencia, convenciones, incertidumbre y reproducción. P7: revisión y firma académica. La evidencia interna no sustituye una referencia independiente ni la aprobación docente.

## Interpretación de datos

Las fuentes simuladas, planificadas, medidas y de referencia se distinguen explícitamente. Las derivadas de muestras de posición son estimaciones, no mediciones directas de servo. La dinámica 1R no es un modelo dinámico identificado de los seis ejes. Los ensayos de versiones anteriores conservan su fecha y procedencia. Las capturas seleccionadas corresponden a su ensayo original, no necesariamente al árbol final acumulativo.

## Referencias

- IMT-342, programa oficial 2-2026; W10, Unidad II y Hito 2; W12-S01, lanzamiento ECI-3/4 y protocolo Gates; W14-S02, criterios unificados de validación cinemática.
- ABB, *Manual del producto IRB 120*, 3HAC035728-005, rev. W; *Especificaciones del producto IRB 120*, 3HAC035960-005, rev. W.
- Bahani et al., *The Inverse Kinematics Evaluation of 6-DOF Robots in Cooperative Tasks Using Virtual Modeling Design and Artificial Intelligence Tools*, 2023, DOI: 10.18178/ijmerr.12.2.121-130.
- Truc y Lam, *Quasi-physical modeling of robot IRB 120 using Simscape Multibody for dynamic and control simulation*, 2020, DOI: 10.3906/elk-1909-131.
- ROS-Industrial: licencia de mallas conservada en el paquete de descripción.
