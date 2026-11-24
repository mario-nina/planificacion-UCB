# Modelo cinemático, planificación y tareas

El modelo comprende seis articulaciones revolutas. La pose se expresa respecto a `base_link`; `dh_frame_6` identifica la brida y el TCP activo es configurable. La brida es la referencia predeterminada. El TCP modelado no equivale a una calibración física independiente.

La cinemática directa, la inversa analítica por Pieper y el Jacobiano geométrico se complementan con análisis de singularidades y un modelo dinámico didáctico 1R. LSPB y perfiles quínticos proporcionan referencias temporales; no representan necesariamente la interpolación interna del ABB. Las unidades del modelo son metros y radianes; la interfaz ABB usa grados cuando corresponde.

## Celda nominal

La configuración de escena y validación se comparte en `ros2_ws/src/imt342_irb120_bringup/config/cell.yaml`.

| Magnitud | Valor nominal |
| --- | --- |
| Dimensiones exteriores | 800 × 800 mm |
| Interior | 700 × 700 mm |
| Altura / espesor de muro | 200 / 50 mm |
| Traslación del marco de celda respecto a base | +128 mm en X |
| Escala de malla | 0,001 m/mm |

`geometry_verified: false` identifica una geometría aún no contrastada físicamente. Los resultados de alcance y colisión se limitan al modelo empleado.

## Tareas del catálogo

El recorrido sobre el muro considera una línea nominal de 500 mm a 300 mm de altura. La revisión continua v2 permite pasadas rectas MoveL; las modalidades segmentadas emplean MoveAbsJ. Las muestras de planificación no son necesariamente órdenes de parada.

El triángulo equilátero horizontal de 100 mm de lado se ubica nominalmente a 450 mm de altura y utiliza la brida como referencia. Su recorrido articular aproxima los lados; no se identifica como trayectoria cartesiana MoveL exacta. Se conserva el catálogo final de siete tareas y la captura de HOME desde la postura vigente del plan.
