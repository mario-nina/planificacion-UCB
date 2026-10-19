# Evidencia reproducible — singularidad de muñeca (Hito 2)

Desde la raíz `hito2/`, usando el entorno Python académico con NumPy y Matplotlib:

```bash
python evidencias/generar_graficas_singularidad.py
```

El script llama a `src/singularidades.py` y genera en `evidencias/figuras/` tres imágenes PNG y un CSV con resultados numéricos. Los PNG no son imágenes dibujadas manualmente y el CSV no contiene valores introducidos a mano: ambos se calculan desde el código del Hito 2.

## Caso de estudio

- Robot y modelo: ABB IRB 120 académico, Jacobiano geométrico de **brida** en marco base `{0}`.
- Configuración de referencia (radianes): `q = [0.30, -0.35, 0.25, 0.40, 0.0, -0.60]`.
- Juntas J1–J4, J6 constantes; J5 desciende según `q5 = [0.2, 0.1, 0.05, 0.01, 0.001, 0.0001, 0.0]` rad.
- Normalización numérica: `Jn = diag(1/L, 1/L, 1/L, 1, 1, 1) J`, con `L = 1 m`.
- Tolerancia de rango: `1e-9` relativa al máximo valor singular.
- Consigna cartesiana fija: dirección izquierda nula del Jacobiano singular, con norma `0.01` en coordenadas normalizadas; para la componente lineal se restablece `L` antes de calcular `qdot`.
- En `q5=0` el rango es 5. La norma de `qdot` no está definida porque la consigna elegida no es realizable exactamente en esa postura; **no colocar un valor artificial en el gráfico**.

El número de condición, la dirección singular y la magnitud de `qdot` dependen de la normalización y de la consigna elegida. La pérdida de rango es el criterio central exigido por la guía del docente.

Estos gráficos son evidencia de diagnóstico numérico, **no** una trayectoria ejecutable ni una simulación dinámica. No se ha verificado el cumplimiento de límites de velocidad reales de ABB en las velocidades amplificadas. Una singularidad cinemática no equivale a torque infinito.

## Fuente docente

`S10_material_oficial.pdf`, documento `hito_2_guia.pdf`, p. 31 del consolidado, y `hito_2_rubrica.pdf`, p. 35: experimento independiente de singularidad, rango matricial, pérdida de movilidad, amplificación de velocidades e interpretación.
