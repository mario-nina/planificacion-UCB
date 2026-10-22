
# Arquitectura y convenciones — Hito 2

## 1. Descripción general

El Hito 2 implementa una cadena de cálculo para el ABB IRB 120 que integra cinemática inversa, selección articular, planificación LSPB, cinemática diferencial y evaluación dinámica reducida.

Los cálculos reutilizables se encuentran en `src/`. El Notebook realiza la integración, visualización y comprobación de resultados.

La arquitectura es independiente de la implementación histórica del Hito 1 durante la ejecución.

## 2. Base cinemática

El manipulador posee seis articulaciones revolutas:

$$
\mathbf q=
[q_1,q_2,q_3,q_4,q_5,q_6]^T
$$

Se emplean:

- Convención Denavit–Hartenberg estándar.
- Ángulos articulares en radianes.
- Longitudes y posiciones en metros.
- Velocidades articulares en rad/s.
- Aceleraciones articulares en rad/s².

### Parámetros D-H

| Junta | $\theta_i$ | $d_i$ (m) | $a_i$ (m) | $\alpha_i$ |
|---|---|---:|---:|---|
| J1 | $q_1$ | 0.290 | 0 | $-\pi/2$ |
| J2 | $q_2-\pi/2$ | 0 | 0.270 | $0$ |
| J3 | $q_3$ | 0 | 0.070 | $-\pi/2$ |
| J4 | $q_4$ | 0.302 | 0 | $\pi/2$ |
| J5 | $q_5$ | 0 | 0 | $-\pi/2$ |
| J6 | $q_6$ | 0.072 | 0 | $0$ |

El desplazamiento angular de J2 forma parte de la convención del modelo:

$$
\theta_2=q_2-\frac{\pi}{2}
$$

Cada transformación elemental se obtiene mediante:

$$
{}^{i-1}T_i=
R_z(\theta_i)\,
T_z(d_i)\,
T_x(a_i)\,
R_x(\alpha_i)
$$

La cinemática directa se construye como:

$$
{}^0T_6=
{}^0T_1\,{}^1T_2\,
{}^2T_3\,{}^3T_4\,
{}^4T_5\,{}^5T_6
$$

## 3. Brida y TCP

El marco `{6}` representa la brida terminal del modelo D-H.

El TCP académico tiene una traslación fija de 0.120 m sobre el eje local $z_6$:

$$
{}^6T_{\mathrm{TCP}}=
\begin{bmatrix}
1&0&0&0\\
0&1&0&0\\
0&0&1&0.120\\
0&0&0&1
\end{bmatrix}
$$

La transformación del TCP respecto a la base es:

$$
{}^0T_{\mathrm{TCP}}=
{}^0T_6\,{}^6T_{\mathrm{TCP}}
$$

Cuando se recibe una pose objetivo del TCP, la pose equivalente de la brida es:

$$
{}^0T_6=
{}^0T_{\mathrm{TCP}}
\left({}^6T_{\mathrm{TCP}}\right)^{-1}
$$

La distancia brida–TCP de 0.120 m no debe confundirse con el parámetro D-H $d_6=0.072$ m.

Ambos marcos comparten orientación, pero sus posiciones y velocidades lineales pueden ser distintas.

## 4. Organización de los módulos

| Archivo | Responsabilidad |
|---|---|
| `src/dh.py` | Transformaciones D-H y encadenamiento |
| `src/irb120_fk.py` | Geometría del IRB 120, FK y TCP |
| `src/pieper.py` | Cinemática inversa analítica |
| `src/limites_irb120.py` | Límites articulares y equivalencias de J6 |
| `src/seleccion_pieper.py` | Filtrado y selección de soluciones |
| `src/lspb.py` | Planificación y evaluación temporal |
| `src/jacobiano.py` | Jacobiano geométrico y velocidades cartesianas |
| `src/singularidades.py` | Diagnóstico y experimento de singularidad |
| `src/dinamica_1r.py` | Torque del modelo reducido 1R |

Los módulos `dh.py` e `irb120_fk.py` provienen del modelo cinemático desarrollado en el Hito 1, con las adaptaciones de organización necesarias para integrarlos como paquete Python.

La implementación histórica del Hito 1 permanece separada.

## 5. Cinemática inversa analítica

El método de Pieper utiliza el desacoplamiento entre la posición del centro de muñeca y la orientación final.

Para una pose objetivo de brida:

$$
\mathbf p_{wc}
=\mathbf p_d-d_6R_d
\begin{bmatrix}0\\0\\1\end{bmatrix}
$$

La orientación restante se obtiene mediante:

$$
R_3^6=(R_0^3)^TR_d
$$

El algoritmo identifica las combinaciones de brazo y muñeca y valida cada candidata mediante cinemática directa.

Las soluciones matemáticas no necesariamente cumplen los límites articulares nominales.

### Selección articular

Entre las representaciones admisibles se minimiza:

$$
C(\mathbf q)=
\sum_{i=1}^{6}
w_i(q_i-q_{\mathrm{inicio},i})^2
$$

Los pesos predeterminados son unitarios.

Para J6 se consideran representaciones equivalentes:

$$
q_6'=q_6+2k\pi,\qquad k\in\mathbb Z
$$

Siempre deben cumplir los límites nominales ABB.

En el caso de autocontrol se obtuvieron ocho ramas matemáticas y dos ramas con representaciones admisibles.

## 6. Límites nominales ABB

Los rangos considerados corresponden a la documentación oficial ABB del IRB 120.

| Articulación | Mínimo | Máximo | Velocidad nominal máxima |
|---|---:|---:|---:|
| J1 | −165° | +165° | 250°/s |
| J2 | −110° | +110° | 250°/s |
| J3 | −110° | +70° | 250°/s |
| J4 | −160° | +160° | 320°/s |
| J5 | −120° | +120° | 320°/s |
| J6 | −400° | +400° | 420°/s |

**Fuentes:** ABB `3HAC035960-005`, secciones 1.8.1 y 1.8.3; ABB `3HAC035728-005`, sección 2.2.2.

Los límites se convierten a radianes para su utilización computacional.

Las aceleraciones utilizadas en LSPB son parámetros de planificación académica. No se presentan como máximos certificados por ABB.

El cumplimiento de límites nominales no sustituye una comprobación de colisiones ni las restricciones de un controlador real.

## 7. Planificación LSPB

El módulo `src/lspb.py` admite perfiles de una o seis articulaciones.

Las entradas principales son:

- Posición articular inicial.
- Posición articular final.
- Duración total $t_f$.
- Magnitudes de aceleración.
- Límites opcionales adicionales de velocidad.

Para cada articulación:

$$
\Delta q_i=q_{f,i}-q_{0,i}
$$

La condición de factibilidad temporal es:

$$
a_i\geq\frac{4|\Delta q_i|}{t_f^2}
$$

El tiempo de mezcla satisface:

$$
|\Delta q_i|=
a_i t_{c,i}(t_f-t_{c,i})
$$

La velocidad de crucero es:

$$
v_{c,i}=
\operatorname{sgn}(\Delta q_i)a_i t_{c,i}
$$

La planificación entrega:

$$
q_i(t),\qquad
\dot q_i(t),\qquad
\ddot q_i(t)
$$

Las seis articulaciones comparten la duración total, aunque sus tiempos de mezcla pueden ser diferentes.

El perfil ideal mantiene continuidad de posición y velocidad, pero no necesariamente de aceleración.

## 8. Jacobiano geométrico

El Jacobiano relaciona las velocidades articulares con las velocidades del efector:

$$
{}^0\mathbf V_e=
{}^0J_e(\mathbf q)\dot{\mathbf q}
$$

Para una articulación revoluta, cada columna se construye mediante:

$$
{}^0J_i=
\begin{bmatrix}
{}^0\mathbf z_{i-1}\times
({}^0\mathbf p_e-{}^0\mathbf o_{i-1})\\
{}^0\mathbf z_{i-1}
\end{bmatrix}
$$

Las tres primeras filas del Jacobiano representan la contribución lineal y las últimas tres la angular.

La implementación admite como punto efector la brida o el TCP.

Sus velocidades se relacionan mediante:

$$
\mathbf v_{\mathrm{TCP}}=
\mathbf v_6+
\boldsymbol\omega_6\times
(\mathbf p_{\mathrm{TCP}}-\mathbf p_6)
$$

El módulo se validó mediante comprobaciones manuales y diferencias finitas de la FK.

## 9. Análisis de singularidades

El experimento de muñeca estudia una secuencia de configuraciones en la que:

$$
q_5\longrightarrow 0
$$

En la postura singular de referencia, los ejes de J4 y J6 se alinean.

El criterio principal es la pérdida de rango del Jacobiano:

$$
\operatorname{rank}(J)<6
$$

Para analizar valores singulares y sensibilidad, el Jacobiano se normaliza mediante una longitud de referencia de 1 m.

Esta longitud constituye una convención numérica y no una dimensión adicional del manipulador.

El experimento registra:

- Rango numérico.
- Menor valor singular.
- Número de condición.
- Dirección cartesiana débil.
- Modo articular nulo.
- Norma de velocidad articular requerida por una consigna fija.

En la configuración singular estudiada, el rango pasa de seis a cinco.

No se invierte el Jacobiano cuando se clasifica como singular.

## 10. Dinámica reducida 1R

La evaluación dinámica emplea el modelo académico del docente:

$$
J_{1R}=I_c+m\ell_c^2
$$

$$
\tau(t)=J_{1R}\ddot q(t)+mg\ell_c\sin q(t)
$$

Las contribuciones se calculan separadamente:

$$
\tau_I(t)=J_{1R}\ddot q(t)
$$

$$
\tau_g(t)=mg\ell_c\sin q(t)
$$

El modelo puede recibir una trayectoria LSPB 1R o la señal de una articulación de una trayectoria 6R, sin interpretar el resultado como torque físico de ese actuador.

El ejemplo oficial produce:

$$
\tau(2.5\,\mathrm s)\approx4.429791\ \mathrm{N\,m}
$$

La dinámica reducida no representa los acoplamientos del ABB IRB 120 completo.

## 11. Integración en el Notebook

El archivo `notebooks/hito2_integracion.ipynb` utiliza las funciones de `src/` y presenta las evidencias del Hito 2.

La celda de parámetros permite modificar el marco objetivo, la matriz de pose, la configuración inicial y los parámetros LSPB.

No es necesario modificar las implementaciones de los módulos para evaluar una nueva pose objetivo válida.

Se utiliza el kernel Python de IMT-342.

## 12. Validación

El conjunto de pruebas está organizado en `validacion/`.

Desde `hito2/`:

```bash
python -m unittest discover -s validacion -p "test_*.py" -v
```

La ejecución integral del 22 de octubre de 2026 produjo:

**71 pruebas satisfactorias, sin fallos.**

Estas pruebas cubren FK, Pieper, límites, selección, LSPB, Jacobiano, singularidades y dinámica reducida.

La validez matemática y numérica observada no constituye una certificación de seguridad del robot real.

## 13. Fuentes

### Material docente

- `00_programa_IMT342.pdf`.
- `Guia_PFI.pdf`.
- `S09_material_oficial.pdf`.
- `S10_material_oficial.pdf`, guía, rúbrica y solucionario del Hito 2.

### Fabricante

- ABB, *Especificaciones del producto — IRB 120*, `3HAC035960-005`.
- ABB, *Manual del producto — IRB 120*, `3HAC035728-005`.

### Implementación

- `src/`
- `validacion/`
- `notebooks/hito2_integracion.ipynb`
- `informe/main.tex`
