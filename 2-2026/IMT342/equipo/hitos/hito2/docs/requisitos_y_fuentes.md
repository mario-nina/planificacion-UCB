
# Hito 2 — Cinemática inversa, LSPB y análisis del ABB IRB 120

Proyecto Final Integrador de **IMT-342 Robótica** — Universidad Católica Boliviana.

## Objetivo

Implementar y validar un procedimiento computacional para el manipulador **ABB IRB 120**, integrando cinemática inversa analítica, planificación de trayectorias, análisis diferencial, singularidades y dinámica reducida.

El desarrollo utiliza el modelo Denavit–Hartenberg validado en el Hito 1 y se presenta mediante un Notebook Jupyter reproducible.

## Componentes implementados

1. Cinemática inversa analítica mediante el método de Pieper.
2. Verificación de soluciones mediante cinemática directa.
3. Filtrado por límites articulares ABB y selección de configuraciones.
4. Planificación articular LSPB.
5. Jacobiano geométrico para brida y TCP.
6. Análisis de singularidad de muñeca.
7. Evaluación de torque mediante un modelo académico reducido 1R.
8. Integración, gráficas y validaciones en Jupyter Notebook.

## Modelo de referencia

Se utiliza el ABB IRB 120 de seis articulaciones revolutas, con convención D-H estándar y las unidades del Sistema Internacional.

Se distinguen dos puntos de interés:

- **Brida:** marco terminal D-H `{6}`.
- **TCP académico:** situado 0.120 m sobre el eje local `z6` de la brida, sin rotación adicional.

El Notebook admite poses objetivo definidas respecto a cualquiera de los dos marcos.

## Organización del proyecto

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
│   ├── limites_irb120.py
│   ├── seleccion_pieper.py
│   ├── lspb.py
│   ├── jacobiano.py
│   ├── singularidades.py
│   └── dinamica_1r.py
├── notebooks/
│   └── hito2_integracion.ipynb
├── validacion/
├── evidencias/
└── informe/
    ├── main.tex
    ├── informe_hito2.pdf
    └── figuras/
```

Los módulos de `src/` concentran las implementaciones matemáticas. El Notebook organiza los ensayos, utiliza las funciones existentes y presenta los resultados.

## Ejecución

Desde la carpeta `hito2/`, con el entorno Python de IMT-342 disponible:

```bash
python -m jupyterlab
```

Abrir:

`notebooks/hito2_integracion.ipynb`

Seleccionar el kernel **Python (IMT-342)** y ejecutar las celdas en orden.

### Parámetros del ensayo

La celda de datos permite configurar:

| Parámetro | Función |
|---|---|
| `MODO_POSE` | Pose generada por FK o matriz manual |
| `MARCO_OBJETIVO` | Selección entre `flange` y `tcp` |
| `T_MANUAL` | Matriz homogénea objetivo 4×4 |
| `q_referencia` | Configuración del ensayo de autocontrol FK |
| `q_inicio` | Configuración inicial de la trayectoria |
| `TF_6R` | Duración total LSPB |
| `ACEL_6R` | Aceleraciones de planificación |

La pose debe ser una transformación homogénea válida y encontrarse dentro del espacio alcanzable para obtener una solución articular admisible.

## Validación

La batería de pruebas automatizadas se ejecuta desde `hito2/`:

```bash
python -m unittest discover -s validacion -p "test_*.py" -v
```

En la validación integral del 22 de octubre de 2026 se obtuvieron **71 pruebas satisfactorias y ninguna fallida**.

El Notebook de integración también fue ejecutado mediante el kernel Python de IMT-342. Los cambios posteriores de presentación deben verificarse mediante una nueva ejecución completa antes de la entrega.

### Resultados del caso de referencia

Para la configuración de autocontrol:

$$
\mathbf q_{\mathrm{ref}}=
[0.30,-0.20,0.10,0.20,-0.40,0.50]^T
\ \mathrm{rad}
$$

se obtuvieron:

- Ocho ramas matemáticas regulares de Pieper.
- Dos ramas con representaciones dentro de los límites nominales ABB.
- Reconstrucción por FK con errores posicionales del orden de $10^{-16}$ m.
- Trayectoria LSPB de 8 s, con reposo inicial y final.
- Reducción de rango del Jacobiano de 6 a 5 en el experimento de muñeca con $q_5=0$.
- Torque de 4.429791 N·m en el ejemplo académico 1R para $t=2.5$ s.

Los resultados corresponden a un caso reproducible del equipo. Las poses solicitadas durante la evaluación pueden sustituirse mediante la entrada manual del Notebook.

## Informe técnico

El informe se encuentra en:

- `informe/main.tex`: fuente LaTeX en formato IEEE.
- `informe/informe_hito2.pdf`: documento compilado.
- `informe/figuras/`: gráficas utilizadas en el informe.

La edición del informe se realiza en Overleaf.

## Alcance

La implementación verifica modelos cinemáticos y un modelo dinámico reducido bajo las hipótesis declaradas. No constituye una certificación de seguridad, una comprobación de colisiones ni una estimación de torques reales de los seis actuadores ABB.

Los requisitos docentes y la correspondencia de cada evidencia se detallan en `docs/requisitos_y_fuentes.md`.
