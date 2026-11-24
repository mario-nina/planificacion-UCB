# Informe ECI-3/4 — ABB IRB 120, RAPID v2

- `informe.tex`: fuente IEEEtran, compilable en Overleaf (pdfLaTeX).
- `informe.pdf`: informe de dos páginas, compilado y revisado visualmente.
- `figuras/`: posición y velocidad, trayectoria XY, aceleración y valores singulares.
- `resultados.json`: métricas extraídas y calculadas.

Fuente experimental: `EVIDENCIA_TRIANGULO_V2.tar.gz`, ensayo físico `f71aa1fae7de446492dc247c17bf8fb0`, 24/11/2026. El archivo original de evidencia se conserva por separado. Se excluyen los registros de observación y previsualización ajenos al lote físico. La gráfica usa muestras RWS únicas y la referencia por tramo alineada al inicio de fase 2 observado. Las estimaciones de velocidad y aceleración proceden de la aplicación. Los errores cartesianos son distancias 3D de la brida obtenida por FK al segmento ideal correspondiente, exclusivamente durante el trazado de los lados. No son mediciones cartesianas independientes ni cotas del error continuo entre muestras. Las métricas de seguimiento se toman del registro final del lote, con su criterio de 239 muestras; no se confunden con las 273 muestras utilizadas en la representación temporal.
