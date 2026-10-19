# Evidencia reproducible de dinámica reducida 1R

El archivo `generar_graficas_dinamica_1r.py` genera los resultados del ejemplo
pedagógico oficial de **W10-S01** para el modelo dinámico 1R del Hito 2.

## Fuente y modelo

- `S10_material_oficial.pdf`: solucionario `w10_s01_practica_soluciones.pdf`, página 17 del consolidado (Tarea 4).
- `S10_material_oficial.pdf`: `hito_2_guia.pdf`, página 31 del consolidado.
- `src/lspb.py`: perfil `qi=0`, `qf=2 rad`, `tf=3 s`, magnitud de aceleración `1 rad/s²`.
- `src/dinamica_1r.py`: `J_1R=0.25 kg·m²`, `m=1 kg`, `lc=0.5 m`, `g=9.81 m/s²`.

Los parámetros **son didácticos**; no identifican masas, inercias ni torques de
los actuadores del ABB IRB 120. El ángulo 1R se mide desde la vertical inferior.

## Uso

Desde la raíz `equipo/hitos/hito2/`:

```bash
python evidencias/generar_graficas_dinamica_1r.py
```

Dependencias: NumPy y Matplotlib. Se utiliza backend gráfico `Agg`.

Se producen dentro de `evidencias/figuras/`:

- `01_perfiles_lspb_1r.png`: posición, velocidad y aceleración alineadas en tiempo.
- `02_torques_componentes_1r.png`: contribuciones inercial y gravitatoria.
- `03_torque_total_1r.png`: torque total y muestra a `t=2.5 s`.
- `dinamica_1r_datos.csv`: datos exportados para reproducción y reporte.

Las líneas verticales discontinuas indican las transiciones temporales de
LSPB (`tc` y `tf-tc`). La aceleración ideal cambia discontinuamente, por lo
que el torque inercial también puede saltar. Un crucero con aceleración nula
puede requerir torque gravitatorio no nulo.

Se valida numéricamente el ejemplo oficial a `t=2.5 s`:
`q=1.875 rad`, `qdot=0.5 rad/s`, `qddot=-1 rad/s²`,
`tau=-0.25 + 4.905 sin(1.875) ≈ 4.43 N·m`.
