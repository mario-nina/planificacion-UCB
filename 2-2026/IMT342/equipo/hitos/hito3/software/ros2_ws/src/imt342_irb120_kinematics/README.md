# Cinemática IRB 120

La cadena activa en `imt342_irb120_description/urdf/irb120.urdf` define
`base_link` → `tcp`. El orden es `joint_1` a `joint_6`, en radianes y con
los signos de los ejes URDF Z, Y, Y, X, Y, X. Se conservan las tres
transformaciones fijas tras J6. El marco D-H resulta de una rotación fija
(RPY de 0, −π/2, π) y el TCP está a 0.120 m en su Z según el modelo.
Las dimensiones nominales y el TCP del modelo requieren cotejo con el
controlador para una comparación física Gate 2.

## Desacoplamiento Pieper

Se elimina la transformación fija `link_6 → tcp` por la derecha. Si
`T_06 = [R_06, p_06]`, el centro de muñeca es
`p_W = p_06 − 0.072 R_06 e_x`. J4, J5 y J6 se cruzan allí porque J4
y J6 son ejes X y sus traslaciones hacia y desde J5 son colineales.

Restados los 0.290 m de altura del hombro, el par radial/vertical se
resuelve con dos brazos efectivos. El primero mide 0.270 m. El
segundo mide `hypot(0.302, 0.070)` m y su ángulo respecto a Z es
`atan2(0.302, 0.070)`. La rama de hombro elige azimut y radio positivo,
o azimut más π y radio negativo. La ley del coseno da las dos ramas
de codo. Con q1–q3 calculados, se descompone
`(Rz(q1) Ry(q2+q3))ᵀ R_06` en X–Y–X. Los signos de sin(q5)
distinguen las dos ramas de muñeca. Los cuadrantes se resuelven con
`atan2`.

Los límites URDF recortan las ocho ramas geométricas. J6 puede admitir
más de un giro para una misma rama, lo que produce consignas articulares
distintas. Cada candidata conserva un identificador de rama y variante,
y se comprueba por FK con tolerancias internas de 1 µm y 1 µrad. A q5=0
se preserva q4+q6 y se elige una representante por suma equivalente
compatible con los límites. Cuando el centro de muñeca cae en el eje
de base, se prueban las representantes de q1 dada por la semilla y cero.
Ese caso degenerado admite una familia continua y la búsqueda discreta
puede omitir soluciones válidas. Se conserva la API numérica local
`Model.ik` como método separado.

## Convenciones y verificación

- `pose_from_rpy` y `pose_to_rpy` usan XYZ en metros y RPY extrínseco
  en radianes, con `R = Rz(yaw) Ry(pitch) Rx(roll)`. En bloqueo de
  cardán la representación canónica usa roll cero.
- El cuaternión usa orden XYZW. La matriz debe ser homogénea con
  rotación ortonormal propia.
- Las seis filas del Jacobiano espacial son velocidad lineal y angular
  en base. `jacobian_fd` usa diferencias centradas y el logaritmo
  espacial de la rotación relativa. Para SVD se escala `Jv/L` con
  `L = 0.5 m` por defecto.
- `SingleLinkDynamics` calcula contribuciones inercial y gravitatoria
  para una sola junta, con cero de q en equilibrio colgante y
  parámetros SI explícitos.
- Los tests incluyen comparación URDF frente a la tabla D-H histórica,
  ocho ramas geométricas en un caso sintético, barrido a q5=0 y casos de
  alcance y convención. La validación física requiere una referencia
  independiente con marcos y TCP comparables.
