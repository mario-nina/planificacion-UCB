# Fuentes externas

## ABB IRB 120

Para obtener las cotas geométricas utilizadas en la construcción de la
tabla D-H se consultó la documentación oficial del fabricante:

- ABB Robotics
- *Especificaciones del producto - IRB 120*
- Documento `3HAC035960-005`
- Revisión W
- Página 15: "Dimensiones IRB 120-3/0.6"

Cotas utilizadas:

- 290 mm
- 270 mm
- 70 mm
- 302 mm
- 72 mm

El documento no se incluye en este repositorio para evitar redistribuir
documentación del fabricante cuyo permiso de redistribución no ha sido
verificado.

Las cotas anteriores se utilizaron como datos geométricos del
manipulador. La parametrización D-H fue desarrollada a partir del material
y las convenciones trabajadas en la asignatura.

## Datos utilizados en el modelo URDF

De la misma especificación oficial de ABB se utilizaron además:

- Página 37: rangos de movimiento de los ejes 1 a 6.
- Página 40: velocidades máximas de los ejes 1 a 6.

Los rangos angulares y las velocidades fueron convertidos de grados y
grados por segundo a radianes y radianes por segundo, respectivamente.

No se encontró en la documentación oficial consultada un valor de torque
máximo por articulación adecuado para el atributo `effort` de URDF.
Por este motivo, estos valores se mantienen provisionalmente en `0` como
placeholders para el modelo cinemático.

## Mallas 3D del ABB IRB 120

Para la representación visual del manipulador se utilizaron las mallas
STL del modelo IRB 120 3/0.58 disponibles en el paquete
`abb_irb120_support` del proyecto ROS-Industrial.

Fuente:

- Proyecto: ROS-Industrial
- Repositorio: `ros-industrial/abb`
- Paquete: `abb_irb120_support`
- Modelo: `irb120_3_58`

Se utilizaron las mallas correspondientes a:

- `base_link`
- `link_1`
- `link_2`
- `link_3`
- `link_4`
- `link_5`
- `link_6`

Antes de integrarlas se inspeccionó el archivo
`irb120_3_58_macro.xacro`, verificando que las mallas se encuentran
definidas con origen local `xyz="0 0 0"` y `rpy="0 0 0"`, y que la
estructura de juntas es compatible con la cadena cinemática ya
desarrollada por el equipo.

No se importó el modelo cinemático completo de ROS-Industrial.
Las mallas se utilizaron únicamente como recurso geométrico.

## Mallas de colisión del ABB IRB 120

Del mismo modelo `irb120_3_58` del paquete
`abb_irb120_support` de ROS-Industrial se utilizaron también las
mallas STL simplificadas ubicadas en el directorio `collision`.

Se incorporaron las mallas correspondientes a:

- `base_link`
- `link_1`
- `link_2`
- `link_3`
- `link_4`
- `link_5`
- `link_6`

Estas mallas se utilizan únicamente como geometría de colisión del
URDF. Son modelos simplificados respecto de las mallas visuales y
permiten representar los volúmenes de los eslabones con menor
complejidad geométrica.

Antes de cerrar la integración se verificó en RViz que las mallas de
colisión mantienen una escala, posición y orientación coherentes, y que
acompañan correctamente el movimiento de las seis articulaciones.
