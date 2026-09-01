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
