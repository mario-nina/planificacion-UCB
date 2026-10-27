# Gate 0 — Bitácora técnica y seguimiento

Proyecto: IMT-342, Hito 3 progresivo ECI-3/4
Robot: ABB IRB 120
Fecha de registro: 2026-10-27

Estado general: PARCIALMENTE VERIFICADO

## 1. Fuente académica

S12_material_oficial.pdf
Documento original:
w12_s01_guia_autonoma_bitacora.pdf

El docente solicita registrar:
hipótesis, prueba, resultado observado,
interpretación, evidencia y siguiente acción.

Se incluyen responsables y bloqueos.

## 2. Criterios de clasificación

- functional: funcional dentro del alcance ensayado.
- partially_verified: existe evidencia parcial.
- blocked: falta una dependencia crítica.
- not verified: no existe evidencia suficiente.

Una verificación offline no certifica
la integración con el robot físico.

## 3. Registro B00-01 — Modelo URDF

Fecha de registro: 2026-10-27.
Hora original: no registrada.
Responsables: pendientes de asignación.
Modo: auditoría documental.

Hipótesis:
El proyecto dispone de un modelo URDF propio
con seis juntas revolutas y un TCP académico.

Objetivo:
Identificar los marcos y transformaciones
declarados en el modelo.

Software/hardware:
Repositorio del curso.
Robot físico no inspeccionado.

Condiciones:
URDF del Hito 1.
Marcos base_link y tcp.

Prueba ejecutada:
Inspección documental del modelo y sus
transformaciones terminales.

Resultado observado:
Se identificaron seis juntas revolutas,
transformaciones fijas y TCP provisional
con desplazamiento de 0.120 m.

Evidencia:
- gate0/marcos_y_tcp.md
- Fuente URDF: commit ae2cf35
- Registro Hito 3: commit 6edd1de

Interpretación:
Existe un modelo académico documentado.

Estado funcional: partially_verified.

Limitaciones:
No se ha confirmado el TCP físico
ni su correspondencia con el controlador ABB.

Siguiente acción:
Comparar marcos y configuraciones equivalentes
cuando exista evidencia del laboratorio.

Revisión docente/laboratorio: requerida.

## 4. Registro B00-02 — Entorno ROS local

Fecha de registro: 2026-10-27.
Hora original: no registrada.
Responsables: pendientes de asignación.
Modo: auditoría local.

Hipótesis:
Existe un entorno ROS 2 Jazzy reutilizable
para el desarrollo de IMT-342.

Objetivo:
Verificar sistema, contenedor y paquetes básicos.

Software/hardware:
- Fedora Workstation 44.
- Podman 5.8.9 rootless.
- Ubuntu 24.04.4 LTS en contenedor.
- ROS 2 Jazzy.

Prueba ejecutada:
Consulta del sistema anfitrión y ejecución
de un contenedor temporal sin red.

Resultado observado:
- ROS_DISTRO = jazzy.
- rviz2 localizado.
- robot_state_publisher localizado.
- rclpy localizado e importable.
- Sin contenedores persistentes registrados.

Evidencia:
- gate0/entorno_ros.md
- Commit f92165f

Interpretación:
Los componentes básicos existen
en el entorno local comprobado.

Estado funcional: partially_verified.

Limitaciones:
No se probó integración ROS–ABB.
No se verificó la PC del laboratorio.
No se comprobó RViz2 gráficamente
en esta auditoría.

Siguiente acción:
Investigar compatibilidad de interfaces
y drivers con el controlador real.

Revisión docente/laboratorio: requerida.

## 5. Registro B00-03 — Compatibilidad ROS–ABB

Fecha de registro: 2026-10-27.
Hora original: no registrada.
Responsables: pendientes de asignación.
Modo: investigación documental.

Hipótesis:
RWS o RWS/EGM podrían proporcionar
una interfaz de integración con ROS 2.

Objetivo:
Identificar alternativas y dependencias.

Software/hardware:
Documentación de interfaces ABB
y repositorios de drivers candidatos.

Prueba ejecutada:
Consulta documental y git ls-remote
de repositorios externos.

Resultado observado:
- abb_ros2, rama rolling:
  e6486f7409073cdb2a10d6878bb50191dd50172b
- abb_robot_driver, rama master:
  d1678957d3792e6aa54582ff894db62b6a51dac0

Se documentaron cinco hipótesis
y dos arquitecturas candidatas.

Evidencia:
- gate0/compatibilidad_ros_abb.md
- gate0/arquitecturas.md
- Commits bf7fd6a y 4b61466

Interpretación:
Existen alternativas técnicas,
pero ninguna está validada para
el controlador del laboratorio.

Estado funcional: not verified.

Limitaciones:
- Controlador desconocido.
- RobotWare sin verificar.
- Opciones RWS/EGM sin verificar.
- Compatibilidad Jazzy sin demostrar.
- Sin conexión al robot.

Siguiente acción:
Auditar el controlador y determinar
qué interfaces están disponibles.

Revisión docente/laboratorio: requerida.

## 6. Registro B00-04 — Validación cinemática

Fecha: 2026-10-27.
Hora exacta: no registrada.
Responsables: pendientes de asignación.
Modo: simulación/cálculo offline.

Hipótesis:
El modelo FK/IK/J presenta consistencia
numérica en configuraciones de referencia.

Objetivo:
Evaluar dos posturas regulares y
una aproximación a singularidad.

Software:
- Python 3.14.8.
- NumPy 2.5.2.
- Entorno virtual IMT-342.
- Implementaciones del Hito 2.

Condiciones:
- Ángulos en radianes.
- Distancias en metros.
- Marco objetivo: TCP académico.
- Sin conexión al controlador.

Pruebas ejecutadas:
- 36 pruebas automatizadas del Hito 2.
- VAL-01.
- VAL-02.
- VAL-03.
- Referencia singular q5 = 0.

Resultados observados:
- Pruebas automatizadas: 36/36 OK.
- VAL-01: rango J = 6.
- VAL-02: rango J = 6.
- VAL-03: rango J = 6.
- Referencia singular: rango J = 5.
- Errores de reconstrucción FK/IK pequeños.
- Errores relativos del Jacobiano
  del orden de 1e-11.

Evidencia:
- validacion/evaluar_cinematica.py
- evidencias/validacion_cinematica.json
- gate0/plan_validacion_cinematica.md
- Commit f0ac203

Interpretación:
Se comprobó consistencia numérica
interna del modelo académico.

Estado funcional: functional
exclusivamente para las pruebas offline.

Limitaciones:
No constituye validación geométrica
independiente con el robot físico.
Los criterios formales de Gate 2
continúan pendientes de establecerse.

Siguiente acción:
Formalizar tolerancias, correspondencia
de marcos y criterios de Gate 2.

Revisión docente/laboratorio: pendiente.

## 7. Registro B00-05 — Diseño Gate 1

Fecha de registro: 2026-10-27.
Responsables: pendientes de asignación.
Modo: diseño documental.

Hipótesis:
Será posible definir consultas de lectura
seguras una vez auditado el controlador.

Objetivo:
Preparar dos pruebas sin movimiento.

Prueba ejecutada:
Revisión y elaboración documental.
No hubo comunicación con el ABB.

Resultado observado:
Se diseñaron G1-01 y G1-02,
incluyendo condiciones previas,
evidencias y criterios de éxito.

Evidencia:
- gate0/plan_pruebas_gate1.md
- Commit e5ff57e

Interpretación:
El protocolo preliminar está documentado,
pero su ejecución sigue bloqueada.

Estado funcional: blocked.

Limitaciones:
Faltan controlador, interfaz, permisos,
responsables y autorización institucional.

Siguiente acción:
Solicitar revisión y autorización del
laboratorio para pruebas de solo lectura.

Revisión docente/laboratorio: obligatoria.

## 8. Matriz de responsabilidades

| Actividad | Responsable principal | Respaldo | Estado |
|---|---|---|---|
| Controlador, RobotWare y opciones | Por asignar | Por asignar | Pendiente |
| Código histórico de ajedrez | Por asignar | Por asignar | Pendiente |
| ROS y compatibilidad de drivers | Por asignar | Por asignar | Investigación documental |
| URDF y modelos FK/IK/J | Por asignar | Por asignar | Validación offline ejecutada |
| Seguridad y autorización | Por asignar | Por asignar | Pendiente laboratorio |
| Bitácora, Git y evidencias | Por asignar | Por asignar | En desarrollo |

La asignación deberá ser acordada por el equipo.
No se presume autorización por pertenecer al equipo.

## 9. Bloqueos actuales

| ID | Bloqueo | Consecuencia |
|---|---|---|
| BL-01 | Controlador exacto desconocido | Compatibilidad no verificable |
| BL-02 | RobotWare desconocido | RWS/EGM sin confirmar |
| BL-03 | Opciones y licencias sin auditar | EGM no demostrable |
| BL-04 | Acceso y permisos sin confirmar | Gate 1 no ejecutable |
| BL-05 | Driver Jazzy/IRB 120 sin validar | Integración ROS bloqueada |
| BL-06 | TCP físico desconocido | Comparación física incompleta |
| BL-07 | Código de ajedrez no recuperado | Reutilización no evaluada |
| BL-08 | Responsables de laboratorio no definidos | Sin autorización operativa |

Un elemento no verificado no se considera ausente.

## 10. Backlog verificable

### T01 — Identificar sistema ABB

Responsable: por asignar.
Respaldo: por asignar.

Salida esperada:
Modelo de controlador, RobotWare y
opciones documentadas con fuente verificable.

Dependencia:
Acceso autorizado a información del laboratorio.

### T02 — Auditar compatibilidad del driver

Responsable: por asignar.
Respaldo: por asignar.

Salida esperada:
Matriz de versiones y compatibilidad
para Jazzy, IRB 120 e interfaz ABB.

Dependencia:
Especificaciones del controlador y
revisión técnica del software candidato.

### T03 — Formalizar validación Gate 2

Responsable: por asignar.
Respaldo: por asignar.

Salida esperada:
Criterios cuantitativos y tolerancias
de FK/IK/J, marcos y TCP documentados.

Dependencia:
Resultados offline y convenciones
cinemáticas confirmadas.

## 11. Estado de compuertas

Gate 0:
Documentación y auditoría parcialmente completadas.
Bloqueos externos pendientes.

Gate 1:
Pruebas diseñadas, no ejecutadas.
Sin autorización de conexión.

Gate 2:
Validación interna offline inicial ejecutada.
Criterios finales y comparación física pendientes.

Gate 3:
NO-GO para movimiento físico.

Gate 4:
No iniciado.

## 12. Próxima revisión

- Confirmar responsables principales y suplentes.
- Revisar inventario de 15 elementos.
- Solicitar información institucional pendiente.
- Revisar criterios de aceptación de Gate 2.
- Mantener separado el trabajo offline
  de cualquier autorización física.
