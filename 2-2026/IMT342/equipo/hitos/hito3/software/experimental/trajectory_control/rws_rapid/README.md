# RWS/RAPID — Trayectorias v2

## Servidor v2 para RWS

`Imt342TrajectoryRapid.mod` contiene la revisión v2 recuperada. El PP debe quedar en `Imt342TrajectoryRapid/ServeTrajectoryOrders` en la tarea de movimiento `T_ROB1`. La rutina permanece en espera, acepta una sola consigna nueva por secuencia y cierra las órdenes pendientes al entrar. Después de un STOP, se debe verificar PP y realimentación antes de colocar el PP en la entrada para el siguiente ensayo. El servidor se mantiene independiente de `Imt342JointServer.mod` y no contiene `main`.

| Símbolo | Contrato |
| --- | --- |
| `trjVersion` | Constante numérica `2`. |
| `trjOrder` | `[seq,startPose,target,moveTime]`. Ambos `jointtarget` tienen orden J1–J6 y grados, con los seis ejes externos en `9E9`. Duración solicitada en segundos de 0,1 a 30. |
| `trjReply` | `[seq,phase]`. Fases `0` inicial, `1` aceptada, `2` en movimiento, `3` destino comprobado, `4` rechazada o fallo y `5` orden pendiente descartada al entrar. |

El servidor compara la postura inicial y final con la consigna usando `CJointT()` y un margen interno de 0,30° por eje. La fase `3` se publica después de `WaitRob \InPos` y de comprobar la postura. El acuse de RAPID debe cotejarse con la muestra J1–J6 recibida por RWS. Un único `MoveAbsJ` termina con parada `fine`, usando `tool0` y solicitando la duración con `\T`. En esa instrucción `\T` sustituye la velocidad de `speeddata` por una velocidad calculada en ABB. El literal `v10` no actúa como límite físico de velocidad cuando se solicita `\T`. La comparación temporal debe usar cada reloj explícitamente y distinguir el acuse, la duración solicitada y el movimiento observado. [ABB RAPID RW6, 3HAC050917](https://library.e.abb.com/public/b227fcd260204c4dbeb8a58f8002fe64/Rapid_instructions.pdf)

La revisión recuperada tiene SHA-256 `be70f1682fdb9548ff174a6899080c480858d906f81522379d9bfe468d9b1410`. El ensayo de laboratorio anterior registró un incremento solicitado de 0,5° en J1, acuse `[1,3]` y error final máximo de aproximadamente `8,16 × 10⁻⁶ rad`. La integración de esta carpeta se comprueba por separado en el IRC5 de laboratorio.

En Fedora, abre **Solo lectura** con el lanzador. Elige **RWS/RAPID — Trayectorias v2** y usa **Conectar y preparar IRC5**. La preparación comprueba el servidor y la tarea antes de habilitar ejecución. Pasa a **Control físico** para revisar y ejecutar el objetivo desde FK, IK o Tareas. El registro distingue la referencia, la duración solicitada y la respuesta observada por RWS.

## Borrador virtual

El generador ofrece LSPB ideal y un perfil quíntico con aceleración continua y jerk acotado durante el tramo. Usa radianes y segundos en ROS, grados en `jointtarget` y orden J1–J6. Cada borrador tiene versión, huella del plan, puntos y duración solicitada. El ejecutor de prueba correlaciona ID de ensayo, plan y punto. Un resultado incierto cierra el despacho. La salida `Imt342TrajectoryDraft.mod` exige armado explícito del ensayo virtual y retorna en un controlador físico según `RobOS()`.

~~~sh
python3 -m experimental.trajectory_control.rws_rapid \
  --inicio 0 0 0 0 0 0 --destino .05 -.02 0 0 0 0 \
  --perfil quintic --duracion 2 --intervalos 1 --jerk-maximo 5 5 5 5 5 5 \
  --mod /tmp/Imt342TrajectoryDraft.mod --simular > /tmp/imt342_rapdraft.json
~~~

El ejemplo anterior es sintético. Si se cargara en un controlador virtual, inspecciona la posición inicial, herramienta y recorrido, verifica compilación y vincula el ID antes de iniciar deliberadamente la rutina candidata. Identifica el acuse por marcador RAPID y contrástalo con las posiciones. La cancelación de este borrador se comprueba entre tramos. La parada durante un movimiento requiere validar el procedimiento de RobotStudio/IRC5. El gestor mock permite observar estados y resultados, pero no instala programas experimentales por RWS. Para volver a la operación normal, cierra el ensayo virtual y selecciona RWS/RAPID Normal en la sesión apropiada.

## Discretización y compatibilidad

La estrategia conservadora del generador es un punto final con parada fine. Para un caso sintético de 3 s y Δq = [0.30, −0.15, 0, 0, 0, 0] rad con límites de estudio de 1 rad/s, 1 rad/s² y 5 rad/s³, el control de paradas modelo marca 0 de 1, 2 de 4, 16 de 16 y 64 de 64 tramos como demasiado cortos para parar en cada punto. La métrica usa el tiempo mínimo del mismo perfil desde reposo. No ejecuta ni simula el planificador ABB. Un blending permitiría continuar entre puntos y podría reducir la exactitud espacial. No se activa blending en la salida.

La duración puede estar limitada por el controlador. En otros movimientos, `speeddata` y `VelSet` gobiernan consignas o límites de velocidad de RAPID, `AccSet` reduce aceleración y rampa y `PathAccLim` limita aceleración del TCP. La referencia ROS conserva las cotas de estudio. ABB determina q̇ y q̈ del movimiento real. Las velocidades nominales ABB de referencia para IRB 120-3/0.6 son 250, 250, 250, 320, 320 y 420 °/s, recogidas en la ficha 3HAC035960-005, §1.8.3 y reflejadas en el modelo. Las cotas de aceleración y jerk usadas aquí son sintéticas. Speed Ratio, herramienta, carga y restricciones del controlador afectan el resultado físico.

La comparación entre ensayos debe registrar tiempos del controlador si están disponibles, acuses y feedback con sus propios relojes. El siguiente ensayo físico se define desde la postura real y tras revisar recorrido y condiciones GO. [ABB RAPID RW6, 3HAC050917](https://library.e.abb.com/public/b227fcd260204c4dbeb8a58f8002fe64/Rapid_instructions.pdf)
