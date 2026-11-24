# Servidor articular IMT342 por RWS 1.0

El cliente `imt342_irb120_control.rws_backend` usa sesiones HTTP persistentes con Digest. La configuración requiere el origen del controlador, la unidad mecánica, la tarea, el módulo y la tolerancia articular de verificación. Le suministra las credenciales el proceso que lo inicia. Las lecturas de modo, motores, ejecución y J1–J6 funcionan independientemente de la presencia del módulo. El cliente realiza escrituras en AUTO y comprueba motores, ejecución, puntero del programa y versión del protocolo. MANUAL/RMMP queda por validar. El cliente lee el PP y no lo reubica.

El módulo propio `Imt342JointServer.mod` ofrece la rutina `ServeJointOrders`. Su entrada prepara un bucle de espera. El operador debe confirmar en IRC5 o controlador virtual que el RobotWare acepta los tipos RECORD persistentes vía RWS, que la tarea y unidad mecánica corresponden al robot, que no hay ejes externos comandados y que el PP identifica esta rutina. La instrucción candidata usa `MoveAbsJ`, `v10`, `fine` y `tool0`. Validar herramienta, carga, velocidad efectiva, límites, trayectoria y condición mecánica antes de cualquier prueba supervisada. La trayectoria generada en RAPID no sigue el calendario LSPB de RViz.

## Registros

- `rbVersion` identifica la versión 1.
- `pcOrder=[secuencia,jointtarget]` es una sola escritura bajo mastership de RAPID. El `jointtarget` expresa J1–J6 en grados y usa `9E9` para los ejes externos sin destino.
- `rbReply=[secuencia,fase]` indica 0 para reposo inicial, 1 para aceptado, 2 para ejecución, 3 para finalizado por RAPID, 4 para error RAPID y 5 para orden descartada al reubicar PP en la entrada de la rutina.
- El cliente toma la siguiente secuencia solamente cuando coinciden la orden y la respuesta y la fase es terminal o inicial. El límite de secuencia conserva exactitud entera en `num`. La reinicialización de los registros se hace en el laboratorio.
- HTTP 204 indica transmisión. La fase 3 se coteja con una lectura posterior de J1–J6. La máxima diferencia absoluta por eje, en radianes, decide el resultado con la tolerancia configurada.

## Pérdidas y parada

Un timeout al escribir puede corresponder a pérdida antes o después de la escritura. El cliente mantiene la secuencia pendiente, vuelve a consultar el acuse y nunca reenvía la orden. Con HTTP 204 y sin un acuse correlacionado al vencer el timeout de acuse, mantiene la misma secuencia en estado desconocido. Al restablecerse el enlace, conserva las transiciones observables y pide nueva lectura medida al detectar un fin.

Si se solicita cancelar, el cliente verifica que el PP está en el servidor articular y que la orden no fue reemplazada por otra secuencia. Solicita la parada RAPID documentada, que actúa sobre la ejecución RAPID seleccionada por el controlador, y confirma después el estado detenido y una lectura de postura. Tras parar, el operador debe reubicar explícitamente el PP en la **entrada** de `ServeJointOrders` antes de iniciar el servidor de nuevo. Reanudar por el PP suspendido puede retomar un `MoveAbsJ` interrumpido. En la entrada, el módulo descarta la orden pendiente y conserva una respuesta que ya era terminal. Si sigue pendiente, revisar PP, registro y ejecución en el IRC5 antes de solicitar otro destino. Con enlaces que no se recuperan, el operador usa los controles reales de la celda.

La política de cliente presupone un único proceso autorizado para comandar el robot. Cotejar cadencia de polling, comportamiento de 204 y alcance de parada en el RobotWare de la celda.

## Referencias ABB

- [Manual de Robot Web Services](https://developercenter.robotstudio.com/api/rwsApi/)
- [Mastership de un dominio](https://developercenter.robotstudio.com/api/rwsApi/msh_domain_request_page.html)
- [Escritura de un símbolo RAPID](https://developercenter.robotstudio.com/api/rwsApi/rapid_symbol_data_set_page.html)
- [Punteros de tarea RAPID](https://developercenter.robotstudio.com/api/rwsApi/rapid_task_pcp_get_page.html)
- [Parada RAPID](https://developercenter.robotstudio.com/api/rwsApi/rapid_execution_stop_page.html)
