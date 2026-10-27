# Gate 0 — Plan de pruebas para Gate 1

Fecha de elaboración: 2026-10-27
Proyecto: Hito 3 progresivo — ECI-3/4
Robot de trabajo: ABB IRB 120

Estado: PLANIFICADO / NO EJECUTADO

## 1. Objetivo

Diseñar dos pruebas de comunicación y lectura de estados
entre ROS 2 y el controlador ABB, sin movimiento físico.

Este documento no autoriza conectarse al controlador
ni ejecutar las pruebas.

Fuente académica:
S12_material_oficial.pdf
w12_s01_guia_autonoma_bitacora.pdf

## 2. Condiciones previas

Antes de cualquier prueba sobre el sistema real:

- Confirmar modelo y revisión del controlador.
- Confirmar versión de RobotWare.
- Identificar interfaz y cliente compatibles.
- Obtener autorización institucional de acceso.
- Confirmar red y método de conexión permitidos.
- Identificar operador o responsable autorizado.
- Definir y aprobar consultas exclusivamente de lectura.
- Disponer del protocolo local aplicable.
- Registrar fecha, responsables y alcance autorizado.

Estado actual: condiciones incompletas.

Decisión actual: NO-GO para pruebas físicas.

El GO de una prueba de lectura no constituye
autorización para ejecutar movimiento.

## 3. Prueba G1-01 — Comunicación sin movimiento

### Hipótesis

Una interfaz compatible permitirá realizar una
consulta autorizada al controlador ABB.

### Objetivo

Demostrar intercambio de solicitudes y respuestas
con el controlador identificado, sin ejecutar acciones
que modifiquen su estado operativo.

### Procedimiento propuesto

1. Verificar todas las condiciones previas.
2. Identificar el controlador de destino autorizado.
3. Utilizar una consulta de solo lectura previamente
   aprobada por el responsable del laboratorio.
4. Registrar la respuesta o el error obtenido.
5. Identificar la procedencia de la respuesta.
6. Finalizar sin enviar consignas de movimiento.

No utilizar funciones de arranque de RAPID,
cambio de estado de motores ni comandos EGM.

### Variables y evidencias

- Identidad del controlador consultado.
- Interfaz y versión utilizadas.
- Tipo de consulta autorizada.
- Resultado de la consulta.
- Fecha y hora de observación.
- Error o diagnóstico si falla.
- Registro técnico sin credenciales ni secretos.

### Criterio de éxito

Respuesta válida, atribuible al controlador
identificado y obtenida mediante interfaz autorizada.

Una respuesta a ping por sí sola no es suficiente.

### Responsable de autorización

Laboratorio / responsable autorizado:
PENDIENTE DE IDENTIFICAR.

### Estado

NO EJECUTADO.

## 4. Prueba G1-02 — Lectura de estados articulares

### Hipótesis

La interfaz seleccionada permitirá adquirir
estados articulares procedentes del controlador ABB.

### Objetivo

Verificar la procedencia, estructura y coherencia
de los datos articulares observados sin movimiento.

### Procedimiento propuesto

1. Confirmar la autorización de acceso y lectura.
2. Identificar el recurso de estado documentado.
3. Obtener muestras sin ordenar movimiento.
4. Registrar identidad y origen de los datos.
5. Comprobar cantidad y orden de articulaciones.
6. Identificar unidades y convenciones.
7. Comparar únicamente con referencias equivalentes.
8. Registrar limitaciones y resultados.

No se moverán las juntas para generar cambios
en las lecturas.

### Variables y evidencias

- Origen de los estados.
- Número de articulaciones observadas.
- Nombres y orden cuando estén disponibles.
- Valores y unidades declaradas.
- Marca temporal o información de adquisición.
- Resultado de lectura y posibles errores.
- Registro reproducible de observación.

### Criterio de éxito

Se reciben estados atribuibles al controlador
físico y se documentan sus convenciones.

La coherencia con el URDF deberá verificarse
considerando posibles diferencias de nombres,
orden de juntas y unidades.

Un joint_states simulado no satisface este criterio.

### Responsable de autorización

Laboratorio / responsable autorizado:
PENDIENTE DE IDENTIFICAR.

### Estado

NO EJECUTADO.

## 5. Distinción de evidencias

Simulación:
Demuestra comportamiento del software bajo
las condiciones del modelo.

Conectividad:
Demuestra intercambio mediante una interfaz
determinada, no control de trayectorias.

Lectura de estados:
Demuestra adquisición de información del ABB
únicamente si su procedencia es verificable.

Movimiento físico:
No forma parte de Gate 1.

## 6. Eslabones todavía no verificables

- Compatibilidad real de RWS con el controlador.
- Disponibilidad de las interfaces autorizadas.
- Comunicación ROS–ABB en el laboratorio.
- Procedencia física de joint_states.
- Compatibilidad de EGM y control externo.
- Correspondencia entre el TCP físico y el URDF.

## 7. Condiciones de bloqueo

Una prueba no se ejecutará cuando:

- Falte autorización de laboratorio.
- No se conozca el controlador de destino.
- Se desconozcan las funciones de una solicitud.
- No pueda garantizarse que sea de solo lectura.
- La red o las credenciales no estén autorizadas.
- Exista una condición de seguridad no resuelta.

Las pruebas se mantendrán como bloqueadas
o no verificadas, según la evidencia disponible.

## 8. Evidencias futuras

Cuando existan pruebas autorizadas, documentar:

- ID de prueba G1-01 o G1-02.
- Fecha, operador y autorización.
- Hardware y software con versiones.
- Configuración relevante sin secretos.
- Consulta aprobada realizada.
- Resultado observado.
- Evidencia permitida.
- Interpretación y limitaciones.
- Siguiente acción.

No almacenar contraseñas, tokens ni
credenciales institucionales en Git.

## 9. Decisión actual

G1-01: NO EJECUTADO.
G1-02: NO EJECUTADO.

Controlador y permisos: PENDIENTES.

El desarrollo autónomo continúa únicamente
con investigación y validación offline.
