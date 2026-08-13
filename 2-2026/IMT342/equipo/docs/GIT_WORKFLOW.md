# Flujo de trabajo Git y GitHub — IMT-342

Este documento define el procedimiento operativo acordado por el equipo. No pretende explicar Git de forma general; establece cómo se utilizarán Git y GitHub en este proyecto académico.

## 1. Repositorio y remotos

El equipo utiliza el fork completo `mario-nina/planificacion-UCB` del repositorio oficial `berquiroga/planificacion-UCB`. GitHub realiza forks a nivel de repositorio, por lo que el fork conserva también las demás materias del repositorio aunque el equipo sólo trabaja en `2-2026/IMT342`.

El fork es el repositorio canónico de colaboración. Todos los integrantes clonarán el mismo fork; no se crearán repositorios paralelos para el trabajo del equipo y no se utilizarán archivos ZIP como mecanismo de colaboración.

Se emplean dos remotos:

```text
origin   → git@github.com:mario-nina/planificacion-UCB.git
upstream → git@github.com:berquiroga/planificacion-UCB.git
```

`origin` es el remoto normal para `push`, Pull Requests y colaboración. `upstream` se utiliza para consultar e incorporar actualizaciones oficiales del docente; su uso es una convención interna del equipo y no un requisito explícito de la cátedra.

En el clon de Mario (`mario-nina`), GitHub CLI tiene como repositorio predeterminado `mario-nina/planificacion-UCB` y Git tiene `remote.pushDefault=origin`. Estas configuraciones reducen el riesgo de ejecutar por defecto operaciones sobre `upstream`.

El nombre original del repositorio, `planificacion-UCB`, se conserva. No se alteran arbitrariamente la visibilidad, la estructura global ni la automatización oficial de seguimiento presente o incorporada posteriormente.

## 2. Modelo de ramas

Las familias obligatorias definidas por la cátedra son `main`, `dev` y `feature/*`.

### `main`

`main` es la rama estable. Para cambios producidos por el equipo, sólo debe recibir estados del PFI que hayan sido integrados y validados. También puede recibir actualizaciones oficiales provenientes de `upstream`; esta excepción no convierte `main` en una rama de trabajo cotidiano.

### `dev`

`dev` se crea inicialmente desde `main` y permanece como rama de integración durante el semestre. Es la base normal para nuevas ramas de trabajo.

### `feature/*`

Las ramas `feature/*` son ramas de trabajo. Toda feature de primer nivel nace normalmente de `dev`, salvo instrucción oficial expresa en sentido contrario.

```text
main
 └── dev
      ├── feature/tp1-rotation
      ├── feature/h1-urdf-model
      ├── feature/h1-forward-kinematics
      └── ...
```

Una rama auxiliar de una entrega académica puede nacer de la rama de entrega correspondiente:

```text
feature/tp1-rotation
├── feature/tp1-active-passive
├── feature/tp1-composition
└── feature/tp1-python-validation
```

Las ramas representan tareas o entregables, no personas. No se crean ramas personales como `feature/mario`.

## 3. Destino de las ramas de trabajo

El origen normal de las features es común; su destino depende del propósito.

### Prácticas y guías académicas

Una rama de entrega como `feature/tp1-rotation` se utiliza para desarrollar y entregar la práctica. Una vez presentada, se conserva como evidencia y se considera congelada salvo que el docente solicite una corrección o una nueva versión.

Las ramas auxiliares internas de la práctica pueden integrarse a la rama de entrega mediante Pull Request. La rama de entrega no se integra automáticamente a `dev` ni a `main`.

Si posteriormente una solución de una práctica resulta útil para el PFI, se reutiliza o implementa selectivamente desde una feature del PFI; no se fusiona toda la práctica a `dev` por defecto.

### Proyecto Final Integrador

Las features del PFI se integran mediante Pull Request a `dev`. Cuando el estado acumulado de `dev` esté estable y validado para el PFI, se integra mediante Pull Request a `main`.

```text
feature/h* → dev → main
```

## 4. Preparación de una tarea

Antes de crear una rama se debe comprender la consigna, identificar el resultado verificable, dependencias, responsable, deadline interno, checkpoints y criterio de finalización.

Una tarea es una unidad lógica de trabajo. No debe ser tan amplia como “hacer TP1” ni fragmentarse artificialmente en microtareas sin valor independiente.

Cada tarea tendrá un responsable principal, pero no existen dueños permanentes de módulos. Una tarea puede reasignarse si no muestra progreso sustantivo en un checkpoint, si el responsable comunica imposibilidad de continuar, si existe un bloqueo persistente o si amenaza el deadline interno.

La reasignación conserva la misma rama cuando sea posible. El trabajo útil debe estar publicado para que otra persona pueda continuar sin reconstruirlo desde cero.

## 5. Inicio de una sesión de trabajo

Antes de trabajar se verifica el estado local y la rama activa. Las ramas base se actualizan desde `origin` antes de crear nuevas features.

Secuencia conceptual:

```text
verificar estado local
→ actualizar referencias remotas
→ actualizar rama base
→ verificar rama activa
→ trabajar
```

En ramas base compartidas se prefiere una actualización fast-forward, por ejemplo `git pull --ff-only`, para impedir que un `pull` genere merges locales accidentales. Cuando exista divergencia, se utilizará `fetch` y una integración explícita en lugar de resolverla automáticamente mediante `pull`.

## 6. Trabajo y commits

Cada integrante trabaja sólo dentro del alcance de su tarea. Los cambios no relacionados se separan.

Antes de un commit se revisan el estado, el diff y el contenido staged. `git add .` no está prohibido, pero no se utiliza a ciegas.

No existe una cuota fija de commits. La regla es una unidad lógica por commit. Los checkpoints deben mostrar progreso sustantivo versionado y publicado, no commits vacíos o artificiales.

Los commits ya publicados no se reescriben normalmente. Una corrección posterior se registra como un nuevo commit, habitualmente `fix(...)`, en lugar de modificar silenciosamente la historia.

La convención exacta de commits se define en `CONVENTIONS.md`.

## 7. Push y checkpoints

`commit` registra historia local; `push` publica commits en GitHub. Los checkpoints del equipo se verifican sobre evidencia visible en GitHub: commits, ramas, archivos, resultados y estado de la tarea.

Al finalizar una sesión se revisa el estado local, se publica el progreso coherente disponible, se actualiza el estado de la tarea y se comunica cualquier bloqueo.

## 8. Gestión de tareas en GitHub

GitHub Issues es la unidad operativa para tareas concretas. Cada Issue debe tener un objetivo, resultado esperado, responsable, deadline interno y criterio de finalización suficientemente claros para permitir una eventual reasignación.

Los campos nativos y el Project se utilizarán de la siguiente forma:

| Elemento | Uso |
|---|---|
| Issue | Tarea lógica concreta. |
| Assignee | Responsable actual. |
| Milestone | Entrega completa: TP, guía o hito. |
| Project | Vista global del trabajo de IMT-342. |
| Status | `Pendiente`, `En progreso`, `En revisión`, `Bloqueada`, `Terminada`. |
| Deadline interno | Fecha/hora interna de la tarea. |

No se utiliza `Reasignada` como estado. Una reasignación cambia el Assignee y queda documentada en la actividad o comentarios del Issue.

No se añaden inicialmente story points, sprints, velocity ni campos Scrum que no tengan una necesidad real. La prioridad tampoco se añade como campo hasta que el volumen de trabajo lo justifique.

Las labels personalizadas se mantienen al mínimo y se añaden sólo cuando exista una necesidad real de filtrado por área técnica.

## 9. Milestones y deadlines

El Milestone representa la fecha oficial de una entrega. Los Issues contienen deadlines internos anteriores a la fecha oficial. El deadline del docente nunca se utiliza como deadline normal de desarrollo.

### Ventana martes → jueves

| Momento aproximado | Objetivo interno |
|---|---|
| Martes después de clase | Analizar, dividir, asignar y comenzar. |
| Miércoles 14:00–16:00 | Primer checkpoint con progreso real. |
| Miércoles 22:00 | Checkpoint principal con avance sustantivo. |
| Jueves 08:00 | Checkpoint crítico; posible reasignación. |
| Jueves 12:00 | Cierre del desarrollo normal. |
| Jueves 14:00 | Versión interna final. |
| 14:00–16:00 | Buffer antes de una entrega cercana a las 16:00. |

### Ventana jueves → martes

| Momento aproximado | Objetivo interno |
|---|---|
| Jueves después de clase | Analizar, dividir, asignar y detectar dependencias. |
| Viernes 20:00 | Primer checkpoint. |
| Domingo 18:00 | Progreso intermedio sustantivo. |
| Lunes 20:00 | Checkpoint principal con soluciones funcionales. |
| Martes 08:00 | Checkpoint crítico; posible reasignación. |
| Martes 12:00 | Cierre del desarrollo normal. |
| Martes 14:00 | Versión interna final. |
| 14:00–16:00 | Buffer antes de una entrega cercana al horario de clase. |

Estas ventanas son plantillas. Con plazos de hasta tres días se usan checkpoints frecuentes; con cuatro a siete días, checkpoints cada 24–48 horas; con plazos mayores, objetivos semanales y checkpoints específicos.

## 10. Pull Requests

Se utiliza Pull Request siempre que se integre una rama en otra.

Flujos típicos:

```text
feature/h* → PR → dev

dev → PR → main

feature/tp1-* → PR → feature/tp1-rotation
```

Un PR debe explicar qué cambió, por qué cambió, cómo se verificó y qué tarea está relacionada. Su título seguirá una forma compatible con Conventional Commits cuando resulte natural.

Cuando un PR complete totalmente un Issue puede utilizar una referencia de cierre como `Closes #N`. Si sólo contribuye parcialmente, se referencia el Issue sin cerrarlo automáticamente.

## 11. Revisión

Se requiere normalmente al menos una aprobación de otro integrante para integrar en `dev` o `main`. El autor no aprueba su propio trabajo cuando existe otro integrante disponible.

El revisor verifica la consigna, corrección técnica, archivos modificados, coherencia de commits, legibilidad, convenciones, pruebas, documentación y rama destino.

Los comentarios de revisión se corrigen sobre la misma rama y el mismo PR. Las conversaciones deben quedar resueltas antes del merge en `dev` y `main`.

En una contingencia académica real, sin revisor disponible y con un deadline inmediato, una persona con permisos suficientes puede necesitar realizar una integración excepcional. Debe quedar documentada en el PR y no constituye el flujo normal.

## 12. Método de merge

La configuración preferida del repositorio es:

| Método | Estado |
|---|---|
| Merge commit | Habilitado. |
| Squash merge | Deshabilitado. |
| Rebase merge | Deshabilitado. |
| Auto-merge | Deshabilitado inicialmente. |
| Auto-delete de ramas | Deshabilitado inicialmente. |

Se preservan los commits lógicos originales porque la cátedra evalúa el historial y penaliza cargas masivas al final.

Las ramas temporales pueden eliminarse manualmente después de verificar el merge. Las ramas académicas de entrega se conservan.

## 13. Protección de ramas

La política objetivo para `main` y `dev` es:

| Regla | `main` | `dev` |
|---|---:|---:|
| Pull Request requerido | Sí | Sí |
| Aprobaciones requeridas | 1 | 1 |
| Conversaciones resueltas | Sí | Sí |
| Force push | Bloqueado | Bloqueado |
| Eliminación | Bloqueada | Bloqueada |
| Push directo cotidiano | No | No |
| Status checks obligatorios | No inicialmente | No inicialmente |
| Commits firmados obligatorios | No | No |

Las features no tendrán protección automática general en la primera versión. La prohibición de force-push y reescritura de historia compartida se aplica por política.

No se utilizará `CODEOWNERS` inicialmente. No se exigirán commits firmados. Los required status checks se introducirán únicamente cuando exista CI propia conocida y estable.

No se modificará ni deshabilitará automatización oficial de seguimiento presente o incorporada posteriormente sin comprender previamente su función.

## 14. Sincronización con `origin`

Los integrantes colaboran normalmente mediante `origin`. Antes de crear una nueva feature se actualiza `dev` desde `origin`. Una feature activa no necesita incorporar cada cambio remoto inmediatamente; se actualiza cuando necesita trabajo integrado reciente, cuando se aproxima un PR, cuando existe un conflicto o cuando aparece una actualización relevante.

Una nueva rama no debe crearse deliberadamente desde una copia local desactualizada.

## 15. Sincronización con `upstream`

Las actualizaciones del docente se consultan primero mediante `fetch`. No se utilizará un `pull upstream main` ciego.

Flujo conceptual:

```text
fetch upstream
→ inspeccionar cambios
→ incorporar actualización oficial en main
→ verificar
→ propagar main a dev cuando corresponda
→ avisar al equipo
→ actualizar features activas sólo si es necesario
```

La operación `upstream → main` se coordina para que una sola persona la ejecute en cada ocasión. Los demás integrantes reciben posteriormente el cambio desde `origin`.

El mecanismo exacto para integrar `upstream/main` en una `main` protegida se cerrará después de aplicar y probar las protecciones reales del fork. La política no cambia: la sincronización debe ser explícita, revisada y trazable; no debe convertirse en desarrollo directo sobre `main`.

Las actualizaciones oficiales pueden modificar contenido ajeno al PFI. El equipo no utiliza `ours` o `theirs` ciegamente. Si aparece un conflicto en contenido de otras materias, se revisa primero si existe una modificación accidental del equipo.

## 16. Conflictos

Un conflicto se resuelve comprendiendo ambos cambios, editando la solución correcta, ejecutando las verificaciones necesarias y completando después la integración. No se considera automáticamente culpable a ninguno de los autores.

Procedimiento conceptual:

```text
identificar
→ comprender ambos cambios
→ resolver
→ probar
→ stage
→ completar merge
→ push
```

Si la resolución no es clara, se puede abortar la operación y analizarla antes de continuar.

Los Jupyter Notebooks requieren especial coordinación porque un `.ipynb` es un documento JSON y puede producir conflictos difíciles de fusionar cuando varias personas editan simultáneamente el mismo archivo. Se preferirá dividir responsabilidades y reducir edición concurrente del mismo notebook.

## 17. Historia publicada y comandos destructivos

No se usa `git push --force` sobre ramas compartidas. Tampoco se reescribe normalmente historia publicada mediante `rebase`, `commit --amend` o `reset` cuando otros integrantes ya pueden depender de esos commits.

`git reset --hard` y `git clean -fd` se consideran operaciones destructivas y no se utilizan como solución genérica a problemas de Git.

`git stash` se permite como almacenamiento local temporal; no sustituye commits ni push y no debe convertirse en almacén permanente de tareas.

Los conflictos no se “resuelven” descargando ZIPs, copiando archivos encima o reconstruyendo manualmente el repositorio fuera de Git.

## 18. Actualización de la documentación

Cuando una política cambie, se registra el motivo en `DECISIONS.md`, se actualiza este documento y se realiza un commit `docs(...)`. La documentación vigente y el historial de decisiones cumplen funciones distintas y no deben duplicarse íntegramente.
