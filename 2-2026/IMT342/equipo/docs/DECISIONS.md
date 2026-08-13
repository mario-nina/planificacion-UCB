# Registro de decisiones — IMT-342

Este documento conserva las decisiones importantes del equipo, su motivo y sus consecuencias. La política vigente se desarrolla en los documentos operativos correspondientes; aquí se registra por qué se adoptó.

## D-001 — Fork completo del repositorio oficial

**Decisión:** utilizar un fork completo de `berquiroga/planificacion-UCB`.

**Motivo:** GitHub realiza forks a nivel de repositorio, no de subcarpeta. La asignatura está dentro de `2-2026/IMT342` en un repositorio que contiene otras materias.

**Consecuencia:** el equipo hereda contenido ajeno a IMT-342 y se compromete a no modificarlo.

## D-002 — Repositorio canónico del equipo

**Decisión:** utilizar `mario-nina/planificacion-UCB` como repositorio canónico de colaboración.

**Motivo:** el equipo necesita una única fuente compartida para ramas, commits, Issues, Pull Requests y entregas.

**Consecuencia:** todos los integrantes trabajarán sobre el mismo fork y no sobre repositorios paralelos.

## D-003 — Un solo repositorio de colaboración

**Decisión:** todos los integrantes clonarán el mismo fork.

**Motivo:** evitar repositorios paralelos, copias divergentes y transferencia manual de archivos.

**Consecuencia:** no se utilizarán repositorios alternativos ni archivos ZIP como flujo de colaboración.

## D-004 — Alcance restringido a IMT-342

**Decisión:** el equipo trabajará únicamente en `2-2026/IMT342`.

**Motivo:** el fork incluye otras materias del docente que no pertenecen al equipo.

**Consecuencia:** no se eliminan, renombran ni reorganizan directorios de otras asignaturas.

## D-005 — Espacio propio `equipo/`

**Decisión:** concentrar el trabajo del equipo bajo `2-2026/IMT342/equipo/`.

**Motivo:** separar material oficial y producción del equipo, reduciendo conflictos con futuras actualizaciones del docente.

**Consecuencia:** las carpetas semanales del docente no se utilizan como espacio normal de trabajo.

## D-006 — Remotos `origin` y `upstream`

**Decisión:** `origin` apunta al fork del equipo y `upstream` al repositorio del docente.

**Motivo:** separar colaboración del equipo y sincronización con la fuente oficial.

**Consecuencia:** los pushes normales van a `origin`; `upstream` se utiliza para consulta e incorporación controlada de actualizaciones.

## D-007 — Conservar inicialmente el nombre y configuración heredada

**Decisión:** mantener el nombre `planificacion-UCB` y evitar cambios arbitrarios de visibilidad, estructura global o automatización oficial.

**Motivo:** el seguimiento académico puede depender de la configuración del repositorio o de automatización incorporada por el docente.

**Consecuencia:** cualquier modificación global requiere primero comprender su efecto.

## D-008 — Clonado completo, sin sparse checkout inicial

**Decisión:** clonar el repositorio completo en la primera configuración.

**Motivo:** reducir complejidad para un equipo que está consolidando su flujo de Git.

**Consecuencia:** sparse checkout sólo se reconsiderará si el tamaño del repositorio lo vuelve necesario.

## D-009 — Identidad Git individual

**Decisión:** cada integrante utilizará su propia cuenta de GitHub, `user.name`, correo y credenciales.

**Motivo:** preservar trazabilidad real de autoría.

**Consecuencia:** no se comparten contraseñas, tokens o claves SSH y un integrante no realiza commits simulando ser otro.

## D-010 — Sin coautoría habitual por trailers

**Decisión:** no utilizar `Co-authored-by` como práctica habitual.

**Motivo:** el equipo quiere que cada contribución quede asociada al autor real mediante commits individuales.

**Consecuencia:** la colaboración se demuestra con commits, Issues, Pull Requests y reviews.

## D-011 — Responsabilidades por tarea, no por módulo

**Decisión:** no habrá propietarios permanentes de módulos.

**Motivo:** las responsabilidades cambian entre prácticas e hitos y todos deben adquirir comprensión del proyecto.

**Consecuencia:** cada tarea tiene un responsable principal, pero puede reasignarse.

## D-012 — Tareas como unidades lógicas

**Decisión:** una tarea representa un objetivo concreto, verificable e integrable.

**Motivo:** evitar tanto tareas demasiado amplias como microtareas artificiales.

**Consecuencia:** una tarea debe documentar objetivo, resultado esperado, archivos o alcance, responsable, deadline interno y criterio de finalización.

## D-013 — Reasignación neutral de tareas

**Decisión:** una tarea puede reasignarse cuando falta progreso sustantivo, existe imposibilidad comunicada, bloqueo persistente o amenaza al deadline.

**Motivo:** garantizar continuidad sin designar una persona permanente como “rescate”.

**Consecuencia:** se preserva el trabajo útil y, cuando sea posible, el nuevo responsable continúa en la misma rama.

## D-014 — Checkpoints basados en evidencia GitHub

**Decisión:** los checkpoints se verifican con trabajo visible en GitHub.

**Motivo:** evitar depender únicamente de reportes verbales.

**Consecuencia:** los integrantes deben publicar progreso coherente antes de checkpoints relevantes.

## D-015 — Deadlines internos anteriores a la entrega oficial

**Decisión:** la fecha oficial no será el deadline normal de desarrollo.

**Motivo:** reservar tiempo para integración, revisión y correcciones.

**Consecuencia:** se utilizarán plantillas martes→jueves y jueves→martes, adaptables a la fecha real.

## D-016 — Modelo principal `main`, `dev`, `feature/*`

**Decisión:** respetar las familias de ramas exigidas por la cátedra.

**Motivo:** requisito oficial y compatibilidad con el seguimiento académico.

**Consecuencia:** no se introducirá inicialmente GitFlow completo con `release/*` o `hotfix/*`.

## D-017 — `dev` nace inicialmente desde `main`

**Decisión:** crear `dev` una sola vez desde el estado inicial de `main`.

**Motivo:** establecer una rama de integración común.

**Consecuencia:** `dev` permanece durante el semestre y no se recrea por cada actividad.

## D-018 — Toda feature de primer nivel nace normalmente desde `dev`

**Decisión:** unificar el punto de partida normal de prácticas y PFI en `dev`.

**Motivo:** evitar una regla adicional donde algunas features nacieran de `main` y otras de `dev`.

**Consecuencia:** la diferencia entre una práctica y una feature del PFI se determina por su destino, no por su origen.

## D-019 — Ramas auxiliares pueden nacer de una feature de entrega

**Decisión:** las subtareas de una práctica pueden ramificarse desde la feature principal de la entrega.

**Motivo:** permitir trabajo paralelo sin contaminar `dev` con ramas cuya única finalidad es componer una entrega concreta.

**Consecuencia:** por ejemplo, `feature/tp1-composition` puede integrarse a `feature/tp1-rotation`.

## D-020 — Prácticas no regresan automáticamente a `dev` o `main`

**Decisión:** una rama académica de entrega termina su ciclo en la propia rama requerida por el docente.

**Motivo:** conservar la evidencia exacta de la entrega y separar prácticas independientes del desarrollo acumulativo del PFI.

**Consecuencia:** `feature/tp1-rotation` se preserva después de la entrega.

## D-021 — Features del PFI regresan a `dev`

**Decisión:** las features del PFI se integran mediante Pull Request a `dev`.

**Motivo:** `dev` es la rama de integración continua definida por la cátedra.

**Consecuencia:** sólo estados estables y validados del PFI pasan posteriormente de `dev` a `main`.

## D-022 — `main` puede recibir actualizaciones oficiales

**Decisión:** además de PFI estable, `main` puede incorporar publicaciones oficiales provenientes de `upstream`.

**Motivo:** el fork debe poder mantenerse sincronizado con nuevas semanas, rúbricas o correcciones del docente.

**Consecuencia:** la expresión “main sólo contiene PFI estable” se interpreta únicamente respecto de cambios producidos por el equipo.

## D-023 — Nombres de ramas técnicos y consistentes

**Decisión:** utilizar inglés técnico, minúsculas ASCII y guiones en ramas.

**Motivo:** mantener nombres predecibles y compatibles con herramientas.

**Consecuencia:** se evitan nombres personales, espacios, tildes, fechas por defecto y variantes como `final2`.

## D-024 — Conventional Commits con descripción en español

**Decisión:** usar `<type>(<scope>): <descripcion>`, con type/scope en inglés y descripción en español.

**Motivo:** cumplir el requisito oficial y mantener mensajes comprensibles para el equipo.

**Consecuencia:** se definen types y scopes iniciales en `CONVENTIONS.md`.

## D-025 — Una unidad lógica por commit, sin cuota fija

**Decisión:** no imponer un número mínimo artificial de commits.

**Motivo:** la cátedra penaliza commits masivos, pero microcommits vacíos tampoco aportan trazabilidad.

**Consecuencia:** cada commit debe corresponder a un cambio lógico y sustantivo.

## D-026 — No reescribir historia publicada

**Decisión:** evitar `rebase`, `commit --amend`, `reset` o force-push cuando modifican commits ya compartidos.

**Motivo:** preservar trazabilidad y evitar romper el trabajo de otros integrantes.

**Consecuencia:** una corrección posterior normalmente se expresa con un nuevo commit.

## D-027 — Pull Request para integrar ramas

**Decisión:** utilizar Pull Request siempre que una rama se integre a otra.

**Motivo:** proporcionar trazabilidad, revisión y contexto de integración.

**Consecuencia:** las tareas internas de una práctica también pueden integrarse mediante PR a la rama de entrega.

## D-028 — Una aprobación requerida

**Decisión:** exigir normalmente una aprobación para integrar a `dev` y `main`.

**Motivo:** una segunda aprobación obligatoria bloquearía fácilmente a un equipo de tres integrantes.

**Consecuencia:** el autor no se autoaprueba cuando existe otro integrante disponible.

## D-029 — Merge commits como único método normal

**Decisión:** habilitar merge commits y deshabilitar squash merge y rebase merge.

**Motivo:** preservar la secuencia de commits lógicos que la cátedra utiliza para seguimiento.

**Consecuencia:** el historial conserva commits originales más el punto explícito de integración.

## D-030 — Protección de `main` y `dev`

**Decisión:** proteger ambas ramas contra eliminación y force-push y exigir PR, una aprobación y conversaciones resueltas.

**Motivo:** impedir cambios accidentales en ramas compartidas críticas.

**Consecuencia:** el desarrollo cotidiano ocurre en features.

## D-031 — Bypass sólo como contingencia

**Decisión:** conservar capacidad de recuperación cuando exista una contingencia real, pero no utilizar el bypass como flujo normal.

**Motivo:** evitar que una protección impida cumplir una entrega por una situación excepcional.

**Consecuencia:** cualquier bypass relevante debe quedar documentado.

## D-032 — Sin required checks inicialmente

**Decisión:** no exigir status checks automáticos en la primera configuración.

**Motivo:** todavía no existe CI propia conocida y estable, y el material oficial indica que existe tracking automatizado que no debe alterarse sin análisis.

**Consecuencia:** los checks obligatorios se introducirán sólo cuando exista una validación confiable.

## D-033 — Sin CODEOWNERS ni firmas obligatorias inicialmente

**Decisión:** no utilizar `CODEOWNERS` ni exigir commits firmados en la primera versión.

**Motivo:** no existen propietarios permanentes de módulos y la complejidad adicional no está justificada para tres integrantes.

**Consecuencia:** la revisión se asigna de forma flexible y la identidad se controla mediante cuentas Git/GitHub individuales.

## D-034 — Auto-merge y auto-delete deshabilitados inicialmente

**Decisión:** realizar merges y eliminación de ramas de forma deliberada.

**Motivo:** el equipo está consolidando el workflow y algunas ramas deben conservarse como evidencia académica.

**Consecuencia:** después de cada merge se decide manualmente si la rama temporal se elimina o se conserva.

## D-035 — Automatización oficial no se modifica sin análisis

**Decisión:** no editar, deshabilitar o reemplazar automatización oficial de seguimiento presente o incorporada posteriormente sin comprender su función.

**Motivo:** el README oficial indica que el tracking académico utiliza GitHub Actions.

**Consecuencia:** cualquier CI propia deberá coexistir con la automatización oficial.

## D-036 — Sincronización `upstream` mediante fetch e inspección

**Decisión:** consultar primero `upstream` con `fetch`, revisar cambios y luego integrarlos de forma controlada.

**Motivo:** evitar que un `pull` ciego modifique la rama actual sin comprender qué publicó el docente.

**Consecuencia:** la actualización `upstream → main` será coordinada y trazable.

## D-037 — Una sola persona sincroniza `upstream` en cada ocasión

**Decisión:** coordinar la actualización oficial para que una sola persona la ejecute cada vez.

**Motivo:** evitar que varios integrantes intenten integrar simultáneamente los mismos cambios.

**Consecuencia:** el resto del equipo recibe después la actualización desde `origin`.

## D-038 — Propagar actualizaciones oficiales de `main` a `dev`

**Decisión:** mantener `dev` razonablemente alineado después de actualizaciones relevantes de `main`.

**Motivo:** las nuevas features deben poder partir de una base que conozca material oficial reciente.

**Consecuencia:** las features activas se actualizan sólo cuando sea necesario, no después de cada cambio remoto.

## D-039 — No usar force-push en ramas compartidas

**Decisión:** prohibir `git push --force` en `main`, `dev`, ramas de entrega y otras ramas compartidas.

**Motivo:** puede reescribir historia y eliminar commits de otros integrantes.

**Consecuencia:** los conflictos se resuelven mediante integración explícita y nuevos commits cuando corresponda.

## D-040 — Jupyter requiere coordinación especial

**Decisión:** reducir edición simultánea del mismo `.ipynb`.

**Motivo:** los notebooks son documentos JSON y los conflictos pueden ser difíciles de fusionar.

**Consecuencia:** las tareas de TP1 se dividirán buscando minimizar concurrencia sobre el mismo archivo.

## D-041 — `stash` sólo como almacenamiento temporal

**Decisión:** permitir `git stash` como herramienta temporal, no como repositorio de trabajo.

**Motivo:** evitar que cambios importantes queden olvidados fuera del historial compartido.

**Consecuencia:** el trabajo útil debe terminar en commits y push.

## D-042 — Issues como unidad operativa de trabajo

**Decisión:** gestionar tareas concretas mediante GitHub Issues.

**Motivo:** conservar responsable, contexto, deadline y trazabilidad en el mismo sistema donde vive el código.

**Consecuencia:** los archivos Markdown no sustituyen el gestor de tareas.

## D-043 — Milestones para entregas

**Decisión:** utilizar Milestones para TP, guías e hitos completos.

**Motivo:** separar la entrega global de sus tareas constituyentes.

**Consecuencia:** el Milestone contiene el deadline oficial; cada Issue mantiene un deadline interno anterior.

## D-044 — Un GitHub Project para toda IMT-342

**Decisión:** utilizar un único Project llamado conceptualmente `IMT342 — Robótica 2-2026`.

**Motivo:** mantener una vista global sin crear un tablero separado por cada práctica.

**Consecuencia:** los estados iniciales son `Pendiente`, `En progreso`, `En revisión`, `Bloqueada` y `Terminada`.

## D-045 — Campos mínimos del Project

**Decisión:** utilizar inicialmente `Status` y `Deadline interno`, complementados por Assignee y Milestone nativos del Issue.

**Motivo:** evitar burocracia de gestión innecesaria.

**Consecuencia:** no se añaden de inicio story points, velocity, sprint ni prioridad.

## D-046 — Labels mínimas y bajo demanda

**Decisión:** no diseñar una taxonomía extensa de labels desde el inicio.

**Motivo:** el equipo es pequeño y todavía no existe volumen que justifique categorías complejas.

**Consecuencia:** se incorporarán labels técnicas sólo cuando aporten filtrado real.

## D-047 — Estructura documental de seis archivos

**Decisión:** utilizar `equipo/README.md` más cinco documentos bajo `equipo/docs/`.

**Motivo:** separar navegación, operación, convenciones, contexto, estado y decisiones sin fragmentar en exceso.

**Consecuencia:** no se crean inicialmente `CONTRIBUTING.md`, `TASK_MANAGEMENT.md`, `BRANCHES.md`, `COMMITS.md` ni documentos redundantes.

## D-048 — Fuente de verdad por tema

**Decisión:** cada tipo de información tiene un documento o herramienta principal.

**Motivo:** evitar inconsistencias por duplicación.

**Consecuencia:** workflow en `GIT_WORKFLOW.md`, convenciones en `CONVENTIONS.md`, contexto estable en `PROJECT_CONTEXT.md`, estado en `PROJECT_STATUS.md`, razones en `DECISIONS.md` y tareas en GitHub Issues/Project.

## D-049 — Sin templates `.github/` inicialmente

**Decisión:** no crear todavía plantillas automáticas de Pull Request o Issues.

**Motivo:** `.github/` afecta al repositorio completo y debe evitarse cualquier cambio global innecesario. La inspección inicial del checkout no detectó archivos bajo `.github/`.

**Consecuencia:** el formato esperado de Issues y PRs se documenta antes de automatizarlo.

## D-050 — Estructura interna del PFI diferida

**Decisión:** reservar conceptualmente `equipo/pfi/` sin diseñar todavía `src/`, `urdf/`, `launch/`, etc.

**Motivo:** no sobrearquitectar antes de conocer las necesidades reales del Hito 1 y del entorno ROS.

**Consecuencia:** la arquitectura técnica del PFI se definirá cuando comience el trabajo correspondiente.

## D-051 — No duplicar material oficial en `equipo/`

**Decisión:** no copiar guías, PDFs o fuentes del docente dentro de las carpetas de prácticas.

**Motivo:** evitar duplicación y versiones desactualizadas.

**Consecuencia:** las prácticas contienen los entregables del equipo y referencian el material oficial en su ubicación original.

## D-052 — Resultados pertenecen al trabajo que los genera

**Decisión:** no crear un directorio global `resultados/` desde el inicio.

**Motivo:** evitar mezclar resultados de prácticas, cinemática, visión y PFI.

**Consecuencia:** cada práctica o módulo organiza sus propios resultados cuando sean necesarios.

## D-053 — No versionar automáticamente todo artefacto generado

**Decisión:** versionar sólo archivos necesarios para reproducir, evaluar o documentar el trabajo.

**Motivo:** herramientas como Python, Jupyter, LaTeX y ROS pueden producir cachés y artefactos temporales.

**Consecuencia:** `.gitignore` se definirá durante la implementación según necesidades reales.

## D-054 — Documentación en español sin traducciones forzadas

**Decisión:** escribir documentación humana en español y conservar términos técnicos estándar en inglés.

**Motivo:** mantener claridad para el equipo sin inventar terminología poco natural.

**Consecuencia:** nombres como Pull Request, merge, branch, commit, origin y upstream se mantienen cuando corresponda.

## D-055 — Documentación sin checkboxes de progreso

**Decisión:** no utilizar listas de casillas como mecanismo de seguimiento dentro de los documentos permanentes.

**Motivo:** priorizar legibilidad y separar documentación de gestión operativa.

**Consecuencia:** el progreso se sigue en GitHub Issues/Project y `PROJECT_STATUS.md` sólo resume el estado.

## D-056 — Cambios de política se documentan explícitamente

**Decisión:** cuando una política cambie, registrar el motivo en este archivo y actualizar el documento operativo correspondiente.

**Motivo:** conservar tanto el estado vigente como la evolución de las decisiones.

**Consecuencia:** los cambios documentales importantes se versionan mediante commits `docs(...)`.

## D-057 — Las instrucciones oficiales prevalecen

**Decisión:** cualquier instrucción posterior explícita del docente prevalece sobre las convenciones internas.

**Motivo:** el workflow del equipo es un mecanismo para cumplir la materia, no una autoridad superior a la consigna académica.

**Consecuencia:** las discrepancias se ajustan y se registran aquí.

## D-058 — Operaciones locales predeterminadas hacia el fork

**Decisión:** configurar GitHub CLI para usar `mario-nina/planificacion-UCB` como repositorio predeterminado del clon y configurar `remote.pushDefault=origin` localmente.

**Motivo:** `gh repo fork --clone` dejó inicialmente al repositorio padre como default para GitHub CLI y el equipo quiere reducir el riesgo de ejecutar por defecto operaciones sobre el repositorio del docente.

**Consecuencia:** las operaciones normales de `gh` y los pushes sin remoto explícito se orientan al fork del equipo.
