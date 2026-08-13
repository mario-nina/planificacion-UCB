# Convenciones del equipo — IMT-342

Este documento concentra reglas de nombres, commits, idioma y formato. Los procedimientos completos se encuentran en `GIT_WORKFLOW.md`.

## 1. Ramas

Formato normal:

```text
feature/<identificador>-<descripcion>
```

Las instrucciones explícitas del docente tienen prioridad sobre la convención interna. Por ejemplo, TP1 debe utilizar exactamente `feature/tp1-rotation`.

### Prácticas

```text
feature/tp<n>-<descripcion>
```

Ejemplos:

```text
feature/tp1-rotation
feature/tp1-active-passive
feature/tp1-composition
feature/tp1-python-validation
```

### PFI

```text
feature/h<n>-<descripcion>
```

Ejemplos:

```text
feature/h1-urdf-model
feature/h1-forward-kinematics
feature/h1-validation
feature/h2-inverse-kinematics
feature/h2-jacobian
feature/h2-lspb-trajectory
feature/h3-camera-calibration
feature/h3-hand-eye-calibration
feature/h3-vision-segmentation
feature/h4-rapid-control
feature/h4-ros-industrial
feature/h4-safety-fsm
```

### Reglas de nombres de ramas

Se utilizará inglés técnico, minúsculas ASCII y palabras separadas por guiones. Se evitarán espacios, tildes, `ñ`, guiones bajos, nombres de integrantes, fechas por defecto y sufijos ambiguos como `final`, `final2` o `nuevo`.

Antes de crear una rama se verifica que no exista ya otra rama para el mismo trabajo.

No se añadirán inicialmente familias como `release/*`, `hotfix/*` o `docs/*`; el equipo se mantendrá dentro del modelo `main`, `dev` y `feature/*` exigido por la cátedra.

## 2. Conventional Commits

Formato:

```text
<type>(<scope>): <descripcion>
```

El `type` y el `scope` se escriben en inglés técnico. La descripción se escribe en español, comienza normalmente en minúscula, no termina en punto y debe ser concreta.

Ejemplos:

```text
feat(tp1): implementa matrices de rotacion elementales
test(tp1): verifica ortogonalidad de matrices
docs(tp1): agrega desarrollo de composicion de rotaciones
fix(kinematics): corrige offset de la tercera junta
```

### Types iniciales

| Type | Uso |
|---|---|
| `feat` | Nueva funcionalidad o implementación. |
| `fix` | Corrección de un defecto. |
| `docs` | Documentación. |
| `test` | Pruebas o validaciones. |
| `refactor` | Reorganización interna sin cambio funcional esperado. |
| `chore` | Mantenimiento técnico que no encaja en los anteriores. |
| `ci` | Automatización de integración continua. |

### Scopes iniciales

```text
repo
tp1
kinematics
urdf
ros
trajectory
dynamics
vision
calibration
rapid
safety
docs
ci
```

La lista puede ampliarse cuando aparezca un módulo real que lo justifique. Para el PFI se prefieren scopes técnicos del módulo antes que scopes genéricos como `h1` o `h2`.

### Commits no aceptables

Se evitarán mensajes vagos como:

```text
avance
cambios
final
listo
update
```

No existe cuota fija de commits. Se exige coherencia lógica, no microcommits artificiales. Un commit debe representar una unidad de cambio comprensible y, cuando sea razonable, mantener el proyecto en un estado funcional.

La primera línea debería mantenerse breve y específica, idealmente alrededor de 72 caracteres cuando sea posible.

## 3. Autoría

Cada integrante configura su propia identidad Git y utiliza su propia cuenta de GitHub. Cada commit corresponde al autor real del trabajo.

No se utilizarán trailers `Co-authored-by` como práctica habitual. La colaboración se refleja mediante Issues, historial de commits, Pull Requests y reviews.

Una tarea puede cambiar de responsable sin cambiar de rama; el historial de commits conserva la autoría real de cada contribución.

## 4. Títulos de Pull Requests

Cuando resulte natural, el título de un PR seguirá la misma estructura conceptual de Conventional Commits:

```text
feat(tp1): implementa validaciones numericas
```

El cuerpo del PR debe indicar qué cambió, por qué, cómo se verificó y qué Issue está relacionado.

## 5. Nombres de archivos y directorios

Dentro del espacio del equipo se utilizarán minúsculas, sin tildes ni `ñ`, sin espacios y con guiones para separar palabras en nombres de directorios técnicos.

Ejemplos:

```text
practicas/tp1-rotation/
pfi/
```

Los nombres convencionales de documentación pueden mantenerse en mayúsculas:

```text
README.md
GIT_WORKFLOW.md
PROJECT_STATUS.md
```

Para TP1, el nombre propuesto del notebook es:

```text
tp1_rotation.ipynb
```

Los nombres existentes del docente no se renombran para adaptarlos a nuestras convenciones.

## 6. Idioma

La documentación humana se escribe en español. Los términos técnicos estándar y los identificadores se conservan en inglés cuando sea natural: `branch`, `commit`, `merge`, `Pull Request`, `origin`, `upstream`, `feature`, ROS, APIs y nombres de módulos.

Los mensajes de commit combinan `type` y `scope` en inglés con descripción en español.

## 7. Estados de tareas

El GitHub Project utilizará los estados:

```text
Pendiente
En progreso
En revisión
Bloqueada
Terminada
```

`Reasignada` no es un estado. Una reasignación se representa cambiando el Assignee y registrando el motivo cuando sea relevante.

## 8. Documentación técnica

Los documentos del equipo deben ser concisos, jerárquicos y explícitos. No se utilizarán checkboxes de progreso dentro de los documentos de referencia; el seguimiento operativo pertenece a GitHub Issues y GitHub Project.

No se duplicará una misma política completa en varios archivos. Cada tema tendrá una fuente de verdad definida.
