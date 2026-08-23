# Flujo de trabajo Git y GitHub — IMT-342

Este documento reúne el flujo de trabajo utilizado por el equipo para mantener el repositorio actualizado y desarrollar los prácticos de **IMT-342 Robótica**.

La asignatura establece el uso de `main`, `dev`, `feature/*` y Conventional Commits. Los procedimientos restantes corresponden a la organización interna del equipo.

Las instrucciones explícitas del docente tienen prioridad sobre este documento.

## 1. Repositorio y ramas

En los clones de trabajo se utilizan dos remotos:

```text
origin   → fork de trabajo del equipo
upstream → repositorio oficial del docente
```

- `origin` se utiliza para compartir y publicar el trabajo del equipo.
- `upstream` se utiliza para consultar las actualizaciones oficiales.

Las ramas se utilizan de la siguiente manera:

- `main`: rama estable y punto de incorporación de las actualizaciones oficiales.
- `dev`: base común para iniciar nuevos trabajos.
- `feature/*`: ramas de desarrollo.

Cada práctico utiliza una rama:

```text
feature/tpN-descripcion
```

Las ramas de prácticos finalizados se conservan como evidencia de la entrega y no se actualizan por cambios generales posteriores.

## 2. Actualizaciones del docente

Antes de integrar novedades se comprueba el estado actual:

```bash
git status
git branch --show-current
```

Descargar las referencias nuevas:

```bash
git fetch upstream --prune
```

Revisar qué cambió:

```bash
git log --oneline main..upstream/main
git diff --name-status main...upstream/main
git diff --stat main...upstream/main
```

Una vez revisadas las novedades, actualizar `main`:

```bash
git switch main
git status
git merge --ff-only upstream/main
git push origin main
```

Después actualizar `dev`:

```bash
git switch dev
git status
git merge main
git push origin dev
```

No se utiliza `git pull upstream main` sin revisar previamente los cambios.

Las ramas de prácticos ya entregados no se actualizan durante este proceso.

## 3. Iniciar un práctico

Partir de `dev` actualizado:

```bash
git switch dev
git status
git pull --ff-only origin dev
```

Crear la nueva rama:

```bash
git switch -c feature/tpN-descripcion
```

Publicarla para que quede disponible para el equipo:

```bash
git push -u origin feature/tpN-descripcion
```

Crear la estructura inicial:

```text
practicas/
└── tpN-descripcion/
    ├── tpN_descripcion.ipynb
    └── evidencias/
        └── .gitkeep
```

El notebook se crea a partir de:

```text
templates/tp_template.ipynb
```

`.gitkeep` permite conservar `evidencias/` mientras la carpeta todavía está vacía y puede eliminarse al agregar la primera evidencia real.

No se crean otras carpetas hasta que la práctica realmente las necesite.

Una vez preparada la estructura inicial:

```bash
git status
git add 2-2026/IMT342/equipo/practicas/tpN-descripcion
git commit -m "chore(tpN): crea estructura inicial de la practica"
git push
```

## 4. Desarrollo y cierre

Los commits utilizan el formato:

```text
<type>(<scope>): <descripcion>
```

Cada commit debe representar un cambio lógico del trabajo.

No existe una cantidad fija de commits. Se evitan tanto los commits masivos al finalizar la práctica como los microcommits sin contenido significativo.

Antes de registrar cambios:

```bash
git status
git diff
```

Después de preparar los archivos:

```bash
git diff --staged
```

El progreso se publica regularmente en la misma rama:

```bash
git push
```

### Jupyter

Los notebooks `.ipynb` requieren especial cuidado cuando ambos integrantes trabajan sobre el mismo archivo, porque los conflictos pueden ser difíciles de resolver.

Se procura coordinar la edición del notebook y evitar modificar simultáneamente las mismas celdas.

Los checkpoints de Jupyter ya están excluidos por el `.gitignore` oficial:

```text
.ipynb_checkpoints/
```

### Evidencias

Las evidencias manuscritas se almacenan en:

```text
evidencias/
```

Para los nuevos prácticos se utiliza:

```text
ejercicio<n>_hoja_<m>.jpg
```

Las prácticas anteriores conservan los nombres utilizados originalmente.

### Antes de entregar

Comprobar:

- que se está en la rama correcta;
- que el notebook y las evidencias necesarias están versionados;
- que las celdas requeridas ejecutan correctamente;
- que no existen archivos temporales accidentales;
- que todos los cambios necesarios fueron publicados.

Como comprobación final:

```bash
git status
git log --oneline --decorate
```

Después de la entrega, la rama se conserva tal como quedó.

Sólo se vuelve a modificar si existe una razón académica concreta relacionada con ese práctico.

## 5. Precauciones

No utilizar `git push --force` sobre ramas compartidas o de entrega.

Evitar reescribir historia ya publicada mediante `rebase`, `commit --amend` o `reset` cuando otros integrantes puedan depender de esos commits.

No utilizar como solución automática:

```bash
git reset --hard
git clean -fd
```

Si aparece un conflicto, se revisan ambos cambios antes de decidir qué conservar.

Los conflictos en notebooks requieren especial cuidado y no deben resolverse automáticamente sin revisar el contenido.

Si el estado del repositorio no está claro, se revisa antes de continuar con una operación que pueda modificar el historial.
