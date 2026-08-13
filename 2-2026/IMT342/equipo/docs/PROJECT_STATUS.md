# Estado actual — IMT-342

Última actualización documental: 13 de agosto de 2026.

## Resumen

El diseño conceptual del workflow Git/GitHub y de la organización documental está completo. La configuración base de Git/GitHub en Fedora, el fork, los remotos y la rama `dev` ya fueron implementados. El repositorio se encuentra actualmente en la primera feature real de inicialización documental.

| Área | Estado actual |
|---|---|
| Arquitectura y propiedad del repositorio | Definida. |
| Identidad, responsabilidades y checkpoints | Definidos. |
| Modelo de ramas | Definido. |
| Nombres de ramas | Definidos. |
| Política de commits | Definida. |
| Flujo cotidiano | Definido. |
| Pull Requests, review y merge | Definidos. |
| Sincronización y conflictos | Definidos a nivel de política. |
| Estructura interna del espacio del equipo | Definida. |
| Issues, Milestones y GitHub Project | Modelo definido. |
| Protecciones y configuración objetivo de GitHub | Definidas; aplicación real pendiente. |
| Arquitectura documental | Definida; documentos actualizados y pendientes de versionar. |
| Git en Fedora | Configurado. |
| GitHub CLI | Instalado y autenticado. |
| SSH con GitHub | Configurado y verificado. |
| Fork `mario-nina/planificacion-UCB` | Creado y clonado. |
| `origin` / `upstream` | Configurados y verificados. |
| Rama `dev` | Creada desde `main` y publicada en `origin`. |
| Issues del fork | Habilitados. |
| Issue de inicialización | `#1` creado. |
| Feature de inicialización | `feature/repo-setup` creada desde `dev` y activa. |
| GitHub Project / Milestones | Pendientes de creación/configuración real. |
| Protecciones de `main` y `dev` | Pendientes de aplicación real. |
| Colaboradores del equipo | Pendientes de incorporación. |
| TP1 | No iniciado técnicamente. |
| PFI | Sin progreso técnico. |

## Configuración verificada

En el equipo de Mario se verificó:

```text
Fedora 44 Workstation
Git 2.55.0
GitHub CLI 2.97.0
GitHub: mario-nina
Git protocol: SSH
origin:   git@github.com:mario-nina/planificacion-UCB.git
upstream: git@github.com:berquiroga/planificacion-UCB.git
remote.pushDefault: origin
```

La autenticación SSH contra `git@github.com` fue probada correctamente.

Al crear `dev`, las referencias `main`, `dev`, `origin/main`, `origin/dev` y `upstream/main` apuntaban al mismo commit `55c8f83`, por lo que la rama de integración se creó desde una base completamente alineada.

## Estado de la inicialización documental

Se habilitaron Issues en el fork y se creó:

```text
Issue #1 — Inicializar documentación y estructura del equipo
```

La rama activa para esta tarea es:

```text
feature/repo-setup
```

Los documentos `README.md`, `GIT_WORKFLOW.md`, `CONVENTIONS.md`, `PROJECT_CONTEXT.md`, `PROJECT_STATUS.md` y `DECISIONS.md` están siendo revisados antes de incorporarse al repositorio. Todavía no se ha realizado el primer commit de esta feature.

La inspección inicial no detectó archivos bajo `.github/` en el checkout actual. El README oficial, sin embargo, indica que el tracking académico utiliza GitHub Actions; por ello cualquier automatización oficial que aparezca posteriormente se tratará como material que no debe modificarse sin análisis previo.

## Situación de TP1

La guía oficial fija como entrega un Jupyter Notebook en la rama `feature/tp1-rotation`, con fecha límite jueves 13 de agosto de 2026 a las 16:00.

El trabajo técnico aún no se ha iniciado. Dado que la entrega corresponde al mismo día de esta actualización, la inicialización restante debe mantenerse mínima: versionar la documentación, completar sólo la configuración GitHub imprescindible y pasar de inmediato a la rama oficial de TP1.

## Próximos pasos operativos

1. Incorporar y revisar la documentación en `feature/repo-setup`.
2. Crear commits lógicos de documentación y publicar la feature.
3. Abrir Pull Request hacia `dev` y verificar el flujo de revisión/merge.
4. Aplicar la configuración mínima necesaria de `main` y `dev` sin interferir con el trabajo urgente de TP1.
5. Crear `feature/tp1-rotation` desde `dev` actualizado.
6. Crear el seguimiento operativo de TP1 y comenzar la práctica.
7. Incorporar a los demás integrantes como colaboradores cuando corresponda y registrar sus identidades Git individuales.

## Datos todavía abiertos

El mecanismo concreto para incorporar `upstream/main` a una `main` protegida se cerrará después de probar las protecciones reales del fork.

La estructura interna del PFI permanece deliberadamente abierta. Se definirá cuando comience el Hito 1 y existan necesidades reales de paquetes, código, URDF, pruebas y recursos.

## Criterio para actualizar este archivo

`PROJECT_STATUS.md` contiene únicamente información que cambia con el tiempo. Las políticas permanentes pertenecen a `GIT_WORKFLOW.md` y `CONVENTIONS.md`; los motivos históricos pertenecen a `DECISIONS.md`.
