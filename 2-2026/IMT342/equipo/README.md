# Equipo IMT-342 — Robótica

Este directorio es el espacio de trabajo del equipo dentro del repositorio académico `planificacion-UCB`, gestión II-2026, asignatura IMT-342 Robótica de la Universidad Católica Boliviana "San Pablo" — Sede Tarija.

## Alcance

El repositorio del docente es la fuente oficial de material, consignas, rúbricas y automatización académica. El equipo trabajará únicamente dentro de `2-2026/IMT342` y no modificará, eliminará ni reorganizará contenido de otras materias.

El repositorio canónico de colaboración del equipo es el fork `mario-nina/planificacion-UCB`. Dentro de IMT-342, el trabajo producido por el equipo se concentrará bajo `2-2026/IMT342/equipo/` para mantenerlo separado del material oficial y reducir conflictos con futuras actualizaciones del docente.

## Organización del espacio del equipo

```text
2-2026/IMT342/equipo/
├── README.md
├── docs/
│   ├── GIT_WORKFLOW.md
│   ├── CONVENTIONS.md
│   ├── PROJECT_CONTEXT.md
│   ├── PROJECT_STATUS.md
│   └── DECISIONS.md
├── practicas/
│   └── ...
└── pfi/
    └── ...
```

Las carpetas `practicas/` y `pfi/` se crearán cuando exista contenido real que versionar. No se crearán directorios vacíos sólo para anticipar una estructura futura.

## Modelo de trabajo

`main` representa el estado estable del PFI producido por el equipo y también puede recibir actualizaciones oficiales provenientes de `upstream`. `dev` es la rama de integración y el punto de partida normal del desarrollo. Las ramas `feature/*` son las ramas de trabajo.

Toda `feature/*` de primer nivel nace normalmente desde `dev`, salvo instrucción oficial expresa en sentido contrario. Las ramas auxiliares de una entrega académica pueden nacer de la rama de entrega correspondiente.

Las prácticas académicas terminan en su rama de entrega y se conservan. Las features del PFI se integran a `dev`, y los estados estables del PFI pasan posteriormente de `dev` a `main`.

## Documentación

| Documento | Función |
|---|---|
| `docs/GIT_WORKFLOW.md` | Procedimiento operativo de Git, GitHub, ramas, tareas, PR, sincronización y conflictos. |
| `docs/CONVENTIONS.md` | Convenciones de nombres, commits, idioma y formato. |
| `docs/PROJECT_CONTEXT.md` | Contexto estable de la materia, repositorio, TP1 y PFI. |
| `docs/PROJECT_STATUS.md` | Estado actual del trabajo y próximos pasos. |
| `docs/DECISIONS.md` | Registro de decisiones importantes, motivos y consecuencias. |

La documentación humana se escribe en español. Los términos técnicos estándar de Git, GitHub, ROS y programación se conservan en inglés cuando resulte natural.

## Autoría y colaboración

Cada integrante utilizará su propia cuenta de GitHub, identidad Git y credenciales. No se comparten contraseñas, tokens ni claves SSH. Cada commit debe corresponder al autor real del cambio.

No se empleará coautoría mediante trailers `Co-authored-by` como práctica habitual. La trazabilidad se obtiene mediante commits individuales, Issues, ramas, Pull Requests y reviews.

Las tareas se asignan por unidades lógicas de trabajo y no por propiedad permanente de módulos. Si una tarea debe reasignarse, otro integrante puede continuar sobre la misma rama, preservando el historial de autores y commits.

## Precedencia

Una instrucción explícita del docente prevalece sobre cualquier convención interna de este directorio. Si aparece una contradicción, se registra el ajuste en `docs/DECISIONS.md` y se actualiza la documentación operativa correspondiente.
