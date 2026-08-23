# IMT-342 — Robótica

Espacio de trabajo del equipo para la asignatura **IMT-342 Robótica**, gestión II-2026, de la Universidad Católica Boliviana "San Pablo" — Sede Tarija.

El material oficial de la asignatura se mantiene en las carpetas publicadas por el docente dentro de `2-2026/IMT342/`. Los archivos desarrollados por el equipo se organizan por separado en `2-2026/IMT342/equipo/`.

## Equipo

- **Roberth Williams Ruiz Condori** — GitHub: `@RWROBERTH`
- **Mario Alberto Nina Gallo** — GitHub: `@mario-nina`

## Estructura

```text
equipo/
├── README.md
├── docs/
│   └── GIT_WORKFLOW.md
├── templates/
│   └── tp_template.ipynb
└── practicas/
    └── ...
```

- `practicas/`: trabajos prácticos desarrollados durante la asignatura.
- `templates/`: archivos base utilizados para mantener una presentación común en los nuevos prácticos.
- `docs/`: documentación complementaria sobre el flujo de trabajo del equipo.

La estructura se amplía únicamente cuando una práctica o actividad realmente lo necesita.

## Prácticos

Cada trabajo práctico se desarrolla en su propia rama `feature/tpN-*`, creada normalmente a partir de `dev` actualizado.

Los prácticos finalizados se conservan en sus respectivas ramas de entrega como evidencia del estado en que fueron desarrollados.

No se actualizan posteriormente únicamente por cambios generales en la documentación, las plantillas o la organización del repositorio. Por este motivo, `dev` funciona como base para los trabajos nuevos y no necesariamente contiene una colección acumulada de todos los prácticos realizados.

### Estructura común

Los nuevos prácticos utilizan como referencia la siguiente estructura:

```text
practicas/
└── tpN-descripcion/
    ├── tpN_descripcion.ipynb
    └── evidencias/
        └── ...
```

El notebook parte de `templates/tp_template.ipynb` y se adapta a la consigna correspondiente.

Las evidencias manuscritas siguen la convención:

```text
ejercicio<n>_hoja_<m>.jpg
```

Las entregas anteriores se conservan tal como fueron realizadas y no se modifican retroactivamente para ajustarlas a convenciones posteriores.

## Ramas

Se mantienen las familias de ramas utilizadas en la asignatura:

- `main`: rama estable y punto de incorporación de las actualizaciones oficiales del repositorio del docente.
- `dev`: base común para iniciar nuevos trabajos del equipo.
- `feature/*`: ramas de desarrollo para prácticos y otras actividades.

El procedimiento para sincronizar el repositorio, crear ramas y trabajar con las entregas se encuentra en `docs/GIT_WORKFLOW.md`.

## Convenciones

Los commits utilizan **Conventional Commits** y deben representar unidades lógicas de trabajo.

Formato general:

```text
<type>(<scope>): <descripcion>
```

La documentación del equipo se escribe en español, manteniendo los términos técnicos en inglés cuando resulte natural.

Las instrucciones explícitas del docente tienen prioridad sobre cualquier convención interna del equipo.
