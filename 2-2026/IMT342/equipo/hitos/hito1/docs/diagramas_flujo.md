# Diagramas de flujo — Hito 1 ABB IRB 120

Este documento resume únicamente los procesos técnicos del Hito 1.

---

## 1. Flujo general del Hito 1

```mermaid
flowchart TB
    A[Geometría del ABB IRB 120]

    A --> B[Asignación de marcos D-H]
    B --> C[Tabla D-H]
    C --> D[Cinemática directa en Python]
    D --> E[Pose calculada]

    A --> F[Modelo URDF]
    F --> G[Estados articulares]
    G --> H[robot_state_publisher]
    H --> I[Árbol TF]

    E --> J[Validación cruzada]
    I --> J

    J --> K[Error de posición y orientación]
    K --> L{¿Cumple el criterio?}
    L -->|Sí| M[PASS]
    L -->|No| N[FAIL]
```

---

## 2. Flujo de cinemática directa D-H

```mermaid
flowchart TB
    A[Vector articular q]
    --> B[Aplicar offsets articulares]
    --> C[Construir parámetros D-H]
    --> D[Calcular matrices individuales]
    --> E[Multiplicar la cadena homogénea]
    --> F[Obtener 0T6]
    --> G[Aplicar transformación fija al TCP]
    --> H[Obtener 0Ttcp]
```

---

## 3. Flujo URDF / ROS

```mermaid
flowchart TB
    A[Vector articular q]
    --> B[Publicar /joint_states]
    --> C[robot_state_publisher]

    D[URDF del ABB IRB 120]
    --> C

    C --> E[Árbol TF]

    E --> F[base_link a dh_frame_6]
    E --> G[base_link a tcp]
    E --> H[Visualización en RViz]
```

---

## 4. Flujo de validación FK vs ROS

```mermaid
flowchart TB
    A[Seleccionar configuración q]

    A --> B[Calcular FK D-H en Python]
    A --> C[Publicar la misma q en ROS]

    B --> D[Pose FK]
    C --> E[Pose desde TF]

    D --> F[Comparar posición]
    E --> F

    D --> G[Comparar orientación]
    E --> G

    F --> H[Calcular Delta E]
    G --> I[Calcular error de rotación]

    H --> J{Delta E < 1e-6 m}
    I --> J

    J -->|Sí| K[PASS]
    J -->|No| L[FAIL]
```
