# Contexto del proyecto — IMT-342

## 1. Materia

Asignatura: Robótica, código IMT-342. Gestión: II-2026. Universidad Católica Boliviana "San Pablo" — Sede Tarija, carrera de Ingeniería Mecatrónica.

Repositorio oficial utilizado por la materia: `berquiroga/planificacion-UCB`, dentro de `2-2026/IMT342`.

## 2. Requisitos oficiales de control de versiones

El README oficial del PFI establece que cada equipo debe realizar un fork del repositorio. El seguimiento académico se apoya en commits y GitHub Actions. Se exige Conventional Commits y se establecen como ramas obligatorias `main`, `dev` y `feature/*`.

El mismo README advierte que los commits masivos realizados de una sola vez serán penalizados y no contabilizarán horas. La política interna del equipo de commits lógicos y frecuentes responde directamente a este requisito.

Según el material oficial, `main` representa código estable probado en el robot real, `dev` es la rama de integración continua y `feature/*` se utiliza para desarrollo.

## 3. Fork y alcance de trabajo

El fork canónico del equipo es `mario-nina/planificacion-UCB`. GitHub realiza forks de repositorios completos, no de carpetas individuales, por lo que el fork incluye también materias ajenas a IMT-342.

El equipo trabaja únicamente en `2-2026/IMT342` y no modifica, elimina ni reorganiza materias ajenas. Dentro de la asignatura, el trabajo propio se concentra bajo `2-2026/IMT342/equipo/` para separar material oficial y producción del equipo.

## 4. Remotos

La configuración verificada del clon de trabajo es:

```text
origin   → git@github.com:mario-nina/planificacion-UCB.git
upstream → git@github.com:berquiroga/planificacion-UCB.git
```

`origin` es el remoto de colaboración y publicación normal. `upstream` es una convención interna para mantener el fork sincronizable con futuras publicaciones del docente; no se presenta como un requisito explícito de la materia.

## 5. Guía Práctica 1

La Hoja de Trabajo Práctico N.º 1 trata Geometría de Rotación, Dualidad de SO(3) y Composición.

La guía oficial exige:

| Área | Requisito |
|---|---|
| Fundamentación teórica | Demostración matemática manuscrita paso a paso. |
| Implementación | Script modular en Python con NumPy. |
| Validación | Isometría y ortogonalidad de matrices de rotación. |
| Evidencia | Jupyter Notebook en el repositorio Git del equipo. |
| Rama de entrega | `feature/tp1-rotation`. |
| Fecha límite indicada | Jueves 13 de agosto de 2026, 16:00. |

Los ejercicios obligatorios son: dualidad activa/pasiva, composición con ejes fijos y móviles, e implementación/validación numérica en Python.

La guía solicita las funciones `rot_x(deg)`, `rot_y(deg)` y `rot_z(deg)`, el cálculo de la matriz final del Ejercicio 2, la verificación `||R^T R - I||_F < 10^-15` y la conservación de la norma para `p_B = [2, -1, 4]^T`.

El documento oficial menciona Ubuntu para la ejecución de la práctica. La configuración Git inicial del equipo se está realizando en Fedora 44 Workstation; la decisión sobre el entorno efectivo de ejecución de TP1 se tratará al iniciar la práctica y no se altera silenciosamente en esta documentación.

## 6. Proyecto Final Integrador

El PFI oficial se titula "Celda de Manufactura Flexible Inteligente: Clasificación y Paletizado Autónomo por Visión Artificial sobre el ABB IRB 120".

El material oficial divide el proyecto en cuatro hitos:

| Hito | Contenido principal |
|---|---|
| Hito 1 | Gemelo digital URDF y cinemática directa. |
| Hito 2 | Cinemática inversa mediante Pieper y perfiles LSPB. |
| Hito 3 | Localización Hand-Eye/SVD y segmentación OpenCV. |
| Hito 4 | Integración física, ROS-Industrial, RAPID, FSM y artículo IEEE. |

El detalle técnico, KPIs, restricciones anti-caja-negra y requisitos de seguridad permanece en la guía oficial del PFI y no se replica por completo aquí.

## 7. Modelo de Git

La arquitectura acordada es:

```text
main
 └── dev
      └── feature/*
```

`dev` fue creado inicialmente desde `main`. Toda feature de primer nivel nace normalmente desde `dev`. Una rama auxiliar de una práctica puede nacer de la feature principal de esa práctica.

Las prácticas terminan en su rama académica de entrega y se preservan. Las features del PFI regresan a `dev` mediante Pull Request; los estados estables del PFI pasan posteriormente de `dev` a `main`.

## 8. Equipo y forma de trabajo

El equipo está compuesto por tres integrantes. Cada integrante utilizará cuenta de GitHub, identidad Git y credenciales propias. No habrá propietarios permanentes de módulos. Las responsabilidades se asignan por tareas lógicas y pueden cambiar cuando un bloqueo o deadline lo requiera.

La identidad Git actualmente configurada en el equipo de Mario es:

```text
GitHub:    mario-nina
user.name: mario-nina
user.email: malber5824@gmail.com
```

Los datos de los demás integrantes se registrarán cuando se incorporen al repositorio y configuren sus propias identidades.

## 9. Entorno actual

La preparación de Git y GitHub se realiza desde Fedora 44 Workstation y terminal. Actualmente están instalados y operativos Git 2.55.0, OpenSSH y GitHub CLI 2.97.0. La autenticación con GitHub está configurada por SSH mediante una clave Ed25519 registrada en la cuenta `mario-nina`.

El material oficial del PFI especifica Ubuntu 24.04 LTS como entorno de desarrollo autorizado para el PFI; esa diferencia se mantiene explícita y se resolverá cuando corresponda al trabajo técnico del PFI, no durante la preparación inicial de Git.

## 10. Fuentes oficiales revisadas

Esta documentación se apoya en los archivos oficiales disponibles en el proyecto, especialmente:

```text
README.md
01_guia.tex
Guia_PFI.tex
IMT-342-Robotica.pdf
```

Ante una discrepancia entre esta documentación y una instrucción oficial posterior, prevalece la instrucción del docente y se registra el cambio en `DECISIONS.md`.
