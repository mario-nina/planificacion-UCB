# Instalación y operación

## Entorno

El entorno de referencia es Fedora Workstation 44 con Podman rootless y ROS 2 Jazzy (Ubuntu Noble). La imagen y sus parámetros efectivos se definen en los scripts de la aplicación. No se distribuyen imágenes de contenedor ni credenciales. La reproducción en un host limpio requiere disponer de la imagen y las dependencias utilizadas por dichos scripts; no se afirma haber probado una instalación nueva desde cero.

Desde `software/`, para compilación y pruebas offline:

```bash
./scripts/build_jazzy_offline.sh
```

Para iniciar el selector gráfico:

```bash
./scripts/imt342_launcher.sh
```

El lanzador requiere el entorno gráfico de Fedora y Zenity. La modalidad offline utiliza red aislada. Los modos Solo lectura y Control físico utilizan la configuración externa del laboratorio; las credenciales se gestionan mediante Secret Service, no mediante archivos versionados.

## Interfaz

FK permite definir una configuración articular y examinar la pose. IK permite introducir una pose y seleccionar una rama. Tareas reúne el catálogo y las secuencias personalizadas. Diagnóstico y análisis muestran estados, gráficas y procedencia de datos. La previsualización no es feedback físico.

El acceso al controlador y cualquier prueba física requieren el procedimiento institucional de laboratorio, autorización aplicable y verificación de la instalación. La publicación no implica instalación o sustitución de módulos en el IRC5.
