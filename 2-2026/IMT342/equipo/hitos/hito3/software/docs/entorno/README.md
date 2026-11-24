# Entorno de ejecución y reproducción offline

Entorno de referencia: Fedora Workstation 44, Podman rootless, ROS 2 Jazzy en Ubuntu Noble. El script de compilación utiliza por defecto `localhost/ros-jazzy-imt342-rws:latest`; el nombre local no es una imagen publicada en un registro.

| Imagen presente en el host de referencia | ID local observado |
| --- | --- |
| `docker.io/osrf/ros:jazzy-desktop-full-noble` | `db0ff3f9fbde93b873a94caf86299911a5905d10a74fb527646526593ca5894c` |
| `localhost/ros-jazzy-imt342-rws:latest` | `cac7217067eef59c99735a16cbe991eb1c95a8b5d6426c979ff7ccaeb12a7e69` |

La primera imagen se utiliza como referencia de comparación, no como prueba de parentesco entre capas. [Inventario dpkg](paquetes-dpkg.tsv) registra versiones de paquetes de la imagen operativa; [diferencias](diferencias-dpkg.tsv) identifica paquetes añadidos, retirados o con versión distinta respecto de la imagen de referencia. Los inventarios se obtuvieron en contenedores desechables, sin red ni montaje de directorios del host. No constituyen por sí mismos un Containerfile ni registran cambios ajenos a dpkg. La disponibilidad futura de versiones exactas no está garantizada.

Desde `software/`, con una imagen local que satisfaga las dependencias del proyecto:

```bash
IMT342_JAZZY_IMAGE=localhost/ros-jazzy-imt342-rws:latest ./scripts/build_jazzy_offline.sh
./scripts/imt342_launcher.sh
```

El primer script compila los seis paquetes y ejecuta Colcon test y test-result. Usa `--pull=never`, `--network=none`, UID/GID del usuario, y monta los módulos RAPID en solo lectura. El entorno gráfico y Zenity se requieren para el lanzador. Para reproducir en otro host se necesita preparar una imagen equivalente y comprobar allí la compilación y las pruebas; no se acredita una reconstrucción limpia de la imagen a partir de la comparación de paquetes.

No se versionan imágenes, credenciales, parámetros privados del IRC5 ni directorios generados. Para la modalidad offline no se requiere conexión al controlador. La modalidad física exige autorización y el procedimiento del laboratorio.
