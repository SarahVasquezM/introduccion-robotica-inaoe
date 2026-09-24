# Introducción a la Robótica — INAOE

Repositorio de prácticas de la materia **Introducción a la Robótica** de la Maestría en Ciencias Computacionales del INAOE.

Las prácticas utilizan ROS 2 Humble y Gazebo para simular un cuadrotor. Se trabaja con `sjtu_drone`, control PID, acciones de ROS 2, visión con YOLO y generación de trayectorias.

## Prácticas

| Práctica | Tema principal |
|---|---|
| [Práctica 1](practica-01-gazebo/README.md) | Preparación de Gazebo, mundo de simulación y familiarización con el dron. |
| [Práctica 2](practica-02-control-pid/README.md) | Control PID de posición y yaw mediante una acción de ROS 2. |
| [Práctica 3](practica-03-seguimiento-yolo/README.md) | Detección y seguimiento de una persona con YOLO y una máquina de estados. |
| [Práctica 4](practica-04-trayectorias/README.md) | Generación de trayectorias cúbicas con un tiempo deseado de llegada. |

## Organización

Cada práctica separa el código fuente, archivos de lanzamiento, scripts de análisis, resultados y reporte. Las subcarpetas se incorporarán conforme se agreguen archivos reales.

```text
practica-XX/
├── README.md
├── src/          # Paquetes o nodos desarrollados
├── launch/       # Archivos de lanzamiento, cuando apliquen
├── scripts/      # Análisis, gráficas y utilidades
├── mundos/       # Mundos y modelos de Gazebo, cuando apliquen
├── resultados/   # Imágenes y gráficas seleccionadas
└── reporte/      # Reporte y fuentes LaTeX
```

Consulta [dependencias](docs/dependencias.md) y [referencias](docs/referencias.md) antes de reproducir una práctica.

## Recomendaciones

- No subir `build/`, `install/`, `log/` ni entornos virtuales.
- Mantener fuera de Git los rosbag y videos pesados; pueden enlazarse desde el README correspondiente.
- Registrar la versión o el commit de cada dependencia externa.
- Ejecutar los comandos desde un workspace de ROS 2 y cargar ROS 2 y el workspace antes de cada prueba.

## Estado

La estructura documental está preparada. El código, los reportes y los resultados de cada práctica se incorporarán en sus carpetas correspondientes.
