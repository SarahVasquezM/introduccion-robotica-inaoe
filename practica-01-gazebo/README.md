# Práctica 1 — Introducción a Gazebo y al dron

## Objetivo

Preparar el entorno de simulación, iniciar un mundo de Gazebo con el cuadrotor `sjtu_drone` y reconocer nodos, tópicos, mensajes y comandos de movimiento en ROS 2.

## Descripción

La práctica introduce el flujo de trabajo del curso: compilar el workspace, cargar el entorno y ejecutar el lanzamiento que incorpora el mundo, el dron y sus sensores. `sjtu_drone` aporta el modelo, los controladores y las cámaras frontal e inferior. Cualquier mundo modificado debe conservarse en `mundos/`.

## Ejecución básica

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch sjtu_drone_bringup sjtu_drone_gazebo.launch.py
```

En otra terminal, el despegue puede solicitarse con:

```bash
ros2 topic pub --once /simple_drone/takeoff std_msgs/msg/Empty "{}"
```

## Elementos observados

- Mundo físico de Gazebo y modelo del cuadrotor.
- Comandos de velocidad en `/simple_drone/cmd_vel`.
- Pose del dron en `/simple_drone/gt_pose`.
- Cámara frontal en `/simple_drone/front/image_raw`.
- Cámara inferior en `/simple_drone/bottom/image_raw`.
- Comandos de despegue y aterrizaje.

## Comprobaciones sugeridas

```bash
ros2 node list
ros2 topic list
ros2 topic echo /simple_drone/gt_pose
ros2 topic hz /simple_drone/bottom/image_raw
```

La cámara puede visualizarse con `rqt_image_view` o RViz2. Esta práctica sirve como base para las prácticas posteriores.

## Organización esperada

- `src/`: nodos o paquetes desarrollados.
- `launch/`: archivos de lanzamiento propios.
- `mundos/`: mundos o modelos modificados.
- `resultados/`: capturas representativas.
- `reporte/`: reporte y fuentes LaTeX.
