# Dependencias

## Entorno base

- Ubuntu compatible con ROS 2 Humble.
- ROS 2 Humble.
- Gazebo y complementos de ROS 2 para simulación.
- `colcon` para compilar el workspace.
- Python 3 para nodos, control y análisis.

## Simulación del dron

Las prácticas emplean [`sjtu_drone`](https://github.com/NovoG93/sjtu_drone), que proporciona el modelo del cuadrotor, cámaras, control y archivos de lanzamiento para Gazebo.

El mundo y el dron se inician normalmente mediante `sjtu_drone_bringup`. Si se utiliza un mundo externo o modificado, debe guardarse en `mundos/` y documentarse con su repositorio y commit.

## Dependencias por práctica

### Prácticas 1, 2 y 4

- `rclpy`, `geometry_msgs` y `std_msgs`.
- Interfaces de acciones de ROS 2.
- NumPy y Matplotlib para análisis y gráficas.

### Práctica 3

- Ultralytics YOLO con el modelo preentrenado YOLO11n.
- Mensajes de detección usados por el nodo de YOLO.
- `rqt_image_view` o RViz2 para observar las cámaras.
- Pandas, NumPy y Matplotlib para procesar el registro CSV.

## Registro de versiones

| Dependencia | Versión o commit |
|---|---|
| ROS 2 | Humble |
| Gazebo | Pendiente de registrar |
| `sjtu_drone` | Pendiente de registrar |
| Ultralytics | Pendiente de registrar |
| Modelo YOLO | YOLO11n preentrenado |
