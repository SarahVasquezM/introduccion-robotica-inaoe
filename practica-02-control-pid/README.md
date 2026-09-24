# Práctica 2 — Control PID de posición y yaw

## Objetivo

Implementar un servidor de acciones de ROS 2 que controle la posición tridimensional y la orientación yaw del cuadrotor mediante un controlador PID.

## Descripción

El paquete `control_action` recibe un objetivo con `xd`, `yd`, `zd` y `wd`. `wd` representa el yaw deseado en radianes. La pose real se obtiene de `/simple_drone/gt_pose` y las velocidades se publican en `/simple_drone/cmd_vel`.

La implementación considera:

- Errores de posición en `x`, `y` y `z`.
- Error angular normalizado para elegir el giro más corto.
- Términos proporcional, integral y derivativo.
- Saturación y protección contra *windup*.
- Retroalimentación mediante la acción de ROS 2.
- Criterio de éxito basado en la tolerancia de todos los ejes.

## Ejemplo de objetivo

```bash
ros2 action send_goal   /pidcontrol   control_action/action/PIDcontrol   "{xd: 2.0, yd: 2.0, zd: 2.0, wd: 1.5708}"   --feedback
```

## Datos y evaluación

Se registran `/simple_drone/gt_pose`, `/simple_drone/cmd_vel` y `/simple_drone/state`. Con ellos se generan gráficas de `x`, `y`, `z`, yaw y señales de control. En la ejecución documentada durante el desarrollo, la acción alcanzó el objetivo aproximadamente en 6.631 s; este valor debe verificarse con el rosbag publicado.

## Aspectos relevantes

- El simulador debe interpretar `/simple_drone/cmd_vel` como velocidades.
- Las ganancias se ajustan por eje, con atención al amortiguamiento vertical.
- Los errores negativos conservan su signo; la tolerancia se verifica con valor absoluto.
- El reporte final utiliza `IEEEtran` e integra las figuras en el texto.

## Organización esperada

- `src/`: paquete `control_action` e interfaz de acción.
- `scripts/`: procesamiento de rosbag y gráficas.
- `resultados/`: gráficas y capturas.
- `reporte/`: reporte y fuentes LaTeX.
