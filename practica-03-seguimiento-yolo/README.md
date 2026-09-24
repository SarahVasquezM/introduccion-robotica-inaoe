# Práctica 3 — Seguimiento de una persona con YOLO

## Objetivo

Detectar y seguir una persona desde la cámara frontal del dron, integrando percepción con YOLO, control reactivo y una máquina de estados en ROS 2.

## Descripción

La cámara frontal publica en `/simple_drone/front/image_raw`. YOLO11n preentrenado detecta personas y publica cajas de detección. `drone_object_follower` transforma esas detecciones en comandos para centrar a la persona y conservar una distancia aproximada.

Los errores principales usan:

- Posición horizontal de la caja: desplazamiento lateral y yaw.
- Posición vertical de la caja: movimiento en `z`.
- Área relativa de la caja: avance o retroceso.

## Máquina de estados

| Estado | Función |
|---|---|
| `MANUAL` | Espera inicial; no publica comandos automáticos. |
| `SEARCH` | Busca girando y cambia de perspectiva en `x` o `y`. |
| `TRACKING` | La persona fue detectada y el dron la sigue. |
| `LOST_WAIT` | Se detiene brevemente después de perder la detección. |

El flujo normal es despegar manualmente, activar el modo automático y regresar a manual cuando sea necesario. Después de dos vueltas sin detección, la búsqueda incorpora desplazamientos para encontrar a una persona situada debajo del campo de visión frontal.

## Ejecución

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch practica3_bringup practica3.launch.py
```

Para inspeccionar los comandos:

```bash
ros2 topic echo /simple_drone/cmd_vel
```

## Evaluación

El nodo registra un CSV con tiempo, estado, errores de imagen, área de la caja y comandos. Con él se generan la vista completa, el error horizontal en `TRACKING`, los errores vertical y de distancia, y los comandos `x`, `y`, `z` y yaw.

La solución usa YOLO11n preentrenado; no se entrenó una red propia. Para reducir oscilaciones cerca de la persona se emplean zonas muertas, histéresis, filtrado y límites de velocidad o aceleración.

## Organización esperada

- `src/`: seguidor, teclado del objeto y paquete integrador.
- `scripts/`: procesamiento del CSV y gráficas.
- `resultados/`: figuras, capturas y enlaces a videos.
- `reporte/`: reporte y fuentes LaTeX.
