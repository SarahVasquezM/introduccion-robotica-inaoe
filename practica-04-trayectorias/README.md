# Práctica 4 — Planeación y seguimiento continuo con RRT

Esta práctica hace que el dron simulado reciba una coordenada global arbitraria,
calcule una ruta libre de colisiones con RRT y la recorra suavemente en el tiempo
solicitado. El destino se envía mediante la acción existente
`control_action/action/PIDcontrol`; no se fija al lanzar el mundo.

El material corresponde al reporte y a la ejecución documentada en:

<https://github.com/SarahVasquezM/introduccion-robotica-inaoe/tree/main/practica-04-trayectorias>

## Requisitos cubiertos

- Mundo de Gazebo con obstáculos y espacio libre.
- Mapa de ocupación con obstáculos inflados por el radio del dron y un margen.
- RRT con comprobación de colisiones sobre cada segmento.
- Atajos y redondeo de esquinas mediante curvas Bézier comprobadas contra el mapa.
- Parametrización por longitud de arco y una sola ley temporal cúbica.
- Seguimiento continuo sin detenerse en los puntos intermedios.
- Verificación del tiempo solicitado, error final, contactos e inclinación.
- Replaneación para nuevas coordenadas sin reiniciar Gazebo.

## Contenido

```text
practica-04-trayectorias/
├── src/
│   ├── control_action/       # definición mínima de PIDcontrol.action
│   └── practica4_rrt/        # planificador, controlador, mundo, launch y pruebas
├── resultados/detour_26s/    # CSV, JSON y mapa de la corrida citada
├── reporte/                  # LaTeX, PDF, figuras vectoriales y métricas
└── documentacion/            # validación, materiales y auditoría de fuentes
```

No se incluyen `build/`, `install/`, cachés ni logs. El simulador
[`sjtu_drone`](https://github.com/NovoG93/sjtu-drone) es una dependencia externa
y debe estar instalado en el workspace.

## Compilación

Copiar o enlazar los dos paquetes de `src/` dentro de un workspace ROS 2 Humble.
Después:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select control_action practica4_rrt --symlink-install
source install/setup.bash
python3 -m pytest -q src/practica4_rrt/test/test_planning.py
```

## Ejecución

Terminal 1:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch practica4_rrt practica4_rrt.launch.py rviz:=true
```

Terminal 2, despegue y ascenso inicial:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic pub --once /simple_drone/takeoff std_msgs/msg/Empty '{}'

ros2 action send_goal /pidcontrol control_action/action/PIDcontrol \
  '{xd: 0.0, yd: 0.0, zd: 2.0, wd: 0.0, tf: 8.0}' --feedback
```

Ejemplo que obliga a rodear el muro central:

```bash
ros2 action send_goal /pidcontrol control_action/action/PIDcontrol \
  '{xd: 6.0, yd: 0.0, zd: 2.0, wd: 0.0, tf: 26.0}' --feedback
```

Se puede enviar otra coordenada al terminar, sin reiniciar:

```bash
ros2 action send_goal /pidcontrol control_action/action/PIDcontrol \
  '{xd: 0.0, yd: 0.0, zd: 2.0, wd: 0.0, tf: 26.0}' --feedback
```

`wd` es el yaw deseado en radianes y `tf` es la duración de la trayectoria
completa. La ruta debe satisfacer `tf >= 2L`, donde `L` es su longitud espacial.
Si el tiempo es físicamente insuficiente, la acción se rechaza antes del
movimiento e imprime el mínimo requerido.

## Evidencia y resultados

La corrida seleccionada parte aproximadamente de `(0, 0, 2)` m y termina en
`(6, 0, 2)` m. Sus resultados principales son:

| Métrica | Resultado |
|---|---:|
| Tiempo solicitado | 26.000 s |
| Tiempo de pared | 26.010 s |
| Longitud suavizada | 11.679 m |
| Error horizontal transitorio máximo | 0.1516 m |
| Error final de posición | 0.0228 m |
| Contactos de Gazebo | 0 |

`resultados/detour_26s/flight.csv` contiene las 2275 muestras usadas para los
errores y las gráficas. `results.json` conserva la semilla, parámetros, rutas y
resultado de la acción.

Para regenerar las figuras desde la raíz de esta carpeta:

```bash
python3 src/practica4_rrt/scripts/generate_report_figures.py \
  resultados/detour_26s --output reporte/figures
```

## Reporte

El artículo y sus fuentes están en [`reporte/`](reporte/). Para compilar:

```bash
sudo apt install texlive-publishers texlive-fonts-recommended
cd reporte
pdflatex report.tex
pdflatex report.tex
```

Las fuentes matemáticas y la separación entre fórmulas bibliográficas,
derivaciones propias y parámetros experimentales se documentan en
[`documentacion/FUENTES.md`](documentacion/FUENTES.md).
