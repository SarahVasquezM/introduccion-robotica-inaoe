# Práctica 4: navegación continua con RRT

El nodo expone la interfaz existente `control_action/action/PIDcontrol` en
`/pidcontrol`. Cada meta se interpreta como un destino global: toma la pose actual
como A, valida B, calcula un RRT, aplica atajos, redondea los vértices, vuelve a
comprobar colisiones y sigue una sola trayectoria parametrizada durante `tf`.

Los puntos del RRT no son metas independientes. El dron conserva velocidad al
atravesarlos y sólo se detiene al final. Cuando termina una acción se puede enviar
otra coordenada sin reiniciar Gazebo ni el nodo.

## Compilación y pruebas

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

El launch abre el mundo, el dron y un único servidor `/pidcontrol`. No proporciones
el destino al launch. No ejecutes simultáneamente teleop, el seguidor YOLO, el
servidor PID anterior ni otro publicador de `/simple_drone/cmd_vel`.

Terminal 2, despega:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic pub --once /simple_drone/takeoff std_msgs/msg/Empty '{}'
```

Tras unos segundos, lleva el dron a z=2 m. Esta meta también pasa por el mismo
planificador; como XY no cambia, produce una trayectoria vertical:

```bash
ros2 action send_goal /pidcontrol control_action/action/PIDcontrol \
  '{xd: 0.0, yd: 0.0, zd: 2.0, wd: 0.0, tf: 8.0}' --feedback
```

Después envía cualquier coordenada libre dentro del mundo. Para cruzar de
`(0,0)` a `(6,0)` rodeando la pared:

```bash
ros2 action send_goal /pidcontrol control_action/action/PIDcontrol \
  '{xd: 6.0, yd: 0.0, zd: 2.0, wd: 0.0, tf: 26.0}' --feedback
```

Al terminar puedes regresar, sin reiniciar:

```bash
ros2 action send_goal /pidcontrol control_action/action/PIDcontrol \
  '{xd: 0.0, yd: 0.0, zd: 2.0, wd: 0.0, tf: 26.0}' --feedback
```

Cada nueva meta empieza en la pose real del dron. `wd` es el yaw deseado en
radianes y `tf` es el tiempo total de la trayectoria completa.

## Tiempo y resultado

La ruta tridimensional debe disponer como mínimo de 2 segundos por metro:

```text
tf_min = 2 × longitud_de_la_ruta
```

La longitud se conoce sólo después de ejecutar el RRT. Si `tf` es insuficiente,
la acción devuelve `ABORTED` antes de mover el dron e imprime el mínimo requerido.
No agrega tiempos de asentamiento en puntos intermedios ni al final.

Durante el vuelo imprime cada segundo:

- tiempo transcurrido y `tf`;
- errores x, y, z;
- error de yaw.

El resultado incluye tiempo deseado, tiempo real, longitud de la ruta y todos los
errores finales. Devuelve `SUCCEEDED` sólo si el error final de posición es como
máximo 0.03 m y el de yaw 0.05 rad. Contacto, inclinación peligrosa, pérdida de pose
o entrada en la envolvente física producen `ABORTED` y velocidad cero.

## Ruta y mapa

RViz muestra:

- `/rrt_map`: ocupación inflada;
- `/rrt/path_raw`: ruta original del RRT;
- `/rrt/path_smoothed`: trayectoria continua redondeada que sigue el dron.

Las líneas azules visibles en Gazebo pertenecen al sensor y no representan la
ruta. Para inspeccionar los tópicos manualmente:

```bash
ros2 topic echo /rrt/path_smoothed --once --qos-durability transient_local
ros2 topic info /simple_drone/cmd_vel --verbose
ros2 action list -t
```

Antes de aceptar una meta, el nodo comprueba que los tres obstáculos de Gazebo
coincidan con el YAML. El RRT utiliza radio de dron 0.4 m y margen de planificación
0.50 m. La vigilancia física utiliza un margen menor y además escucha
`/simple_drone/bumper_states`.

## Evidencia

Cada acción aceptada guarda un directorio en `~/ros2_ws/results/practica4/` con:

- `results.json`;
- `flight.csv` con referencia, pose y errores;
- `rrt_map.png` con árbol, ruta cruda y trayectoria suavizada.

La planificación sin Gazebo de los tres casos configurados sigue disponible:

```bash
ros2 run practica4_rrt plan_scenarios \
  --config src/practica4_rrt/config/world_config.yaml \
  --scenarios src/practica4_rrt/config/scenarios.yaml
```

Para aterrizar después de terminar o cancelar una acción:

```bash
ros2 topic pub --once /simple_drone/land std_msgs/msg/Empty '{}'
```
