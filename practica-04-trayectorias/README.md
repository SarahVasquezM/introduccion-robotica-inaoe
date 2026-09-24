# Práctica 4 — Trayectorias con tiempo deseado

## Objetivo

Extender el controlador para que el dron siga una referencia suave y alcance una posición final dentro de un tiempo deseado `td`, en lugar de aplicar inmediatamente una referencia constante.

## Descripción

El objetivo de la acción incorpora `xd`, `yd`, `zd`, `wd` y `td`. Para cada coordenada se construye un polinomio cúbico con velocidad inicial y final iguales a cero:

```text
a0 = p_inicial
a1 = 0
a2 = 3 (p_final - p_inicial) / td²
a3 = -2 (p_final - p_inicial) / td³
```

Mientras `t < td`, el servidor evalúa la posición y velocidad deseadas. Al cumplirse el tiempo:

- La referencia queda fija en el objetivo final.
- Las velocidades deseadas se fijan en cero.
- El PID continúa corrigiendo el error restante.

El yaw se controla con `wd`, en radianes, usando un error angular normalizado.

## Ejemplo de objetivo

```bash
ros2 action send_goal   /pidcontrol   control_action/action/PIDcontrol   "{xd: 2.0, yd: 2.0, zd: 2.0, wd: 1.5708, td: 10.0}"   --feedback
```

## Evaluación

La práctica compara la trayectoria deseada con la pose publicada por Gazebo y revisa:

- Seguimiento en `x`, `y` y `z`.
- Seguimiento del yaw.
- Tiempo solicitado frente al observado.
- Velocidades deseadas y comandos aplicados.
- Error final y comportamiento después de `td`.

La referencia cúbica evita cambios discontinuos y permite estudiar el compromiso entre rapidez, saturación y amortiguamiento.

## Organización esperada

- `src/`: versión de `control_action` utilizada.
- `scripts/`: análisis de trayectoria y gráficas.
- `resultados/`: gráficas y capturas.
- `reporte/`: reporte y fuentes LaTeX.
