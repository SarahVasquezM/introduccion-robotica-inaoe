# Auditoría de fuentes y ecuaciones

El reporte distingue tres clases de contenido:

1. **Métodos publicados:** RRT, obstáculos en espacio de configuraciones,
   representación Bézier, parametrización temporal, matrices de rotación,
   control proporcional de velocidad y RMSE.
2. **Derivaciones algebraicas del método:** velocidad de referencia, longitud
   espacial y velocidad máxima de la ley cúbica.
3. **Decisiones experimentales de esta implementación:** ganancias, límites de
   velocidad, márgenes, tolerancias y la regla de 2 s/m.

| Elemento del reporte | Procedencia |
|---|---|
| Crecimiento y extensión del RRT | LaValle (1998); LaValle, *Planning Algorithms*, Sec. 5.5 |
| Inflación y obstáculo en espacio de configuraciones | LaValle, *Planning Algorithms*, Cap. 4 |
| Curva Bézier cuadrática y continuidad | Farin, *Curves and Surfaces for CAGD*, Caps. 4–5 |
| Longitud de arco | Farin, Cap. 11 |
| Separación entre camino y tiempo; escalamiento cúbico | Lynch y Park, *Modern Robotics*, Cap. 9 |
| Rotación mundo–cuerpo y control P con entrada de velocidad | Lynch y Park, Caps. 3 y 11 |
| RMSE | Chai y Draxler (2014), DOI 10.5194/gmd-7-1247-2014 |
| Arquitectura ROS 2 | Macenski et al. (2022), DOI 10.1126/scirobotics.abm6074 |
| Gazebo | Koenig y Howard (2004) |

La expresión `T_min = 2L` no se atribuye a una publicación: fue elegida a partir
de pruebas con el dron simulado. Al combinarla con la ley cúbica se obtiene
algebraicamente una velocidad de referencia máxima aproximada de 0.75 m/s.
Asimismo, las tolerancias de 0.03 m y 0.05 rad y las ganancias del controlador
son parámetros documentados de este experimento, no constantes universales.
