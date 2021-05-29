# PL3_r2p2 Planificación Automática
Autor: Pablo Largo Rubio

## Path planning
En este apartado se ha creado el algoritmo Theta*, las heurísticas Manhattan, Euclidean y Octile y el nuevo escenario correspondiente al de la diapositiva 38 del PDF "Path Planning". 
Todo ello se encuentra dentro de la carpeta del propio simulador "r2p2" en ficheros .py (Python) y en png en caso del escenario (en la carpeta res del simulador).

También se han realizado distintas capturas, que corresponden al uso de los distintos algoritmos y las distintas heurísticas.
Todas estas capturas se encuentran en la carpeta "Path-planning apartado 8 (diferencias entre distintos algoritmos y heurísticas)", que se encuentra fuera de la carpeta del simulador.

## Integration
En este apartado he modificado el dominio y el problema del "Planetary exploration on Mars extended" para poder utilizar la solución que de el planificador con el simulador "r2p2". La solución se encuentra dentro de la carpeta "res" en el fichero "planning.txt".

Los ficheros .pddl correspondientes al dominio y el problema, se encuentran fuera del simulador en la carpeta "Dominio y problema para la integración en PDDL". En esta carpeta también se encuentra una captura del simulador una vez ha ejecutado el camino generado usando la solución del problema dada por el planificador PDDL ("planning.txt").

## What else?
Para poder controlar de manera autónoma faltaría por implementar una parte importante: el aprendizaje automático o machine learning. Es un punto vital, que permitirá al robot optimizar su trabajo teniendo en cuenta las tareas que ha realizado anteriormente. De esta forma, los planes y los caminos generados para el robot, tendrán en cuenta los hábitos a los que el robot es sometido.

