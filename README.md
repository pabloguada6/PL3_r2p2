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
Para poder controlar de manera autónoma faltan por implementar diversas partes, y entre ella se encuentra el aprendizaje automático o machine learning (relacionado con la IA). Es un punto vital, que permitirá al robot optimizar su trabajo teniendo en cuenta las tareas que ha realizado anteriormente. De esta forma, los planes y los caminos generados para el robot, tendrán en cuenta los hábitos a los que el robot es sometido y podrá predecir acciones siendo, de esta forma, mucho más autónomo e inteligente.

Tambiénj sería necesario desarrollar una interfaz gráfica para tener la posibilidad de indicar las tareas que queremos que el robot lleve a cabo. En base a las tareas requeridas, el planificador generará el plan más óptimo y con ello se podrá generar el camino más óptimo con los distintos algoritmos y heurísticas que ya están desarrolladas.

Además, también falta programar un sistema de "re-planning" para realizar y ejecutar un nuevo plan en caso de que algo vaya como no se esperaba en el plan original.

Otro posible factor a implementar, sería programar al robot para que en caso de encontrarse atascado en una situación en la que no pueda continuar, pida ayuda a los seres humanos mediante correo electrónico o mediante voz para los seres humanos que estén alrededor. De esta forma, el robot podrá ser totalmente autonómo, pidiendo ayuda en caso de que se encuentre ante un problema, como un ser humano podría hacer en su caso.

El resto de factores como el planificador, el generador de caminos o la reacción ante los elementos del ambiente en que se encuentra (detecta obstáculos) ya están implementados en esta práctica.

