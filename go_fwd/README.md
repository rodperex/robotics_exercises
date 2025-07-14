# Práctica 3: Avance y giro del Kobuki

En esta práctica debes crear un nodo de ROS 2 que, aplicando un FSM, haga que el robot ejecute dos movimientos de forma consecutiva:
1. Avanzar 1 metro
2. Girar PI radianes

Entonces el robot se parará y el nodo finalizará su ejecución.

Debes usar TFs (en particular odom->base_footprint) para determinar cuando el robot ha girado o avanzado
suficiente.
