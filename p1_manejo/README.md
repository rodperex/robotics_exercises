# Práctica 1: Manejo del Kobuki

El objetivo de esta práctica es usar los comandos de ROS 2 en la terminal para examinar el robot Kobuki.
Debes completar el siguiente cuestionario para realizar la práctica. Incluye los comandos que has usado en cada una:

1. ¿Has tenido que hacer algún setup en tu ordenador? Indica los pasos

Hay que ejecutar los siguientes comandos:
```bash
source /opt/ros/jazzy/setup.bash
source ~/mi_ws/install/setup.bash
```

2. ¿Qué comando has usado para lanzar el Kobuki en ROS 2?

Hay que ejecutar para lanzarlo en el robot real:
```bash
ros2 launch kobuki kobuki.launch.py
```

Para lanzarlo en el simulador:
```bash
ros2 launch kobuki simulation.launch.py
```

3. ¿Qué nodos se lanzan?

Para velo hay que ejecutar en la terminal, después de lanzar el kobuki el siguiente comando:
```bash
ros2 node list
```
Aparecerán en la terminal los nodos que se lanzan

4. ¿Qué topics están disponibles?
Para velo hay que ejecutar en la terminal el siguiente comando:
```bash
ros2 topic list
```
Aparecerán en la terminal los nodos disponibles

5. Analiza los topics (tipo y QoS) de /cmd_vel, /events/bumper y /scan_filtered

Para ve los tipos hay que ejecutar en la terminal los siguientes comandos:
```bash
ros2 topic info /cmd_vel
ros2 topic info /events/bumper
ros2 topic info /scan_filtered
```

Para ve las QoS hay que añadir a los comandos anteriores '--verbose':
```bash
ros2 topic info /cmd_vel --verbose
ros2 topic info /events/bumper --verbose
ros2 topic info /scan_filtered --verbose
```

6. ¿Qué servicios o acciones están disponibles?

Para ve los servicios disponibles hay que ejecutar en la terminal el siguiente comando:
```bash
ros2 service list
```

Para ve las acciones disponibles hay que ejecutar en la terminal el siguiente comando:
```bash
ros2 action list
```
