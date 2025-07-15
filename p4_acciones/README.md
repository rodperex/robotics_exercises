# Práctica 4: Acciones

En esta práctica debes crear 2 paquetes:

1. En un paquete debes definir los interfaces necesarios.

2. En otro paquete debes crear dos nodos:
    - Nodo servidor: Este nodo implementa una acción por el que se le envían comandos de avance o de giro (uno u otro, no ambos), indicando la distancia a recorrer o los radianes a girar. El nodo está latente hasta que recibe la petición. Cuando ha acabado, se para y espera una nueva petición. Si recibe una petición mientras está ejecutando otro, el nuevo comando se empieza a ejecutar inmediatamente, expulsando el anterior. El robot notifica a través de feedback:
      - distancia avanzada o giro realizado
      - lo que le queda para completar la petición
      - y el tiempo que le está llevando realizarlo
    - Nodo cliente: Este nodo se encarga de recibir como argumento la distancia a recorrer o los radianes a girar y hará la petición al servidor, mostrando por la consola el feedback recibido