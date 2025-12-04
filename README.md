\===============================================================

README – Control Visual TurtleBot3 con Gestos + micro-ROS + Depth EXTRA

Autores:

\- Adrián Eduardo Vargas Llanquipacha

\- Israel Silva Bernal

\===============================================================

Este proyecto implementa un sistema de control del TurtleBot3 mediante:

Detección de gestos corporales usando MediaPipe

Publicación de comandos desde un ESP32 usando micro-ROS

Modulación de velocidad usando profundidad del Kinect (EXTRA)

Simulación en Gazebo del movimiento final usando /cmd\_vel

\===============================================================

🚀 EJECUCIÓN DEL PROYECTO

\===============================================================

\---------------------------------------------------------------

1️⃣ COMPILAR EL WORKSPACE

\---------------------------------------------------------------

cd ~/Documents/Robotica/visual\_ws

colcon build --symlink-install

source install/setup.bash

\---------------------------------------------------------------

2️⃣ INICIAR EL AGENTE micro-ROS (ESP32)

\---------------------------------------------------------------

source /opt/ros/humble/setup.bash



cd ~/Documents/Robotica/visual\_ws

source install/setup.bash

ros2 run micro\_ros\_agent micro\_ros\_agent serial --dev /dev/ttyUSB0 -b 115200

\---------------------------------------------------------------

3️⃣ REPRODUCIR LA GRABACIÓN DEL KINECT O DE WEBCAM

\---------------------------------------------------------------

source /opt/ros/humble/setup.bash

-

cd ~/Documents/kinect

ros2 bag play kinect\_data2 --loop

PARA WEBCAM

ros2 run gesture\_vision webcam\_gesture\_node

\---------------------------------------------------------------

4️⃣ INICIAR EL NODO DE GESTOS (MediaPipe)

\---------------------------------------------------------------

source /opt/ros/humble/setup.bash

-

cd ~/Documents/Robotica/visual\_ws

source install/setup.bash

ros2 run gesture\_vision gesture\_node

\---------------------------------------------------------------

5️⃣ INICIAR EL NODO DE PROFUNDIDAD (EXTRA)

\---------------------------------------------------------------

source /opt/ros/humble/setup.bash

-

cd ~/Documents/Robotica/visual\_ws

source install/setup.bash

ros2 run gesture\_vision depth\_zones\_node

\---------------------------------------------------------------

6️⃣ VERIFICAR TÓPICOS Y NODOS ACTIVOS

\---------------------------------------------------------------

source /opt/ros/humble/setup.bash

-

ros2 node list

ros2 topic list

ros2 topic echo /gesture\_command

ros2 topic echo /zone\_dist\_center

ros2 topic echo /cmd\_vel

\---------------------------------------------------------------

7️⃣ PRUEBA DEL EXTRA (Simular profundidad)

\---------------------------------------------------------------

\# Lejos (velocidad normal)

ros2 topic pub /zone\_dist\_center std\_msgs/msg/Float32 "{data: 2.0}" -r 5

\# Medio (velocidad reducida)

ros2 topic pub /zone\_dist\_center std\_msgs/msg/Float32 "{data: 0.8}" -r 5

\# Cerca (NO avanzar)

ros2 topic pub /zone\_dist\_center std\_msgs/msg/Float32 "{data: 0.4}" -r 5

\# Cerca a la izquierda

ros2 topic pub /zone\_dist\_left std\_msgs/msg/Float32 "{data: 0.4}" -r 5

\# Cerca a la derecha

ros2 topic pub /zone\_dist\_right std\_msgs/msg/Float32 "{data: 0.4}" -r 5

\---------------------------------------------------------------

8️⃣ BOTÓN DE EMERGENCIA (ESP32)

\---------------------------------------------------------------

ros2 topic echo /cmd\_vel

\# Presionar el botón → STOP total + LED\_EXTRA

\# Soltar el botón → Emergencia permanece activa (sin gestos)

\---------------------------------------------------------------

9️⃣ EJECUTAR TURTLEBOT3 EN GAZEBO

\---------------------------------------------------------------

source /opt/ros/humble/setup.bash

-

export TURTLEBOT3\_MODEL=burger

ros2 launch turtlebot3\_gazebo turtlebot3\_world.launch.py

\===============================================================

📁 ARCHIVOS INCLUIDOS EN EL PROYECTO

\===============================================================

gesture\_node.py # Detección de gestos (MediaPipe)

depth\_zones\_node.py # Extra por profundidad (Left/Center/Right)

micro\_ros # Código ESP32 del ejercicio 2

extra\_micro\_ros #Codigo del ejercicio extra

\===============================================================

📝 NOTAS FINALES

\===============================================================

✔ El ESP32 ejecuta micro-ROS y publica /cmd\_vel.

✔ Los gestos se detectan desde la grabación del Kinect.

✔ Las zonas de profundidad ajustan la velocidad (EXTRA).

✔ El botón físico de emergencia detiene todo el sistema.

✔ El TurtleBot3 en Gazebo replica exactamente los comandos finales.

\===============================================================

FIN DEL README

\===============================================================
