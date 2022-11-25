# Programas para simulación e implementación física del Rover UVG

A continuación se presenta una guía en donde se explican cómo esta integrado cada archivo launch dentro de esta carpeta y su funcionalidad. Para poder ejecutar cada archivo launch solamente es necesario que nos encontremos dentro de la carpeta de nuestro proyecto, corremos el siguiente comando siempre que abramos una nueva terminal:
```console
~$ source install/setup.bash
```
Y luego corremos la siguiente instrucción con el nombre específico de cada archivo launch:

```console
~$ ros2 launch basic_mobile_robot <nombre_del_archivo_launch>

```
Es importante mencionar que siempre se debe colocar el nombre del archivo launch en conjunto con la extensión `launch.py`. A continuación se describe cada archivo:

- `sim_rover_uvg_slam.launch.py` : Este archivo launch se encarga de realizar SLAM con el modelo SDF del ROVER UVG en la simualción en Gazebo. El mapa lo irá actualizando con respecto a los objetos que esten agregados al mundo en Gazebo, lo único que tendremos que realizar es mover al robot por nuestra cuenta. Esto se puede realizar por medio de el plugin `rqt_robot_steering` en donde podemos enviar comandos de velocidad al robot.

- `sim_rover_uvg_control_pap.launch.py` : Este archivo launch se encarga de abrir un mundo en Gazebo y colocar al model SDF dentro de el. Podemos manejar al robot de igual forma con el plugin `rqt_robot_steering`, o bien se puede realizar control punto a punto por medio del controlador PID realizado. Esto se puede realizar corriendo el nodo del controlador con el comando:

  ```console
  ~$ ros2 run basic_mobile_robot simple_controller.py

  ```
 Y se podrá observar que el robot se empieza a mover hasta llegar al punto deseado. Si se desea cambiar el punto deseado es necesario abrir el script con el nombre de simple_controller.py que se encuentra dentro del paquete de controllers.

- `rover_uvg_urdf.launch.py` : Este archivo launch se utiliza únicamente para visualizar el URDF del ROVER UVG. Por si se desearan hacer cambios dentro del archivo, se   pueden observar dichos cambios una vez se corra este programa.
- `rover_uvg_slam_raspi.launch.py` : Este programa es que se debe de correr dentro de la Raspberry Pi 4, de manera que empiece a correr el programa del LIDAR y a su   vez corra el programa que controla los motores por medio de la información que se le indique. Este programa se corre en conjunto con el programa `rover_uvg_slam.launch.py` ya que este segundo programa es el encargado de ir actualizando el map con respecto a la información del LIDAR y la odometría.
- `rover_uvg_slam.launch.py` : Como se mencionó previamente, este programa recibe información del LIDAR y de odometría para ir generando un nuevo mapa del entorno.
- `rover_uvg_control_pap.launch.py` : Este programa recibe información de odometría para realizar control punto a punto, de igual forma que en la simulación, se debe de correr el controlador con el comando: 

  ```console
  ~$ ros2 run basic_mobile_robot simple_controller.py

  ```
  Si se desea cambiar el punto deseado es necesario abrir el script con el nombre de simple_controller.py que se encuentra dentro del paquete de controllers.
  
- `urg_launch.py` : Este programa nos ayuda a visualizar la información que esta obteniendo el LIDAR.
