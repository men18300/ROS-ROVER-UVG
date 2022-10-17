# Integración de Robot Operating System (ROS) dentro del ROVER UVG

La importancia que tiene los robots actualmente ha llegado a fines insospechables. Desde las industrias hasta las oficinas podemos encontrar actividad de estos seres artificiales, y cada vez se fabrican robots mucho más complejos. ROS es un framework flexible, con una amplia variedad de herramientas, librerías y paquetes que busca la creación de software complejo para tener robots robustos y con un comportamiento variado. ROS permite la simulación de distintas clases de robots como se muestra a continuación:

![Alt Text](https://thumbs.gfycat.com/SizzlingHilariousCopperhead-size_restricted.gif) ![Alt Text](https://blog.pal-robotics.com/wp-content/uploads/2016/12/TIAGogmapping.gif)
![Alt Text](https://www.tkjelectronics.dk/uploads/Rear_Wheel_Pose_Published.gif)

## Introducción

ROS proporciona funcionalidad para la abstracción de hardware, controladores para múltiples dispositivos, comunicación entre procesos dentro de varias máquinas, herramientas para simulación, visualización, entre otros. La característica clave de ROS es cómo se ejecuta el software y cómo se comunica, debido a que permite diseñar software complejo sin saber cómo funciona cierto hardware. Es por ello que gran número de robots que se fabrican hoy en día, utiliza este sistema operativo para su funcionamiento. 

En el caso de la Universidad del Valle de Guatemala, se cuenta con el proyecto del robot explorador modular que ha tenido sus últimas modificaciones hasta el año 2021. Aunque hasta el momento, no se ha logrado que el robot logre moverse con facilidad ni tampoco pueda realizar más de una función. Para lograr que el Rover UVG pueda ampliar sus aplicaciones, se cuenta con distintos módulos externos, entre ellos se encuentran encoders en las orugas del robot, módulos DMW1001, sistema de captura de movimiento Optitrack, LIDAR y cámaras. Con dichos módulos se planteó la propuesta de integración junto a ROS, de forma que el robot tenga sensores de localización, visión y mapeo. 

Para validar la correcta integración de los módulos externos se realizaron simulaciones realistas en Gazebo. Estas simulaciones abarcaron distintas clases de experimentos dentro de un mundo virtual, como control punto a punto, SLAM, entre otros. Posteriormente se realizó la selección de la computadora central adecuada que permitiera la integración de todos los módulos externos junto a ROS dentro del robot físico. Para poder validar la implementación física se realizaron pruebas simples dentro de una plataforma que nos permitió verificar la integración de los módulos y la funcionalidad de los controladores realizados en la etapa de simulación.

## Documentación y Código

En el presente repositorio se encuentras las siguientes carpetas:

- Documentos, en donde se encuentran el trabajo de graduación y protocolo que se realizó como propósito de este proyecto. 
- Código, se encuentra el código tanto para la simulación del proyecto como para la implementación física.

## Instalación de ROS 2 en una máquina virtual

Es importante mencionar que todo el proyecto se realizó mendiante la versión de ROS 2, específicamente su distribución Foxy. Para poder ejecutar los programas de ROS fue necesario tener una máquina virtual con Linux. Fue por ello que se descargó VirtualBox del siguiente enlace: https://www.virtualbox.org/es, y se seleccionaron las siguientes características:


|Sistema Operativo|    RAM  | Procesadores| Almacenamiento |
|-----------------|---------|-------------|----------------|
| Ubuntu (64 bits)| 9192 MB |    2        | 30 GB            |


Posteriomente se realizó la instalación de ROS 2 sobre dicha máquina virtual. Para ello se utilizó la siguiente guía de instalación: 
https://docs.ros.org/en/foxy/Installation.html, y se realizó la instalación con los `debian packages` para que se instalen todos los paquetes de simulación y visualización. Posteriomente a la instalación de ROS 2, fue necesario realizar la instalación del compilador con el cual trabaja ROS 2. Dicho compilador es `colcon`, y para ello solamente es necesario abrir una terminal dentro de la máquina virtual y copiar la instrucción: 

```console
~$ sudo apt install python3-colcon-common-extensions
```

## Simulación en Gazebo

Gazebo es un simulador 3D multi-robot con dinámica. Ofrece la posibilidad de simular con precisión y eficiencia, diversidad de robots, objetos y sensores en ambientes complejos interiores y exteriores. Gazebo genera, tanto la realimentación realista de sensores, como las interacciones entre los objetos físicamente plausibles, incluida una simulación precisa de la física de cuerpo rígido. Gazebo se instala en conjunto a ROS cuando se descarga la versión `debian`. Para verificar que Gazebo este instalado y funcione correctamente, basta con escribir la siguiente instrucción en alguna terminal:

```console
~$ gazebo
```

Después de algunos momentos, se debería de abrir el simulador con un mundo vacio. 

