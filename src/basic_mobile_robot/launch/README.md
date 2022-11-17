# Archivos LAUNCH

A continuación se presenta una guía en donde se explican cómo esta integrado cada archivo launch dentro de esta carpeta y su funcionalidad.

## Creación de paquetes y nodos en Python
Para el desarrollo de proyectos en ROS 2 es recomendado realizar un Workspace para
cada uno de los proyectos que se desean realizar. Es por ello que en la máquina virtual se creó
una carpeta llamadaWorkspaces que contendrá todos los proyectos creados para ROS 2. Una
práctica común es crear una carpeta con el nombre deseado seguido de `_ws` para indicar
que es un espacio de trabajo para ROS 2. Dentro de la carpeta del workspace se deben crear
los paquetes deseados, para lo cual es necesario abrir una terminal en dicha carpeta, o bien,
abrir una terminal y cambiar el directorio hasta que nos encontremos dentro del respectivo
workspace. Luego se procede a crear una carpeta con el nombre de src, abreviatura de la
palabra source en inglés, dentro de la cual se deberán crear todos los paquetes deseados. El
número de paquetes que se pueden crear dentro de un workspace es ilimitado, dicha cantidad
depende directamente del proyecto en que se trabaje y la organización que se desee tener.
Para la creación de un paquete en Python (dentro de la carpeta src) se deberá escribir
la siguiente instrucción:
```console
~$ ros2 pkg create <nombre del paqute> --build-type ament_python
```


Luego de esto se puede observar dentro de la carpeta src que se ha creado una carpeta con
el nombre específicado en la instrucción previa. Para todos los paquetes creados en Python
se podrá observar que ingresando dentro de dichas carpetas, se generaron 3 carpetas más. La
primera carpeta, con el mismo nombre de nuestro paquete, sera donde tendremos que almacenar
todos los programas creados. De igual forma, se generaron 3 archivos independientes
los cuales se describen a continuación:
package.xml: contiene los metadatos del paquete, como lo son el nombre del paquete,
distribución de ROS 2, información del autor, entre otros. [18]
setup.py: contiene instrucciones de como instalar el paquete. [18]
setup.cfg: el cual es un paquete de Python que ayuda a poder administar todos los
ejectubables que se almacenen en el paquete creado. [18]
Una vez creado el respectivo paquete, se puede compilar para verificar que no exista
ninguna clase de error. Esto se debe de realizar fuera de la carpeta de dichos paquete por
medio de la instrucción:
```console
~$ colcon build
```


Seguido de la creación de los paquetes, se deben crear los nodos para las distintas tareas.
La función de estos nodos depende directamente del proyecto en que se trabaje. A continuación
se presenta el código mínimo en Python que debe ser incluido dentro los archivos para
que pueda ser interpretado como un ejectuble válido en ROS 2. En un paquete en Python,
un ejecutable debe contener mínimo el siguiente código:
```python
import rlcpy
from rlcpy.node import Node
def main (args=None ) :
rclpy.init ( args=args )
node=Node ("py_test ")
rclpy.shutdown ( )
if _name_ == "_main_" :
main ( )
```
Para poder usar todas las funcionalidades de ROS, se necesita importar la librería rlcpy
al principio de todos nuestros programas. Dentro de la función main lo primero que se realiza
es inicializar la comunicación con ROS 2 mediante la función init. Luego se define el nodo y
se le asigna un nombre, en este caso py_test. Por último se tiene la función shutdowm la cual
se encarga de finalizar la comunicación con ROS 2. El código mostrado solamente enseña
la manera de crear un nodo y cómo establecer su comunicación con ROS 2. Para definir
las tareas y funciones del nodo se realiza con la programación básica de Python, definiendo
funciones sencillas o incluso, una práctica muy común en ROS es seguir los estándares para
el desarrollo de software orientado a objetos tanto en Python como en C++. 
Es importante mencionar que en el archivo package.xml se deben incluir todas las dependencias
de nuestros ejecutables, en este caso, se debe incluir la librería rlcpy y así con
las demás librerías que se utilicen.

## Creación de paquetes y nodos en C++
Para la creación de un paquete en C++ se deberá escribir la siguiente instrucción:
```console
~$ ros2 pkg create <nombre del paqute> --build-type ament_cpp
```
De la misma forma, para todos los paquetes creados en C++ se creará una carpeta con
el nombre específiciado y dentro de ella se podrán encontrar dos carpetas más. La primera
es la carpeta de include en donde se deberán almacenar todos los archivos de cabecera o los
header files en inglés, los cuales son de suma importancia para lenguajes como C y C++
en este caso. La segunda carpeta creada será src en donde se deberá colocar todos nuestros
programas. Adicionalmente, se crearán los siguientes archivos independientes:
package.xml: contiene los metadatos del paquete, como lo son el nombre del paquete,
distribución de ROS 2, información del autor, entre otros. [18]
Cmakelist.txt: contiene un conjunto de directivas e instrucciones que describen los
archivos de origen y los destinos del proyecto. [18]
Seguido de la creación de paquetes y habiendo compilado el mismo para verificar que no
existan errores, se deben crear los nodos que realizaran las distintas tareas. A continuación
se presenta el código mínimo en C++ que debe ser incluido dentro los archivos para que
pueda ser interpretado como un ejectuble válido en ROS 2. En un paquete en C++, un
ejecutable debe contener mínimo el siguiente código:
```cpp
#include "rclcpp/rclcpp.hpp"
int main (int argc, char ∗∗ argv )
{
rlcpp :: init ( argc, argv) ;
auto node=std :: make_shared<rlcpp :: Node>("cpp_test");
rclpp::shutdown();
}
```
De la misma manera que para los nodos creados con Python, se necesita importar la
librería rlcpy al principio de todos nuestros programas para poder disponer de todas las
funcionalidad de ROS. Dentro de la función main lo primero que se realiza es inicializar la
comunicación con ROS 2 mediante la función init. Luego se crea un nodo con el nombre
de cpp_test y por último se utiliza la función shutdowm para finalizar la comunicación con
ROS 2.
