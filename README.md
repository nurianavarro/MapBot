# MapBot
Robot autónomo con tres ruedas que utiliza dos cámaras para hacer un mapeo 3D de interiores.

## Tabla de contenido
1. [Descripción](#descripcion)
2. [Requerimientos](#requerimientos)
3. [Amazing Contributions](#amazing-contributions)
4. [Esquema hardware](#esquema-hardware)
5. [Piezas 3D](#3d)
6. [Arquitectura Software](#arquitectura-software)
7. [Algoritmos](#algoritmos)
8. [Autores](#autores)

## Descripción
Mapbot es un robot de movimiento autónomo de tres ruedas, el cual, mediante dos cámaras, hace un mapeo 3D de su alrededor, con el objetivo de obtener el mapa completo de un recinto.

Para este mapeo, son necesarias dos cámaras. Con las que, conociendo la distancia entre ellas, se pueden calcular distancias y profundidades de forma precisa y poder hacer una reconstrucción 3D sin ambigüedades. Sería la semejanza a la visión humana, donde son necesarios 2 ojos para una correcta visualización del entorno. Con estas cámaras, cada ciertos frames se creará el mapeado 3D del entorno.

Para el desplazamiento del robot se usan 2 motores de corriente continua, uno en cada extremo de la base del robot. La velocidad y la dirección de estos motores estarán dominadas por el controlador L298N.


## Requerimientos
* cv2
* numpy
* math
* matplotlib.pyplot
* convolve2d
* cdist
* plot_matches
* pyramid_gaussian
* Image
* ImageFilter
* ImageDraw

## Amazing Contributions
Este robot, es capaz de realizar desplazamientos completamente autónomos por un recinto nunca visto antes, con un sistema de cámaras en estéreo. Todo esto calculando trayectorias según los puntos extraídos por las cámaras sin colisionar con ningún objeto.
A este movimiento autónomo, le acompaña un mapeo 3d de todo su entorno, así que, es capaz de generar un archivo con el recinto por el cual se está moviendo en 3d, para poder visualizar este recinto desde un ordenador.

## Esquema Hardware
* 1x Arduino Uno Rev3
* 1x Raspberry Pi 4
* 2x Motor DC
* 1x Controlador L298N
* 1x Bluetooth
* 2x Cámaras
* 1x Fuente Alimentación
<img src="https://github.com/nurianavarro/MapBot/blob/main/img/hardware.png" width="600" align="center"/>

## Piezas 3D
Para este proyecto se necesitan varias componentes 3D.
* El chasis que será la estructura principal del robot a la cual se le unirán los diferentes componentes
<img src="https://github.com/nurianavarro/MapBot/blob/main/img/chasis.png" width="200" align="center"/>

* Las ruedas para el desplazamiento del robot
<img src="https://github.com/nurianavarro/MapBot/blob/main/img/rueda.png" width="200" align="center"/>

* El soporte para las cámaras
<img src="https://github.com/nurianavarro/MapBot/blob/main/img/soporte.png" width="200" align="center"/>

* Las cámaras
<img src="https://github.com/nurianavarro/MapBot/blob/main/img/camara.png" width="200" align="center"/>

## Arquitectura Software
<img src="https://github.com/nurianavarro/MapBot/blob/main/img/software.png" width="600" align="center"/>

* **Obtener imágenes del entorno:** módulo encargado de obtener las imágenes de las cámaras y procesarlas para una posterior utilización de estas.
* **Escoger puntos de referencia:** mediante la técnica ORB se escogen diferentes puntos de la imagen como puntos de referencia.
* **Calcular odometría:** cálculo de la posición del robot según los puntos de referencia vistos hasta el momento.
* **Recalcular puntos:** recalcular posición de los puntos en el mundo 3d envolvente al robot.
* **Calcular trayectoria:** con la odometría y los puntos de referencia, calcular la trayectoria que debe seguir el robot sin colisionar con ningún objeto.
* **Mapeo 3D:** módulo encargado de procesar las imágenes de forma que cree un mapa 3D del entorno que va viendo según se mueve el robot.

## Algoritmos
### SLAM (Simultaneous Location And Mapping)
Consiste en que, mediante cámaras, un robot móvil es capaz de desplazarse autónomamente por un recinto nunca visto antes, es decir, sin previo conocimiento de este. Este algoritmo, se basa en escoger diferentes puntos de referencia (landmarks) de las imágenes que va viendo para situarse y localizarse dentro del mapa. Estos, tienen que ser puntos característicos fácilmente reconocibles y que no se puedan confundir ya que serán la base para la orientación del robot. Cuantas más veces vea estos puntos, menos ruido tendrá el movimiento del robot y mejores resultados obtendrán.

### ORB (Oriented FAST and Rotated BRIEF) 
Para escoger estos puntos, se va a utilizar la técnica de ORB (Oriented FAST and Rotated BRIEF) que es la fusión de los mecanismos FAST para detectar los puntos de referencia y BRIEF para declarar los descriptores de dichos puntos.
* Primeramente, se ejecuta el algoritmo FAST. Este, primero aplica un filtrado a la imagen para eliminar posibles puntos negros e imperfecciones de la imagen que puedan cogerse erróneamente como keypoints. Para mejorar el rendimiento del programa, ya que es necesaria su ejecución en tiempo real, se ha decidido recorrer la imagen cada 2 píxeles, aunque con ello sea posible la pérdida de algún keypoint mejor. Este recorrido de la imagen se utiliza para determinar qué píxeles son posibles keypoints. Para ello, para cada píxel ‘x’ de la imagen, se coge un círculo de radio 3 de 16 píxeles alrededor de este como en la imagen y un threshold adecuado. De este círculo, se coge la intensidad de los píxeles 1, 5, 9 y 13. Si 3 de ellos satisfacen que son más brillantes que la intensidad del píxel central más el threshold, o más oscuros que la intensidad del píxel central menos el threshold, se coge este píxel como posible punto interesante y se realiza esta comprobación para los 16 píxeles del círculo. Si este criterio se satisface para ‘n’ de estos píxeles (originalmente ‘n’ = 12), se coge este píxel como keypoint. Al finalizar este proceso para toda la imagen, se eliminan aquellos keypoints adyacentes cogiendo siempre aquel con un mayor score, este score sale de la suma de la diferencia absoluta entre los valores de p y los 16 píxeles circundantes.
<img src="https://github.com/nurianavarro/MapBot/blob/main/img/circulo_bresenham.png" width="250" align="center"/>

* Después de obtener los keypoints, se utiliza BRIEF para asignar descriptores a estos puntos interesantes. BRIEF, es muy sensible al ruido así que se hace un smooth de la imagen. Estos descriptores, son secuencias de 128, 256 o 512 bits y se calculan de forma que, para cada keypoint, se asigna una ventana y se cogen parejas de puntos, dependiendo la longitud del descriptor se cogerán más o menos parejas. Con estas parejas y su intensidad de píxel, si la intensidad del píxel x es menor que la del y, se asignará un 1 a la posición que le corresponda en la cadena de bits. Si la condición se cumple al revés, se asignará un 0.
* Por último, está la parte de match, con la que gracias a los descriptores calculados con BRIEF, se pueden reconocer los mismos puntos en diferentes imágenes. 

## Autores
* Adrià Gómez Acosta
* Núria Navarro Juliana
* Alberto Rubio Pérez
* Oriol Serrat Salomó
