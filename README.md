# MapBot
Robot autónomo con tres ruedas que utiliza dos cámaras para hacer un mapeo 3D de interiores.

## Tabla de contenido
<img src="https://github.com/nurianavarro/MapBot/blob/main/img/mapbot.png" align="right" width="300" alt="header pic"/>

1. [Descripción](#descripcion)
2. [Requerimientos](#requerimientos)
3. [Contribuciones asombrosas](#contribuciones-asombrosas)
4. [Esquema hardware](#esquema-hardware)
5. [Piezas 3D](#3d)
6. [Arquitectura Software](#arquitectura-software)
7. [Algoritmos](#algoritmos)
8. [Experimentos realizados](#exprimentos)
9. [Autores](#autores)

## Descripción
Mapbot es un robot de movimiento autónomo de dos ruedas con motor DC y una rueda loca, el cual, mediante dos cámaras, hace un mapeo 3D de su alrededor, con el objetivo de obtener el mapa completo de un recinto.

Para este mapeo son necesarias dos cámaras con las que, conociendo la distancia entre ellas, se pueden calcular distancias y profundidades de forma precisa y poder hacer una reconstrucción 3D sin ambigüedades. Sería la semejanza a la visión humana, donde son necesarios dos ojos para una correcta visualización del entorno. Con estas cámaras, cada ciertos frames, se creará el mapeado 3D del entorno.

Para el desplazamiento del robot se usan dos motores de corriente continua, uno en cada extremo de la base trasera del robot. La velocidad y la dirección de estos motores estarán dominadas por el controlador L298N.


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
* open3d

## Contribuciones asombrosas
Este robot es capaz de realizar, con un sistema de cámaras en estéreo, desplazamientos completamente autónomos por un recinto nunca visto antes por él. Todo el proceso se realiza calculando trayectorias según los puntos extraídos por las cámaras sin colisionar con ningún objeto.

A este movimiento autónomo le acompaña un mapeo 3D de todo su entorno, por lo que es capaz de generar un archivo con el recinto por el cual se está moviendo en 3D, y así poder visualizar este recinto desde un ordenador.


## Esquema Hardware
* 1x Arduino Uno Rev3
* 1x Raspberry Pi 4
* 2x Motor DC
* 1x Controlador L298N
* 1x Bluetooth
* 2x Cámaras
* 1x Fuente Alimentación
<img src="https://github.com/nurianavarro/MapBot/blob/main/img/componentes_hardware.png" width="600" align="center"/>

## Piezas 3D
Para este proyecto se necesitan varias componentes 3D.
* El chasis que será la estructura principal del robot a la cual se le unirán los diferentes componentes
<img src="https://github.com/nurianavarro/MapBot/blob/main/img/chasis.png" width="200" align="center"/>

* Las ruedas para hacer posible el desplazamiento del robot
<img src="https://github.com/nurianavarro/MapBot/blob/main/img/rueda.png" width="200" align="center"/>

* El soporte para anclar las cámaras
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
Consiste en el diseño del desplazamiento autónomo de un robot móvil por un recinto nunca visto antes, mediante las fotografías de las cámaras. El algoritmo se basa en escoger diferentes puntos de referencia (landmarks) de las imágenes que recoge para situarse y localizarse dentro del mapa. Los puntos tienen que ser característicos, fácilmente reconocibles y que no se puedan confundir, ya que serán la base para la orientación del robot. Cuantas más veces vea estos puntos, menos ruido tendrá el movimiento del robot y mejores resultados obtendrán.

### ORB (Oriented FAST and Rotated BRIEF) 
Para escoger estos puntos, se va a utilizar la técnica de ORB (Oriented FAST and Rotated BRIEF), que es la fusión de los mecanismos FAST (para detectar los puntos de referencia) y (BRIEF para declarar los descriptores de dichos puntos).
* Primeramente, se ejecuta el algoritmo FAST. Primero aplica un filtrado a la imagen para eliminar posibles puntos negros e imperfecciones que puedan escogerse erróneamente como keypoints. Para mejorar el rendimiento del programa (ya que es necesaria su ejecución en tiempo real) se ha decidido recorrer la imagen cada 2 píxeles, aunque conlleve la posible pérdida de algún keypoint mejor. El recorrido de la imagen se utiliza para determinar qué píxeles son posibles keypoints. Para ello, para cada píxel ‘x’ de la imagen, se selecciona un círculo de radio 3 de 16 píxeles alrededor de este (como en la imagen) y un threshold adecuado y se guarda la intensidad de los píxeles 1, 5, 9 y 13. Se clasificará un píxel como posible punto interesante en el caso de que 3 de ellos cumplan que son más brillantes que la intensidad del píxel central más el threshold, o más oscuros que la intensidad del píxel central menos el threshold, y se realiza esta comprobación para los 16 píxeles del círculo. Si este criterio se satisface para ‘n’ de estos píxeles (originalmente ‘n’ = 12), se coge este píxel como keypoint. Al finalizar este proceso para toda la imagen se eliminan aquellos keypoints adyacentes, cogiendo siempre aquel con un mayor score (el score se obtiene de la suma de la diferencia absoluta entre los valores de p y los 16 píxeles circundantes).
<img src="https://github.com/nurianavarro/MapBot/blob/main/img/circulo_bresenham.png" width="250" align="center"/>

* Después de obtener los keypoints se utiliza BRIEF para asignar descriptores a estos puntos interesantes. BRIEF es muy sensible al ruido, así que se hace un smooth de la imagen. Estos descriptores son secuencias de 128, 256 o 512 bits y se calculan de forma que, para cada keypoint, se asigna una ventana y se cogen parejas de puntos (dependiendo la longitud del descriptor se cogerán más o menos parejas). Con estas parejas y su intensidad de píxel, si la intensidad del píxel x es menor que la del y, se asignará un 1 a la posición que le corresponda en la cadena de bits. Si la condición se cumple al revés, se asignará un 0.
* Por último, se realiza el match, donde, gracias a los descriptores calculados con BRIEF, se pueden reconocer los mismos puntos en diferentes imágenes.  

## Experimentos realizados
A continuación se muestra la escena del Coppelia utilizada. Los objetos a mapear deben ser objetos a los que se les pueda añadir una textura, ya que los algoritmos de estéreo necesitan detalles para poder clasificar que un píxel dee una fotografía es el mismo que otro, así como, si hay zonas planas no hay puntos en los que poder establecer el matching correcto.

<img src="https://github.com/nurianavarro/MapBot/blob/main/img/escena.png" width="300" align="center"/>

Aquí vemos un ejemplo, en el que se muestran las imágenes obtenidas por la visión estéreo, el mapa de disparidad y una imagen del cloud point 3D generado.

**Imágenes obtenidas por las cámaras:**

<img src="https://github.com/nurianavarro/MapBot/blob/main/img/imagen1.png" width="300" align="center"/>
<img src="https://github.com/nurianavarro/MapBot/blob/main/img/imagen2.png" width="300" align="center"/>

**Mapa de disparidad:**

<img src="https://github.com/nurianavarro/MapBot/blob/main/img/mapa_disparidad.png" width="300" align="center"/>

**Mapa 3D generado:**

<img src="https://github.com/nurianavarro/MapBot/blob/main/img/mapa_3d.png" width="300" align="center"/>


## Autores
* Adrià Gómez Acosta
* Núria Navarro Juliana
* Alberto Rubio Pérez
* Oriol Serrat Salomó
