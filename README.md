# MapBot
Robot autónomo con tres ruedas que utiliza dos cámaras para hacer un mapeo 3D de interiores.

## Tabla de contenido
1. [¿Qué es?](#que-es)
2. [Descripción](#descripcion)
3. [Requerimientos](#requerimientos)
4. [Amazing Contributions](#amazing-contributions)
5. [Esquema hardware](#esquema-hardware)
6. [Piezas 3D](#3d)
7. [Arquitectura Software](#arquitectura-software)
8. [Módulos](#modulos)
9. [Autores](#autores)

## ¿Qué es?
Mapbot es un robot de movimiento autónomo de tres ruedas, el cual, mediante dos cámaras, hace un mapeo 3D de su alrededor, con el objetivo de obtener el mapa completo de un recinto.

## Descripción


## Requerimientos


## Amazing Contributions


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
* El chasis que será la estructura del robot
<img src="https://github.com/nurianavarro/MapBot/blob/main/img/chasis.png" width="200" align="center"/>

* Las ruedas
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


## Módulos


## Autores
* Adrià Gómez Acosta
* Núria Navarro Juliana
* Alberto Rubio Pérez
* Oriol Serrat Salomó
