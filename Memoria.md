---
title: "Proyecto Fin de Grado:"
subtitle: "Sistema de vigilancia autónomo\nbasado en la plataforma dron Parrot Bebop 2"
author: [José Javier Alonso Ramos]
date: "Curso: 2019 - 2020"
subject: "Markdown"
keywords: [Markdown, Example]
lang: "es"
titlepage: true
titlepage-rule-height: 1
logo: "/home/jjavier/GII/logoUGR/3.png"  
logo-width: 300
toc: TRUE
toc-own-page: TRUE
titlepage-color: 06c2a9
titlepage-text-color: 110406
---

# Agradecimientos

# Resumen

# Introducción

# Planificación

## Objerivos

## Metodología

## Planificación

# Componentes

## Hardware

### Parrot Bebop 2

## Software

### Ubuntu 16.04

### ROS

### bebop_autonomy

# Desarrollo del proyecto

## 1. Ubuntu 16.04 - Máquina Virtual

En primer lugar, crearemos una **máquina virtual (VM)** con la imagen de **Ubuntu 16.04**, ya que es la versión de Ubuntu que más soporte ofrece para **ROS (Robotic Operating System)**.
Se decide realizar el proyecto en una VM por la portabilidad que ofrece además de ahorrarnos tener que intalar una versión antigua de Ubuntu. Respecto al punto de la portabilidad, es importante indicar que podemos emular VMs en casi cualquier sistema operativo, y más aún si la plataforma donde nos interesa instalar el proyecto, en este caso, es un portatil. Tanto en Windows, Mac y Linux podremos ejecutar esta herramienta que nos permita hacer volar a nuestro dron _Bebop 2_ de forma autónoma.  

Esta VM no requiere de una configuración demasiado complicada ni técnicamente compleja. Tan solo tendremos que iniciarla con la i**magen de Ubuntu 16.04**, asignarle suficiente **memoria RAM** que garantice un funcionamiento fluido (en mi caso 2GB han bastado) y configurar el **controlador de red como _adaptador puente_** ya que queremos que la conexión Wi-Fi con el _Parrot_ sea directa, de otro modo no funcionará.

Siguiendo los sucesivos pasos que veremos a continuación, específicamente, a la hora de ejecutar los _drivers de control del dron_, hay ocasiones en que obtenemos errores de conexión con el aparato. Para estar seguros de que se trata de un error de conexión podemos intentar hacer _ping_ al dron cuya _IP_ por defecto es $192.168.42.1$. Si el pin falla, efectivamente, es un error en la conexión con el _Bebop 2_. En mi caso se soluciona accediendo a la configuración de red de la VM (no hace falta apagar la máquina) y, en opciones _Avanzadas_ desmarcar la casilla _"Cable conectado"_. Guardamos la configuración, esperamos a que se nos desconecte la red y volvemos a repetir el proseguimiento pero esta vez marcando la casilla antes desmarcada.

## 2. Instalar ROS Kinetic

**ROS Kinetic** es la distribución de ROS preparada para _Ubuntu 16.04_. Vamos a instalar la versión _Desktop Full_ que es la más recomendada y que además nos proporciona muchas herramientas desde el primer momento.

Aunque el mejor lugar para ver la guía de instalación (de cualquier software) siempre es su [página oficial](http://wiki.ros.org/kinetic/Installation/Ubuntu), voy a intentar sintetizar el procedimiento y hacerlo lo más sencillo posible.

1. Dar permiso a todos los tipos de repositorios de Ubuntu (_main, universe, restricted y multiverse_). Esto lo podemos hacer desde la aplicación de _Software y Actualizaciones_.
Para ver los pasos más detalladamente podemos consultar esta [página](https://help.ubuntu.com/community/Repositories/Ubuntu).

2. Hacer que nuestro equipo (VM) acepte paquetes desde _packages.ros.org_.
>sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

3. Obtenemos las claves de encriptado para ROS.
>sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654  

4. Nos aseguramos que tenemos los repositorios de Debian actualizados
>sudo apt-get update

5. Instalamos la versión _kinetic-desctop-full_
>sudo apt-get install ros-kinetic-desktop-full

6. Inicializamos _rosdep_ que nos permite instalar dependencias y es necesario para el funcionamiento de ciertos componentes de ROS.
>sudo rosdep init
rosdep update

7. Permitimos que todas las variables y todos los ajustes de ROS carguen cada vez que iniciamos una nueva consola.
>echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

8. Instalamos ciertas dependencias que nos permitirán crear y gestionar nuestros propios espacios de trabajo.
>sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential

Con todo esto listo ya tendremos ROS completamente instalado y listo para funcionar.

## 3. Instalar bebop_autonomy

## 4. Instalar video_stream_opencv

Para añadir web cam a la VM instalar Virtual Box Extension Pack, seleccionar interfaz usb 2.0 en la configuración de la VM, ejecutar:
>VBoxManage list webcams  

desde el directorio raíz. Seleccionar la webcam a importar con:  

>VBoxManage controlvm "_nombre de la VM_" webcam attach .X  

Para realizar todo esto es importante que figuremos como miembro del grupo vboxusers.

### 

# Conclusiones

# Referencias

[Web oficial ROS](http://wiki.ros.org/es)
[Bebop Autonomy](https://bebop-autonomy.readthedocs.io/en/latest/index.html)
[Video Stream OpenCV](http://wiki.ros.org/video_stream_opencv)  
[CV Camera](http://wiki.ros.org/cv_camera)  
[Ubuntu Help - habilitar repositorios](https://help.ubuntu.com/community/Repositories/Ubuntu)  
