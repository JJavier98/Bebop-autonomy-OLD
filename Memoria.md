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

### 

# Conclusiones

# Referencias

[Web oficial ROS](http://wiki.ros.org/es) - [http://wiki.ros.org/es](http://wiki.ros.org/es)  
[Bebop Autonomy](https://bebop-autonomy.readthedocs.io/en/latest/index.html) - [https://bebop-autonomy.readthedocs.io/en/latest/index.html](https://bebop-autonomy.readthedocs.io/en/latest/index.html)
