# PSR TP3
Projeto Programação de Sistemas Robóticos, Departamento de Engenharia Mecânica, Universidade de Aveiro.

**Objetivo:** Pretende-se desenvolver um sistema robótico que funcione como um mordomo. O Robô deve ser capaz de realizar variadas missões de apoio habitualmente realizadas por trabalhadores humanos.
O robô deve ter desenvolvidas várias funcionalidades que suportem a operacionalização destas missões, tais como a perceção de objetos e a movimentação no cenário.

## Trabalho realizado por:
* 40170 - Ricardo Baptista
* 98322 - Gonçalo Rodrigues
* 110045 - Miguel Sobreira

## Funcionalidades:
* Navegação (o robô consegue navegar pelo apartamento, evitando obtaculos através do uso de LiDAR)
* Perceção (o robô é capaz de detetar cores, objetos e pessoas com o uso de uma câmera)
* Fotografia (o conjunto das duas funcionalidades anteriores permite ao robo fotografar divisões, objetos e pessoas, quando tem certeza de que os está a ver)
* Missões (através de um menu interativo, é possivel pedir ao robô que se desloque para uma certa divisão, ou que encontre um objeto ou pessoa, é possivel ainda mandá-lo para uma coordenada e ele viaja até la autonomamente)

---
## Instruções:
Para a execução do programa é necessário a instalar ROS Noetic, Python3, OpenCV2, YOLO, Turtlebot3.

Clonar para a pasta catkin os seguintes repositórios, de forma a modelar o ambiente Gazebo:

``` 1- cd catkin_ws/src```
``` 2- git clone https://github.com/aws-robotics/aws-robomaker-small-house-world```

---
## Comandos para inicializar (Terminator):
**Lançar o ambiente Gazebo**
```roslaunch robutler_bringup gazebo.launch```

**Spawn do Robutler com os módulos de perceção**
**Activar a navegação**
**Activar o teleop para controlo manual do robutler**
**Activar o menu_missions**
```roslaunch robutler_bringup bringup.launch```

**Activar sistema de imagem YOLO**
```roslaunch yolo_detection/launch/tracker.launch debug:=true```

**Efectuar spawn objectos/pessoas**
```rosrun robutler_bringup spawn_object.py -o [object] -p [place]```

[Object]: sphere_b, coke_can, laptop_pc_1, keyboard, mouse, human_female_1, human_male_1
[place]: bed, bedroom_table,bedroom_chair, sofa, orange_table, shelf, under_kitchen_table, door, on_kitchen_table



