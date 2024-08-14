# Baixar e configurar o ROS.
O ROS compatível com o Robotino 2, que é o que está sendo usado, é o ROS Noetic, (http://wiki.ros.org/noetic/Installation/Ubuntu) é necessário usar o Ubuntu 20.04. 

**Instalação:**

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt install curl # if you haven't already installed curl

curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt update

sudo apt install ros-noetic-desktop-full

source /opt/ros/noetic/setup.bash

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

source ~/.bashrc


Configurando o ROS.
Após instalar o ROS, é necessário configurar seu espaço de trabalho catkin (http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

 mkdir -p ~/catkin_ws/src
 cd ~/catkin_ws/src
 catkin_init_workspace
 cd ../
 catkin_make
 echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
 source ~/.bashrc

Usando um repositório de pacotes ROS para trabalhar com o Robotino
(https://github.com/red-itmo/robotino)

No seu terminal:
Adicione um repositório 
(http://wiki.openrobotino.org/index.php?title=Debrepository)

sudo su
echo "deb http://packages2.openrobotino.org focal main" > /etc/apt/sources.list.d/openrobotino.list
exit
sudo apt-get install robotino-api2
cd PATH_TO_YOUR_WORKSPACE/src
git clone https://github.com/red-itmo/robotino.git
cd PATH_TO_YOUR_WORKSPACE

Editar pasta:

sudo nano catkin_ws/src/robotino/robotino_node/CmakeLists.txt 

set(CMAKE_MODULE_PATH /usr/local/robotino/api2/cmake)
para que fique assim:

set(CMAKE_MODULE_PATH /opt/robotino/cmake)

catkin_make

Instalando comando de controle por keyboard.
(http://wiki.ros.org/teleop_twist_keyboard)

O noetic pode ser substituído por outra versão do ros que você esteja usando, por exemplo, melodic.

sudo apt-get install ros-noetic-teleop-twist-keyboard
Robotino-2-com-ROS

Gravando CF Card do Robotino:

Lembrar de salvar a imagem original do cartão, usando esse software
https://hddguru.com/software/HDD-Raw-Copy-Tool/

Download Windows Executable (works without installation): HDD Raw Copy ver.1.20 portable
https://wiki.openrobotino.org/index.php?title=Downloads#CF_card_images

No caso do Robotino2, com cartão de 1GB, usando essa opção:
2.4 
1GB 
dd-image (265MB) md5sum

No Robotino:

Baixar e configurar pacotes ROS para o Robotino.
Conectar o cabo de rede, entre o computador e o Robotino.

Configurando o DHCP na tela do Robotino:

Faça isso acessando o menu do Robotino com os botões no lado superior. Pressione Enter e navegue até Rede. Após a rede, navegue para eth0 e depois para DHCP. Uma vez no menu DHCP, espere até aparecer ativado, voltando para a tela inicial,  ela estará com o IP.

Instalar o putty no computador https://www.putty.org/

Conectar com o putty através do IP do Robotino (DHCP)
(https://github.com/AalborgUniversity-ControlLabs/start-here/blob/master/robotino/readme.md?plain=1)

Usuário: root
senha: dorp6 
Baixar os debs:
O php escolhe automaticamente a versão estável atual.

wget http://doc.openrobotino.org/download/packages/i386/rec-rpc_current.php
 wget http://doc.openrobotino.org/download/packages/i386/robotino-common_current.php
wget http://doc.openrobotino.org/download/packages/i386/robotino-daemons_current.php
wget http://doc.openrobotino.org/download/packages/i386/robotino-api2_current.php
wget http://doc.openrobotino.org/download/packages/i386/robotino-api2_current.php

É importante dar um ls para saber as versões
   
    dpkg -i rec-rpc-qt4.5.0_*_i386.deb
    dpkg -i robotino-common_*_i386.deb
    dpkg -i --auto-deconfigure robotino-daemons_*_i386.deb
    dpkg -i robotino-api2_*_i386.deb
    dpkg -i robotino-examples_*_i386.deb

Após todo esse passo a passo, o Robotino está pronto para ser controlado pelo ROS.
Comandos:

roscore
roslaunch robotino_node robotino_node.launch hostname:= IP do Robotino
rosrun teleop_twist_keyboard teleop_twist_keyboard.py

Criar um arquivo de inicialização: 

Na área de trabalho

#!/bin/bash

gnome-terminal -- bash -c "roscore; exec bash"

sleep 2

gnome-terminal -- bash -c "roslaunch robotino_node robotino_node.launch hostname:=$1
; exec bash"

sleep 2

gnome-terminal -- bash -c "rosrun teleop_twist_keyboard teleop_twist_keyboard.py; exec bash"

Abrir o terminal pela área de trabalho: ./robertinho_start.sh.save 172.26.102.103 (nome do seu arquivo e o ip do seu robô)
