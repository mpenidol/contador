<h1 align="center">Contador de obstáculos</h1>
 
**Descrição**: Pacote criado para realizar a contagem de obstáculos em um ambiente.
 
## Visão geral
 
Reconhecer e interagir com o ambiente é de total importância no âmbito da robótica móvel, visto que uma modelagem mais precisa do mundo permite ao robô ter mais autonomia, tomando decisões mais otimizadas para lidar com possíveis complicações em sua rotina, assim como obstáculos dinâmicos. Técnicas de visão computacional vem sendo a maneira implementada para realizar a contagem de objetos e estimar a posição do mesmo no mundo. Este pacote utiliza o sensor LiDAR para realizar a localização e contagem de objetos no mundo através da clusterização dos pontos lidos.
 
---
 
## 1. Requisitos
 
1. Ros Humble
  - Gazebo
2. Ubuntu 22.04
3. TurtleBot (Waffle)

Opcional:

4. Docker
---
 
## 2. Instalação
 
Copie os repositórios do Github para o *workspace* do ros:
 
```bash
$ cd ros_ws2/src
$ git clone https://github.com/mpenidol/contador
$ git clone https://github.com/mpenidol/navigation2
$ git clone https://github.com/mpenidol/turtlebot3_simulations.git
```
É necessário instalar as dependêncisas
```bash
$ rosdep install -i --from-path src --rosdistro humble -y
```
Além disso, é necessário realixar o *export* de algumas variáveis de ambiente.
> :warning: É necessário realizar a alteração do usuário (*USER*) no segundo *export* para o usuário onde está configurado o *workspace* do ROS.

```bash
$ export TURTLEBOT3_MODEL=waffle
$ export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/USER/ros2_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models
```


A seguir, será necessário realizar o *build* do pacote no *workspace* desejado.
 
```bash
$ source /opt/ros/humble/setup.bash
$ cd ros_ws2
$ colcon build
$ . install/local_setup.bash
```
---
## 3. Execução
 
O launch utilizado irá executar o ambiente de simulação Gazebo, o ambiente de visualização Rviz e o código responsável pela movimentação e contatem de objetos. O pacote de contagem do objetos foi programado para realizar a rotina de contatem em dois ambientes cujos obstáculos estão posicionados diferes. Os launchers contendo os diferentes ambientes de execução são executados pelos comandos:
 
```bash
ros2 launch nav2_bringup trial2.py headless:=False
ros2 launch nav2_bringup trial2.py headless:=False

```

---
 
## 4. Resultados
 
![Screenshot from 2022-12-20 16-00-49](https://user-images.githubusercontent.com/80800606/208907772-db634ca8-0909-4172-86d8-e5559915e697.png)

 
---
 
## 5. Algorítmo
 
---
 
## 6. Aprimoramentos
 
Tendo em vista que a navegação foi realizada a partir de um controlador de robô uniciclo, esta pode ser alterada para pelo stack de navegação Nav2, tornando o sistema mais robusto e confiável, além de ter mais suporte pela comunidade e possibilitar utilizar diferentes tipos de controladores para verificar a performance do algoritmo.
 
---
 
## 7. Possíveis Aplicações

Existem várias aplicações para detecção e contagem de objetos. Muitas aplicações envolvem questões de exploração, como a identificação e exploração do ambiente; manutenção, conhecendo o ambiente previamente é possível verificar a integridade de estruturas físicas como pilares, colunas ou canos; ou até aplicações na indústria, como, por exemplo, na indústria madeireira, onde teria a aplicação manter contabilizado as árvores plantadas.


---
 
## 8. Links Úteis

A seguir estão dispostos links referentes aos repositórios necessários para a execução do projeto:

[https://github.com/mpenidol/turtlebot3_simulations.git](https://github.com/mpenidol/turtlebot3_simulations.git

[https://github.com/mpenidol/navigation2.git](https://github.com/mpenidol/navigation2.git)
