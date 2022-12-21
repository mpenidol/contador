<h1 align="center">Contador de objetos</h1>
 
**Descrição**: Pacote criado para realizar a contagem de objetos em um ambiente.
 
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
cd ros_ws2/src
git clone https://github.com/mpenidol/contador
git clone https://github.com/mpenidol/navigation2
git clone https://github.com/mpenidol/turtlebot3_simulations.git
```
Os repositórios nav2 e turtlebot3 são os pacotes padrões mas com algumas alterações. 
No nav2 o mapa padrão do ambiente com 9 cilindros foi removido e foi colocado um mapa que contém apenas as restrições do ambiente. Na aba do launch, foram adicionados dois launchs (trial1.py e trial2.py), eles fazem a mesma função do tb3_simulation_launch.py, porém inicializam o robô em uma posição inicial diferente e chamam a rotina que realiza a navegação e contagem de objetos. Na pasta de params os arquivos com nome trial realizam a mesma tarefa que o na2_params.yaml, mas sem chamar as rotinas de navegação (global_cost_map, local_cost_map). No pacote do turtlebot, foram adicionados dois modelos de ambiente, que tem a distribuição dos cilindros mostrados na apresentação. Para usá-los basta trocar no arquivo do model do config.

É necessário instalar as dependêncisas
```bash
rosdep install -i --from-path src --rosdistro humble -y
```
Além disso, é necessário realixar o *export* de algumas variáveis de ambiente.
> :warning: É necessário realizar a alteração do usuário (*USER*) no segundo *export* para o usuário onde está configurado o *workspace* do ROS.

```bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/USER/ros2_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models
```
Por ser executado em python, é necessário realizar a instalação de bibliotecas e dependências do código de contagem e navegação:

```bash
pip install shapely
pip install -U scikit-learn
pip install pandas
```

A seguir, será necessário realizar o *build* do pacote no *workspace* desejado.
 
```bash
source /opt/ros/humble/setup.bash
cd ros_ws2
colcon build
. install/local_setup.bash
```
---
## 3. Execução
 
O launch utilizado irá executar o ambiente de simulação Gazebo, o ambiente de visualização Rviz e o código responsável pela movimentação e contatem de objetos. O pacote de contagem do objetos foi programado para realizar a rotina de contatem em dois ambientes cujos obstáculos estão posicionados diferentes. Os launchers contendo os diferentes ambientes de execução são executados pelos comandos:
 
```bash
ros2 launch nav2_bringup trial2.py headless:=False
ros2 launch nav2_bringup trial2.py headless:=False

```

---
 
## 4. Algoritmo
 
 A navegação do robô foi realizada através de um controlador de robô unicilo, onde é calculado um ρ que é a distância euclidiana até o ponto desejado, sendo Ψ a orientação do robô, e α é oângulo entre a orientação atual do robô e o vetor ρ através da seguinte equação.
 
 ![Screenshot from 2022-12-21 10-39-26](https://user-images.githubusercontent.com/80800606/208919175-822f87f7-b6da-4848-b142-a3c83f3f580f.png)
 
Por fim, a lei de controle utilizada para calcular as velocidades lineares e angulares, dado que vmax é a velocidade máxima permitida do robô e kωα é uma constante positiva, é calculada pela seguinte equação.
 
![Screenshot from 2022-12-21 10-39-32](https://user-images.githubusercontent.com/80800606/208919684-37fba16f-7576-4176-bf7d-1732a621f58f.png)

A partir disto, a partir deuma posição aleatória, foram determinados pontos fixos no mapa para que este realize a navegação de forma autônoma, enviando os pontos desejado, conforme disposto na imagem abaixo.

![Screenshot from 2022-12-21 10-47-23](https://user-images.githubusercontent.com/80800606/208920181-3adcc7a9-579a-4bdf-b17f-7a970d6f6bf8.png)

A contagem dos objetos é realizada através da clusterização realizada pelas leituras do LiDAR juntamente a informação de odometria obtida através das leituras dos encoders. A leitura do LiDAR é particionada em 360 posições, podendo obter a posição de cada ponto no mapa através de transformações trigonométricas. 

![Screenshot from 2022-12-21 11-00-04](https://user-images.githubusercontent.com/80800606/208922930-fd3ebada-4afa-4421-94e6-9d9f82ab0eae.png)

Um conjunto de leituras que resultam em pontos pŕoximos no ambiente podem simbolizar a presença de objetos no mesmo. Para isso, foi utilizado um algoritmo de agrupamento (DBSCAN) para juntar esses pontos.

![Screenshot from 2022-12-21 11-03-05](https://user-images.githubusercontent.com/80800606/208923535-68c512a5-4f85-407a-9861-baf2889ad091.png)

A clusterização realizada retorna o ponto central de cada objeto identificado. Desta forma, para cada objeto identificado é necessário realizar um comparativo com objetos previamente identificados pela clusterização, verificando se este é um novo objeto.

![Screenshot from 2022-12-21 11-11-35](https://user-images.githubusercontent.com/80800606/208926461-b4be3f3a-b478-46f7-992c-82a6cc8de847.png)

Por fim, é realizada a sinalização visual no mapa quando um novo objeto é identificado, assim como é gerado um arquivo csv no diretório "~/Desktop/" contendo a coordenada dos objetos identificados.

![Screenshot from 2022-12-21 11-18-54](https://user-images.githubusercontent.com/80800606/208927244-caafa3c9-eaec-4311-bb38-33a089b929da.png)

---
 
 ## 5. Resultados
 ### Trial 1

![Screenshot from 2022-12-20 16-00-49](https://user-images.githubusercontent.com/80800606/208907772-db634ca8-0909-4172-86d8-e5559915e697.png)

 
A estratégia funcionou de forma satisfatória para as aplicações desejadas. Onde no primeiro trial o erro máximo em X foi de 0.25 e Y de 0.16, o erro médio em X foi de 0.13 e Y de 0.11. No segundo trial o erro máximo em X foi de 0.25 e Y de 0.43, o erro médio em X foi de 0.10 e Y de 0.15.

 ![Screenshot from 2022-12-21 11-25-48](https://user-images.githubusercontent.com/80800606/208928101-27e35beb-4b80-44d4-9675-8bf6ca9157a6.png)

  ### Trial 2
  
  ![Screenshot from 2022-12-21 11-27-40](https://user-images.githubusercontent.com/80800606/208928478-2f716aa1-fc92-45d2-aa23-8ca3133cf7ea.png)

No segundo trial, acabou não ocorrendo a identificação de um cilindro devido a proximidade com outro, mas isso poderia ser resolvido com a calibração do critério de definição de novos obstáculos ou melhor calibração do DBSCAN.

![Screenshot from 2022-12-21 11-29-02](https://user-images.githubusercontent.com/80800606/208928735-f25578fb-ec5e-405c-a009-2b8798f3f819.png)


---
 
 
## 6. Aprimoramentos
 
Tendo em vista que a navegação foi realizada a partir de um controlador de robô uniciclo, esta pode ser substituida para pelo stack de navegação Nav2, tornando o sistema mais robusto e confiável, além de ter mais suporte pela comunidade e possibilitar utilizar diferentes tipos de controladores para verificar a performance do algoritmo. Além disso, o controlador utilizado não integra a leitura do LiDAR em sua implementação, não sendo possível então realizar o desvio obstáculos, ficando limitado a um ambiente conhecido, onde os *goals* determinados não podem ter em seu caminho algo que obstrua sua movimentação. Desta forma, a utilização do *stack* de navegação Nav2 possibilita a utilização do algoritimo em ambientes genéricos.

A partir do momento que um objeto é identificado, o algoritmo não realiza a rechecagem deste para verificar se um objeto ainda está presente no ambiente, assim como a presença de possíveis objetos dinâmicos iriam deixar um rastro durante sua movimentação, sinalizando uma ocupação indevida do ambiente. Uma possível implementação é de realizar a rechecagem de objetos dentro de um alcance determinado do robô, "limpando" objetos que foram retirados, além de melhorar o comportamentamento em relação a obstáculos dinâmicos.
 
---
 
## 7. Possíveis Aplicações

Existem várias aplicações para detecção e contagem de objetos. Muitas aplicações envolvem questões de exploração, como a identificação e exploração do ambiente; manutenção, conhecendo o ambiente previamente é possível verificar a integridade de estruturas físicas como pilares, colunas ou canos; ou até aplicações na indústria, como, por exemplo, na indústria madeireira, onde teria a aplicação manter contabilizado as árvores plantadas.


---
 
## 8. Links Úteis

A seguir estão dispostos links referentes aos repositórios necessários para a execução do projeto:

https://github.com/mpenidol/navigation2.git

https://github.com/mpenidol/turtlebot3_simulations.git

O pacote de contagem pode ser executado através de um container ROS.

https://hub.docker.com/repository/docker/igorvieira10/object_counter/general


