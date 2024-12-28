---
title: Mapeamento do espaço  
sidebar_label: Mapeamento do espaço  
sidebar_position: 1  
---

# Introdução

Nesta seção, será descrito como foi feita a configuração inicial do TurtleBot3 e como foi realizado o mapeamento de um ambiente. Para que o robô se movimente de forma autônoma, é necessário utilizar o sensor LIDAR, já presente no TurtleBot3, em conjunto com uma técnica chamada SLAM (Simultaneous Localization and Mapping). O SLAM permite que o robô construa um mapa do ambiente ao mesmo tempo que estima sua posição dentro desse mapa. Isso é fundamental para navegação em ambientes desconhecidos, pois permite ao robô identificar e evitar obstáculos, encontrando rotas seguras até o destino desejado.

# Primeiras configurações

Caso o TurtleBot3 Burger não esteja configurado com um sistema Ubuntu ou os drivers do LIDAR não estejam baixados, [siga este tutorial](https://rmnicola.github.io/m6-ec-encontros/setupturtle) que apresenta, passo a passo, como configurar o robô.

Após isso, os seguintes comandos devem ser executados no terminal em todas as entidades que irão interagir com o robô, ou seja, minimamente no robô e no computador que o controlará.

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-turtlebot3*
```

```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

```bash
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
```

```bash
cd /opt/ros/humble/share/turtlebot3_navigation2/param
```

Na pasta, modifique o arquivo `burger.yaml`. Na linha que está escrito `robot_model_type: "differential"`, altere para `robot_model_type: "nav2_amcl::DifferentialMotionModel"`.

Após seguir esse tutorial, o robô e o computador do usuário estarão configurados para utilizar a solução.

# Mapeamento

Para a movimentação do robô, o ambiente precisa ser mapeado previamente. O mapeamento foi realizado utilizando o seguinte comando:

```bash
roslaunch turtlebot3_slam turtlebot3_slam.launch
```

Esse comando abre um programa que, ao mover o robô, permite mapear o ambiente. Assim, foi mapeada uma pista que será utilizada durante o desenvolvimento como teste, e o resultado do mapeamento realizado foi:

<p align="center"> Figura 1 - Mapa </p>
<div align="center" class="zoom-image">
  <img src={require('../../../static/img/sprint-2/map.png').default} alt="10 falhas"/>
</div>
<p style={{textAlign: 'center'}}>Fonte: Elaborado pela equipe J.A.R.B.A.S.</p>

Com o ambiente mapeado, foi possível mover o robô apenas definindo o local de chegada, e o TurtleBot3 Burger irá automaticamente para o destino. Foram realizados alguns testes de movimentação, e observou-se que o robô consegue perceber obstáculos na pista, contorná-los ou até mesmo buscar outra rota para chegar ao destino.