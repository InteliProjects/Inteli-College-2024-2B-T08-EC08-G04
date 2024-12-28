---
title: Serviço Robô
sidebar_label: Serviço Robô
sidebar_position: 2
---

# Introdução

Aqui será detalhado como o robô foi utilizado no projeto, incluindo a integração, setup inicial, e como o robô funciona.

# Setup Inicial

É importante relembrar que nessa versão POC (Proof of Concept) do projeto, utilizamos o robô [Turtlebot 3](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) que tem como base o ROS, Robot Operating System, que é um framework para desenvolvimento de robôs. Além disso, o robô possui um sensor LIDAR, que é um sensor de distância que utiliza laser para medir a distância de objetos ao redor do robô. É importante lembrar também que para fazer esse tutorial, é necessário ter o robô Turtlebot 3 Burger configurado com um sistema Ubuntu e os drivers do LIDAR baixados e um computador com ROS 2 humble instalado que será conectado ao robô através de SSH. Caso o TurtleBot3 Burger não esteja configurado com um sistema Ubuntu ou os drivers do LIDAR não estejam baixados, [siga este tutorial](https://rmnicola.github.io/m6-ec-encontros/setupturtle) que apresenta, passo a passo, como configurar o robô.

Após isso, os seguintes comandos devem ser executados no terminal em todas as entidades que irão interagir com o robô, ou seja, no robô e no computador que o controlará.

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

# Funcionamento

Caso queira apenas mover o robô, é possível utilizar o comando abaixo:

Em um terminal no robô, rode o comando para iniciar a operação do pacote do turtlebot3:

```bash

ros2 launch turtlebot3_bringup robot.launch.py

```

Em um terminal no computador, rode o comando para iniciar o controle do robô:

```bash

ros2 run turtlebot3_teleop teleop_keyboard

```

Com isso, o robô irá se mover de acordo com os comandos enviados pelo teclado do computador. É importante lembrar que o robô precisa estar conectado na mesma rede (Wifi) que o computador para que o controle funcione.

# Movimentação Autônoma

Para a movimentação do robô, o ambiente precisa ser mapeado previamente. O mapeamento foi realizado utilizando o seguinte comando:

```bash
roslaunch turtlebot3_slam turtlebot3_slam.launch
```

Esse comando abre um programa que, ao mover o robô, permite mapear o ambiente. Assim, no nosso contexto foi mapeada uma pista que será utilizada durante o desenvolvimento como teste, e o resultado do mapeamento realizado foi:

<p align="center"> Figura 1 - Mapa </p>
<div align="center" class="zoom-image">
  <img src={require('../../static/img/sprint-2/map.png').default} alt="10 falhas"/>
</div>
<p style={{textAlign: 'center'}}>Fonte: Elaborado pela equipe J.A.R.B.A.S.</p>

Apesar do mapa ser pequeno, qualquer ambiente pode ser mapeado atráves desse comando. Com o ambiente mapeado, basta rodar o comando abaixo para iniciar a movimentação autônoma do robô com base no mapa gerado:

```bash


ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=<path_to_map.yaml>

```

Apos isso, basta definir o destino do robô e ele irá automaticamente para o destino. Foram realizados alguns testes de movimentação, e observou-se que o robô consegue perceber obstáculos na pista, contorná-los ou até mesmo buscar outra rota para chegar ao destino.

# CLI

Para facilitar a movimentação do robô, foi criado um CLI (Command Line Interface) que permite o usuário controlar o robô de forma mais simples. Para utilizar o CLI é muito importante que se esteja no caminho correto da pasta onde está o arquivo "cli.py".

O caminho está assim **"src/workspace/cli/cli/cli.py"**
OBS:(Para que se consiga movimentar o robô, deve ter aberto terminais paralelos para rodar o mapa e o bringup no robô)

-No robô:
```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=False map:=<caminho do mapa>.yaml
ros2 launch turtlebot3_bringup robot.launch.py
```

-No computador:
```bash
python3 cli.py
```

Após iniciar a CLI, selecione uma das ações a seguir:

- **Frente:** Movimenta o TurtleBot para a frente com velocidade linear de 0.2 m/s.
    
- **Trás:** Movimenta o TurtleBot para trás com velocidade linear de -0.2 m/s.
  
- **Esquerda:** Rotaciona o TurtleBot no sentido anti-horário com velocidade angular de 0.5 rad/s.

- **Direita:** Rotaciona o TurtleBot no sentido horário com velocidade angular de -0.5 rad/s.

- **Emergência (Parar Funcionamento):** Interrompe o movimento do TurtleBot imediatamente, definindo as velocidades linear e angular como 0.

- **Ir para ponto:** Lista pontos predefinidos para onde o TurtleBot pode ir. O usuário escolhe um ponto e o TurtleBot navega automaticamente até ele.
    
- **Sair:** Encerra a CLI.

Cada ponto possui coordenadas específicas para navegação. Para mais detalhes sobre CLI, acesse esse outro tópico da documentação [aqui](../Sprint%202/Mov_robo/cli.md).

# API do Robô

Para facilitar a integração do robô com outros sistemas, foi criada uma API RESTful que permite o controle do robô através de requisições HTTP. Para utilizar a API, é necessário que o robô esteja em execução e que a API esteja rodando no computador que controla o robô.

Relendo o que foi dito anteriormente, para rodar o robô, é necessário rodar o seguinte comando no robô:

```bash
   ros2 launch turtlebot3_bringup robot.launch.py
```

- Após isso, deverá rodar o mapa criado anteriormente:

```bash
   ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=<caminho-do-mapa>.yaml
```

Para rodar a API, siga os passos abaixo:

1. **Configuração do Ambiente**:
   - Certifique-se de que o ROS2 está configurado corretamente e que o robô pode ser controlado via `nav2_simple_commander`.

2. **Execução do Servidor**:
   - Inicie o servidor FastAPI na pasta `src/backend/api_robot`:
     ```bash
     uvicorn main:app --reload
     ```

3. **Acessando a API**:
   - Documentação da API:
     - Swagger: [http://127.0.0.1:8000/docs](http://127.0.0.1:8000/docs)
     - Redoc: [http://127.0.0.1:8000/redoc](http://127.0.0.1:8000/redoc)
   - Exemplo de requisição:
     ```bash
     curl -X POST "http://127.0.0.1:8000/go_to_position/0"
     ```

#### **Rotas**
1. **POST /go_to_position/{ `position_index` }**
   - **Descrição**: Comanda o robô para ir até um waypoint específico.
   - **Parâmetros**:
     - `position_index`: Índice do waypoint na lista de posições definidas no controlador.
   - **Possíveis Retornos**:
     - Sucesso (`200 OK`): Robô chegou ao waypoint solicitado.
     - Erro de cliente (`400 Bad Request`): O índice fornecido é inválido.
     - Serviço indisponível (`503 Service Unavailable`): O controlador do robô não foi inicializado.
     - Erro interno (`500 Internal Server Error`): Falha inesperada ao processar a solicitação

Para mais detalhes sobre a API, acesse esse outro tópico da documentação [aqui](../Sprint%203/api_robot.md).

# Conclusão

Com a integração do robô Turtlebot 3 ao projeto, foi possível realizar testes de movimentação autônoma e controle manual do robô. A integração do robô no sistema foi feita atráves de uma action no Rasa, que detecta quando alguem fala para o robô ir para algum lugar ele chama a API (mais detalhes na documetação do [rasa](chatbot.md)). Além disso, a criação de uma API RESTful facilitou a integração do robô com outros sistemas, permitindo o controle do robô através de requisições HTTP. Com isso, o robô se tornou uma parte essencial do projeto, possibilitando a realização de testes e validações de forma mais eficiente e prática.


