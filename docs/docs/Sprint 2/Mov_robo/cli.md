---
title: Movimentação do Robô (CLI)
sidebar_label: Movimentação do robô
sidebar_position: 1
---

# O que é uma CLI (Interface Line Command)

&emsp;&emsp; Uma interface de linha de comando (CLI) é um mecanismo de software que você usa para interagir com o sistema operacional usando o teclado. Com uma interface de linha de comando, você pode inserir comandos de texto para configurar, navegar ou executar programas em qualquer servidor ou sistema de computador. Todos os sistemas operacionais, incluindo Linux, macOS e Windows, fornecem uma CLI para interação mais rápida com o sistema.

# CLI integrado ao nosso projeto (J.A.R.B.A.S)

&emsp;&emsp; Primeiro, temos que ter uma introdução do Nav2 (Framework de navegação utilizado em conjunto com o ROS2 para a criação do mapa que o robô deve percorrer). Com o Nav2, criamos o nosso mapa (um mapa simulado no laboratório para entendermos a nevegação do robô) simulando a casa do paciente que é por onde o robô vai percorrer. Depois da criação do mapa, foi criada a CLI para que o robô navegasse através dos pontos que cetamos para ele percorrer dentro do local especificado, estes pontos de naevgação vão ser os "SPOTS", pontos fixos que o robô deve ir e servirá também como um ponto de carregamento da bateria do robô.

## Configuração do ambiente de NAV2 para criação do mapa

&emsp;&emsp; Para usar o Nav2, deve-se primeiro garantir que o ROS está instalado e configurado adequadamente.
**OBS:(Devemos nos atentar para estarmos na raiz do projeto "2024-2B-T08-EC08-G04")**

1 - Instalar 3 pacotes:
```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-turtlebot3*
```

2 - Instale o algoritmo de DDS utilizado pelo ROS:
```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

3 - PARTE IMPORTANTE - Vá para a pasta **/opt/ros/humble/share/turtlebot3_navigation2/param**.
Depois disso, **modificar o arquivo burguer.yml**. Na linha em que está escrito **robot_model_type: "differential"** deve-se **mudar** para robot_model_type: **"nav2_amcl::DifferentialMotionModel"**. 

## Usando o Simple Commander API para interagir com o robô

&emsp;&emsp; O Simple Commander é uma API feita em Python para interagir com as ações e serviços do Nav2 sem precisar criar os nós para isso diretamente. Com ele, é possível setar a pose inicial, passar pontos como objetivo e até mesmo criar uma lista de pontos pelos quais o robô obrigatoriamente deve passar.

Setup simple do commander:
OBS:(Temos que instalar alguns pacotes)
```bash
sudo apt install ros-humble-nav2-simple-commander ros-humble-tf-transformations python3-transforms3d
```

# CLI usada no JARBINHAS (robô)

## Descrição

&emsp;&emsp; Este projeto é uma CLI (Interface de Linha de Comando) em Python que permite controlar um robô TurtleBot3 utilizando ROS 2. A CLI fornece comandos básicos de movimentação e navegação para o robô, bem como pontos predefinidos para onde ele pode se mover automaticamente.

## Estrutura do código

- **TeleopTurtle:** Classe responsável por publicar comandos de velocidade linear e angular para o TurtleBot.

- **pontos:** Lista de pontos de destino predefinidos, cada um com coordenadas x, y, e z.

- **create_pose_stamped:** Função que cria um objeto PoseStamped, necessário para definir a posição e orientação alvo do robô.

- **Comandos CLI:** Fornecidos usando o typer para receber comandos do usuário e mover o TurtleBot conforme a entrada.

## Pré requisitos (IMPORTANTE)

- **ROS 2:** Instalação do ROS 2 com os pacotes necessários (rclpy, geometry_msgs, turtlesim, nav2_simple_commander).

- **Python 3:** Certifique-se de ter o typer, inquirer, tf_transformations e outros pacotes listados no código.

- **TurtleBot3:** Configuração e conectividade com o TurtleBot3 no ambiente ROS 2.

## Inicialização do código

&emsp;&emsp; É muito importante que se esteja no caminho correto da pasta onde está o arquivo "cli.py".
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
## Comando de movimento

&emsp;&emsp; Após iniciar a CLI, selecione uma das ações a seguir:

- **Frente:** Movimenta o TurtleBot para a frente com velocidade linear de 0.2 m/s.
    
- **Trás:** Movimenta o TurtleBot para trás com velocidade linear de -0.2 m/s.
  
- **Esquerda:** Rotaciona o TurtleBot no sentido anti-horário com velocidade angular de 0.5 rad/s.

- **Direita:** Rotaciona o TurtleBot no sentido horário com velocidade angular de -0.5 rad/s.

- **Emergência (Parar Funcionamento):** Interrompe o movimento do TurtleBot imediatamente, definindo as velocidades linear e angular como 0.

- **Ir para ponto:** Lista pontos predefinidos para onde o TurtleBot pode ir. O usuário escolhe um ponto e o TurtleBot navega automaticamente até ele.
    
- **Sair:** Encerra a CLI.

## Lista de pontos

&emsp;&emsp; Cada ponto possui coordenadas específicas para navegação. A lista de pontos predefinidos é a seguinte:

- **Ponto 1:** (x=1.45, y=-0.5, z=0.0)
    
- **Ponto 2:** (x=0.84, y=0.35, z=0.0)
    
- **Ponto 3:** (x=0.0, y=0.0, z=0.2)

&emsp;&emsp; Para enviar o robô a um desses pontos, basta selecionar a opção "Ir para ponto" e escolher o ponto desejado.

## Estrutura do código

&emsp;&emsp; Dentro da pasta de cli, vai ter o arquivo cli.py onde encontramos esse código para aticar o robô e estabelecer a comunicação.

```bash
- app: Aplicação Typer para a CLI.
- control(): Função principal que inicializa o nó ROS 2 e ativa o controle CLI.
- TeleopTurtle: Classe para publicação de comandos de velocidade.
- create_pose_stamped(): Função para criar uma pose com base nas coordenadas do ponto.
```

## Exemplo de Uso

&emsp;&emsp; Aqui, plotamos a CLI para que o cliente consiga  movimentar o robô "manualmente".

```bash
$ python turtle_cli.py control
Controle do TurtleBot3
? Selecione uma ação: (use ↑/↓ para navegar)
  - Frente
  - Trás
  - Esquerda
  - Direita
  - Emergência (Parar Funcionamento)
  - Ir para ponto
  - Sair
```

&emsp;&emsp; Caso ele selecione "Ir para ponto" o robô irá para um ponto pré setado préviamente na programação.

```bash
? Selecione o ponto de destino: (use ↑/↓ para navegar)
  - Ponto 1
  - Ponto 2
  - Ponto 3
```

## Tratamento de Erros

&emsp;&emsp; Caso ocorra um erro durante o controle do robô, o sistema:

1 - Exibe uma mensagem de erro.
    
2 - Interrompe imediatamente a movimentação do robô para garantir a segurança.

## Encerrando o Programa

&emsp;&emsp; Para encerrar, selecione "Sair" ou utilize Ctrl+C no terminal.

# Referências 
[1] AMAZON. O que é CLI e como que funciona?. Disponível em: [https://aws.amazon.com/pt/what-is/cli/](https://aws.amazon.com/pt/what-is/cli/). Acesso em: 06 novembro. 2024.
