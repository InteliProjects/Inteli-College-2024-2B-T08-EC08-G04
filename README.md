# Inteli - Instituto de Tecnologia e Liderança 

<p align="center">
<img src="https://www.inteli.edu.br/wp-content/uploads/2024/06/logo-inteli-3-768x420-1.png" alt="Inteli - Instituto de Tecnologia e Liderança" width="200">
</p>

# Grupo: J.A.R.B.A.S.

![Logo do grupo J.A.R.B.A.S.](./docs/static/img/jarbas_logo.png)

## Vídeo de demonstração

[Demonstração da Solução Final](https://youtu.be/Hy0UgpZZ7xw/)

## Documentação

Clique no botão abaixo para acessar a documentação:

[![Button Click]][Link]

[Button Click]: https://img.shields.io/badge/Documentação-37a779?style=for-the-badge
[Link]: https://inteli-college.github.io/2024-2B-T08-EC08-G04/


## :student: Integrantes:

- <a href="https://github.com/ahnina">Ana Clara Madureira Marques</a>
- <a href="https://www.linkedin.com/in/laura-p-bueno/">Laura Padilha Bueno</a>
- <a href="https://www.linkedin.com/in/lucasdeluccas/">Lucas Nogueira Storelli de Luccas</a>
- <a href="https://www.linkedin.com/in/m%C3%A1riovm/">Mário Ventura Medeiros</a>
- <a href="https://www.linkedin.com/in/murilo-prianti-0073111a1/">Murilo de Souza Prianti Silva</a>
- <a href="https://www.linkedin.com/in/olincosta/">Ólin Medeiros Costa</a>
- <a href="https://www.linkedin.com/in/raideoliveira/">Raí de Oliveira Cajé</a>
- <a href="https://www.linkedin.com/in/rodrigo-sales-07/">Rodrigo Sales Freire dos Santos</a>

## :teacher: Professores:

### Orientador

- <a href="https://www.linkedin.com/in/rafaelmatsuyama/">Rafael Matsuyama</a>

### Coordenadora

- <a href="https://www.linkedin.com/in/michele-bazana-de-souza-69b77763/">Michele Bazana de Souza</a>

### Instrutores

- <a href="https://www.linkedin.com/in/rodrigo-mangoni-nicola-537027158/">Rodrigo Mangoni Nicola</a>
- <a href="https://www.linkedin.com/in/gui-cestari/">Guilherme Cestari</a>
- <a href="https://www.linkedin.com/in/lisane-valdo/">Lisane Valdo</a>
- <a href="https://www.linkedin.com/in/diogo-martins-gon%C3%A7alves-de-morais-96404732/">Diogo Martins Gonçalves de Morais</a>
- <a href="https://www.linkedin.com/in/filipe-gon%C3%A7alves-08a55015b/">Filipe Gonçalves</a>
- <a href="https://www.linkedin.com/in/andr%C3%A9-leal-a57b2065/">André Leal</a>

## :pushpin: Problema

Existe uma necessidade crescente de oferecer mais segurança e companhia para pessoas idosas que vivem sozinhas, além de garantir uma resposta rápida em situações de emergência médica.

## :memo: Descrição do projeto

O J.A.R.B.A.S. é um sistema de robô autônomo de serviço integrado a um LLM (Large Language Model) que atua na casa de idosos como companhia e facilitador de acompanhamento médico. Como prova de conceito, todo o código deste repositório foi feito com base no TurtleBot3 Burger, que é capaz de emular a funcionalidade de seguir pessoas idosas dentro de casa e conversar com elas para ajudar a reduzir a solidão.

O robô é controlado por comandos de voz através de um aplicativo mobile, o que permite a interação robô X usuário com a possibilidade do robô conversar, seguir o idoso pela casa e, se necessário, chamar ajuda médica de forma automática ou sob solicitação.


## Instalação e execução

Para instalar e executar o projeto localmente, siga os passos abaixo:

### Pré-requisitos

- Git instalado e configurado
- Node.js instalado e configurado
- Python instalado e configurado

Clone o repositório e navegue até o diretório do projeto com os seguintes comandos:

```bash
git clone https://github.com/Inteli-College/2024-2B-T08-EC08-G04.git
cd 2024-2B-T08-EC08-G04
```

### Documentação

Para visualizar a documentação localmente, execute:

```bash
cd docs
npm i
npm start
```

### Execução do robô

#### 1. Conectar ao Robô via SSH e Executar o Bringup
Conecte-se ao robô via SSH e execute o comando de bringup para iniciar o TurtleBot:

```bash
ssh usuario@robo
sudo ros2 launch turtlebot3_bringup turtlebot3_robot.launch.py
```

```bash 
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=False map:=<caminho do mapa>.yaml
```

#### 2. Configurar o Docker no Computador
No computador onde o repositório foi clonado, navegue até `src/robot` e construa os serviços Docker Compose:

```bash
cd src/robot
docker compose build
```

#### 3. Executar a CLI
Utilize o Docker Compose para executar o script de navegação dentro do container Docker para posições pré-definidas:

```bash
docker compose run overlay "python cli/main.py select_pose"
```

#### 4. Instalar 3 pacotes
```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-turtlebot3
```

####  5. Instale o algoritmo de DDS utilizado pelo ROS:
```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

### Execução do frontend

Para executar o frontend localmente, use os seguintes comandos, a partir do diretório raiz do projeto:

```bash
cd src/frontend
npm i
npm run dev
```

## Histórico de lançamentos

### Sprint 1

- Entendimento de Négocios.
- Entendimento de UX.
- Definição do Problema.
- Arquitetura do Sistema.
- Analise de Impacto Ético.
  
### Sprint 2

- Prototipação de baixa fidelidade.
- Mapear um local pré definido.
- Interface CLI para movimentar o robô.

### Sprint 3

- Integrar o robô de serviço com LLM.
- Criação e integração do Banco de Dados.
- Criação do Mockup.
- Dockerização.

### Sprint 4

- Aprimoramento do serviço de LLM
- Adequação da interface às WCAG
- Aprimoramento da interface TTS & STT

### Sprint 5

- Finalização da integração
- Finalização da prova de conceito
- Finalização da documentação

## Licença

 <p xmlns:cc="http://creativecommons.org/ns#" xmlns:dct="http://purl.org/dc/terms/"><a property="dct:title" rel="cc:attributionURL" href="https://github.com/Inteli-College/2024-2B-T08-EC08-G04">J.A.R.B.A.S.</a> by <span property="cc:attributionName">Ana Clara Madureira Marques, Laura Padilha Bueno, Lucas Nogueira Storelli de Luccas, Ólin Medeiros Costa, Mário Ventura Medeiros, Murilo de Souza Prianti Silva, Raí de Oliveira Cajé, Rodrigo Sales Freire dos Santos </span> is licensed under <a href="https://creativecommons.org/licenses/by/4.0/?ref=chooser-v1" target="_blank" rel="license noopener noreferrer" style="display:inline-block;">CC BY 4.0<img style="height:22px!important;margin-left:3px;vertical-align:text-bottom;" src="https://mirrors.creativecommons.org/presskit/icons/cc.svg?ref=chooser-v1" alt=""><img style="height:22px!important;margin-left:3px;vertical-align:text-bottom;" src="https://mirrors.creativecommons.org/presskit/icons/by.svg?ref=chooser-v1" alt=""></a></p> 
