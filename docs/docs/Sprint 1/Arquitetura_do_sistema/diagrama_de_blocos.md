---
title: Diagrama de blocos
sidebar_label: Diagrama de blocos da solução
sidebar_position: 1
---

# Diagrama de blocos da solução

&emsp;&emsp;O diagrama de blocos basicamente consiste em representar um sistema ou processo através de blocos, sendo que cada parte desse sistema é um bloco. Esses blocos são interligados por linhas que representam a comunicação entre eles. O diagrama de blocos é uma ferramenta muito útil para entender o funcionamento de um sistema, pois ele permite visualizar de forma clara e objetiva como as partes de um sistema se relacionam entre si. No momento, o diagrama não descreve quais tecnologias serão utilizadas em cada bloco, porque o projeto está na fase de definição da arquitetura do sistema e esses aspectos serão definidos nas próximas etapas do projeto.

<p align="center"> Figura 1 - Diagrama de blocos </p>
<div align="center" class="zoom-image">
  <img src={require('../../../static/img/sprint-1/Diagrama_de_blocos.png').default} alt="10 falhas"/>
</div>
<p style={{textAlign: 'center'}}>Fonte: Elaborado pela equipe J.A.R.B.A.S.</p>

## Descrição dos blocos

- **Frontend**: É a parte do sistema que interage diretamente com o usuário. É responsável por receber as entradas do usuário e exibir as saídas do sistema. A solução vai ser utilizado para a utilização da conversa com o chatbot e visualizações dos relatórios de saúde. 
    - **Relação com outros blocos**: O frontend se comunica com o backend para enviar as entradas do usuário (ex: voz) e receber as saídas do sistema (ex: resposta do chatbot) e também recebe dados da saúde do idoso e exibe para o usuário.

- **Backend**: É o cérebro do sistema, faz todo o processamento e formatação dos dados. É responsável fazer a ponte entre o LLM e o frontend, além de se comunicar com o banco de dados e também recebe os dados de serviços externos de saúde.
    - **Relação com outros blocos**: O backend se comunica com o frontend para receber as entradas do usuário (ex: voz) e enviar as saídas do sistema (ex: resposta do chatbot) e também se comunica com o banco de dados para armazenar e recuperar os dados coletados e com os serviços de saúde externos para processar esses dados, salvar eles e exibir para o usuário.

- **Banco de dados**: É aonde armazenamos os dados do sistema.
    - **Relação com outros blocos**: O banco de dados se comunica com o backend para armazenar e recuperar os dados coletados.
  
- **LLM**: Esse é o Large Language Model, que é o modelo de linguagem que vai ser utilizado para a conversa com o chatbot.
    - **Relação com outros blocos**: O LLM se comunica com o backend para enviar as entradas do usuário (ex: voz), receber e processar as saídas do sistema (ex: resposta do chatbot).

- **Serviços de saúde externos**: Aqui está generalizado, mas pode ser qualquer serviço que forneça dados de saúde do idosos, como por exemplo, um smartwatch fornecendo dados de batimentos cardíacos e pressão arterial. Esses dados são utilizados para enriquecer o relatório de saúde do idoso, que é gerado também com ajudado do chatbot.
    - **Relação com outros blocos**: Os serviços de saúde externos se comunicam com o backend, que recebe os dados de saúde do idoso, processa esses dados, salva eles e exibe para o usuário.

- **Turtlebot**: É um robô de aprendizado, que utiliza o ROS (Robot Operating System) para ser controlado e interagir com o ambiente. O sistema terá um software embarcado para fazer mapeamento do local para ele poder se locomover de forma autônoma e interagir com o idoso.
    - **Relação com outros blocos**: O Turtlebot se comunica com o Lidar para poder mapear o ambiente e com a CLI para podermos configurar o Turtlebot e controlar ele caso necessário.

- **Lidar**: LiDAR (Light Detection and Ranging) é uma tecnologia de sensoriamento remoto que usa feixes de laser para medir distâncias e movimentos em tempo real. O sistema é utilizado para mapear o ambiente para o Turtlebot poder se locomover de forma autônoma.
    - **Relação com outros blocos**: O Lidar se comunica com o Turtlebot para enviar os dados de mapeamento do ambiente.

- **CLI**: É uma interface de linha de comando que permite a comunicação com o Turtlebot. Ela é utilizada para configurar o Turtlebot e controlar ele caso necessário.
    - **Relação com outros blocos**: A CLI se comunica com o Turtlebot para configurar ele e controlar ele caso necessário.

# Conclusão

&emsp;&emsp;Esse diagrama foi feito com o objetivo de representar incialmente a arquitetura do sistema, para ter uma visão geral de como as partes do sistema se relacionam entre si. A partir desse diagrama, será definido quais tecnologias serão utilizadas em cada bloco e como eles se comunicam entre si. Esse diagrama será atualizado conforme o projeto avança e novas informações são obtidas.