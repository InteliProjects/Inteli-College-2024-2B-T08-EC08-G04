# Documentação da API de Navegação com ROS2 e FastAPI

Este projeto é uma transcrição de um script originalmente usado na CLI para controlar um robô via ROS2, adaptado para uma API RESTful usando FastAPI. A motivação para esta adaptação é permitir que outros sistemas e backends possam interagir com o robô de forma mais fácil e flexível, expondo os controles através de endpoints HTTP.

---

## Estrutura do Projeto

O projeto segue uma organização modular para facilitar a manutenção e a expansão:

1. **`main.py`**: Configura a aplicação FastAPI e gerencia a integração com o ROS2.
2. **`controller`**: Contém a lógica para controlar o robô, incluindo a definição de posições iniciais e waypoints.
3. **`app`**: Define as rotas da API e integra o controlador do robô aos endpoints HTTP.

---

## Funcionamento Geral

### 1. Inicialização do ROS2

- No turtlebot3 burger, deve-se rodar o seguinte código:

```bash
   ros2 launch turtlebot3_bringup robot.launch.py
```

- Após isso, deverá rodar o mapa criado anteriormente:

```bash
   ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=<caminho-do-mapa>.yaml
```

---

### 2. Controlador do Robô (`controller`)

O controlador é responsável por:

1. **Definir a posição inicial do robô**:
   - Um ponto de partida é configurado para o robô, incluindo suas coordenadas espaciais (x, y, z) e orientação.

2. **Criar Waypoints**:
   - Uma lista de waypoints (pontos de caminho) é definida. Cada waypoint possui coordenadas e uma orientação específica.
   - Esses pontos são usados para guiar o robô para posições pré-determinadas.

3. **Navegação entre Posições**:
   - O controlador recebe um índice de waypoint e move o robô até a posição correspondente.
   - Durante o movimento, o feedback é monitorado continuamente até que o robô conclua a tarefa.

---

### 3. API RESTful (`routes`)

Os endpoints disponíveis permitem a interação com o controlador do robô de maneira simples:

#### **Eventos de Inicialização e Finalização**
- **Startup**: O controlador do robô é inicializado quando a aplicação inicia.
- **Shutdown**: Os recursos do controlador são liberados quando a aplicação é encerrada.

#### **Rotas**
1. **POST /go_to_position/{ `position_index` }**
   - **Descrição**: Comanda o robô para ir até um waypoint específico.
   - **Parâmetros**:
     - `position_index`: Índice do waypoint na lista de posições definidas no controlador.
   - **Possíveis Retornos**:
     - Sucesso (`200 OK`): Robô chegou ao waypoint solicitado.
     - Erro de cliente (`400 Bad Request`): O índice fornecido é inválido.
     - Serviço indisponível (`503 Service Unavailable`): O controlador do robô não foi inicializado.
     - Erro interno (`500 Internal Server Error`): Falha inesperada ao processar a solicitação.

---

## Como Utilizar

1. **Configuração do Ambiente**:
   - Certifique-se de que o ROS2 está configurado corretamente e que o robô pode ser controlado via `nav2_simple_commander`.

2. **Execução do Servidor**:
   - Inicie o servidor FastAPI:
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

---

## Benefícios da Adaptação para FastAPI

- **Integração com outros sistemas**: A API REST permite que qualquer backend ou aplicação com suporte a HTTP possa controlar o robô.
- **Flexibilidade**: Os controles que antes eram limitados a comandos de CLI agora estão expostos como endpoints reutilizáveis.
- **Manutenção Simplificada**: A modularidade do projeto facilita a extensão e atualização da lógica de navegação.
