---
title: Como Utilizar o Projeto
sidebar_label: Como Utilizar
sidebar_position: 1
---

# Introdução

Aqui será detalhado como você pode utilizar o projeto, incluindo como configurar e rodar cada parte do sistema. O projeto como um todo consiste no chatbot Rasa, serviço do robô, serviço do LLM, serviço do banco de dados e frontend. Abaixo, você encontrará instruções detalhadas para cada parte do projeto.

# Chatbot Rasa

O Rasa é um framework de código aberto para construir chatbots. O chatbot do projeto foi desenvolvido utilizando o Rasa e é responsável por receber as mensagens do usuário, processá-las e enviar para o serviço do robô. Para rodar o chatbot, siga os passos abaixo:

1. Vá até a pasta `src/rasa_chatbot` do projeto.
2. Como nós utilizamos o Docker para rodar o Rasa, você deve ter o Docker instalado na sua máquina. Caso não tenha, siga as instruções de instalação no site oficial do [Docker](https://docs.docker.com/get-docker/).
3. Execute o comando para criar a imagem do Rasa:
```bash
docker build -t rasa-chatbot .
```
4. Execute o comando para rodar o container do Rasa:
```bash
docker run -p 5005:5005 rasa-chatbot
```

Pronto! O chatbot está rodando e pronto para receber mensagens dos usuários. Para enviar mensagens para o chatbot, você pode enviar requisições HTTP para o endpoint `http://127.0.0.1:5005/webhooks/rest/webhook` com o seguinte payload:

```json
{
  "sender": "Usuário",
  "message": "Olá, como posso te ajudar?"
}
```

# Serviço do Robô

O serviço do robô é responsável por receber as mensagens do chatbot, processá-las e enviar para o serviço do LLM. Para rodar o serviço do robô, siga os passos abaixo:

1. Vá até a pasta `src/backend/api_robot` do projeto.
2. Conectar o computador ao robô através de SSH, através do comando:
```bash
ssh <nome-do-usuario>@<ip-do-robo>
```

3. Para utilizar esse serviço, é necessário rodar o seguinte comando no robô:

```bash
   ros2 launch turtlebot3_bringup robot.launch.py
```

4. Após isso, deverá rodar o mapa criado anteriormente no computador:

```bash
   ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=<caminho-do-mapa>.yaml
```

5. Tendo feito isso, você pode rodar a API do robô. Para isso, siga os passos abaixo:

- **Configuração do Ambiente**:
   - Certifique-se de que o ROS2 está configurado corretamente e que o robô pode ser controlado via `nav2_simple_commander`.
- **Execução do Servidor**:
   - Inicie o servidor FastAPI na pasta `src/backend/api_robot`:
     ```bash
     uvicorn main:app --reload
     ```
- **Acessando a API**:
- Documentação da API:
   - Swagger: [http://localhost:8000/docs](http://localhost:8000/docs)
   - Redoc: [http://localhost:8000/redoc](http://localhost:8000/redoc)
   - Para mais detalhes sobre a API, acesse esse outro tópico da documentação [aqui](servico-robo.md).


# Serviço do LLM

Esse serviço é responsável por administrar a integração entre o serviço do banco de dados, Rasa ChatboT, LLM do Gemini e serviços como Text to Speach e Speach To Text da IBM Cloud. Para rodar o serviço do LLM, siga os passos abaixo:

1. Vá até a pasta `src/backend/llm_service` do projeto.
2. Crie um ambiente virtual Python e instale as dependências do projeto:
```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```
3. Após isso, é importante fazer a configuração do arquivo `.env` com as variáveis de ambiente necessárias. Para isso, crie um arquivo `.env` dentro da pasta `src/backend/llm_service/app/core` e adicione as seguintes variáveis:
```bash
WATSON_API_KEY=<API_KEY>
WATSON_SERVICE_URL=<SERVICE_URL>
WATSON_ASSISTANT_ID=<ASSISTANT_ID>
STT_API_KEY=<API_KEY>
STT_SERVICE_URL=<SERVICE_URL>
TTS_API_KEY=<API_KEY>
TTS_SERVICE_URL=<SERVICE_URL>
GOOGLE_API_KEY=<API_KEY>
```

Essas variáveis são necessárias para a integração com o serviço do LLM. Para obter as chaves de API, você deve criar uma conta no [IBM Cloud](https://cloud.ibm.com/), [Google Cloud](https://cloud.google.com/) e seguir as instruções para obter as chaves de API.

4. Após configurar o arquivo `.env`, você pode rodar o serviço do LLM com o seguinte comando:
```bash
cd src/backend/llm_service/app
uvicorn main:app --reload
```

Pronto! O serviço do LLM está rodando e pronto para receber mensagens do chatbot e enviar para o serviço do banco de dados. Para mais detalhes sobre o serviço do LLM, acesse esse outro tópico da documentação [aqui](servico-llm.md).

# Serviço do Banco de Dados

Esse serviço é responsável por administrar a integração com o banco de dados. Para rodar o serviço do banco de dados, siga os passos abaixo:

1. Vá até a pasta `src/database` do projeto.
2. Configure o arquivo `.env` com as variáveis de ambiente necessárias. Para isso, crie um arquivo `.env` dentro da pasta `src/database` e adicione as seguintes variáveis:
```bash
SUPABASE_URL=<URL>
SUPABASE_KEY=<KEY>
DB_HOST=<HOST>
DB_PORT=<PORT>
DB_NAME=<NAME>
PORT=<PORT>
```

Essas variáveis são necessárias para a integração com o banco de dados. Para obter as chaves de API, você deve criar uma conta no [Supabase](https://supabase.io/) e seguir as instruções para obter as chaves de API.

3. Após configurar o arquivo `.env`, instale as dependências do projeto:
```bash
npm install
```
4. Após instalar as dependências, você pode rodar o serviço do banco de dados com o seguinte comando:
```bash
node server.js
```

Pronto! O serviço do banco de dados está rodando e pronto para se comunicar com o serviço do LLM. Para mais detalhes sobre o serviço do banco de dados, acesse esse outro tópico da documentação [aqui](servico-banco-dados.md).

# Frontend

O frontend é a interface do usuário do projeto, onde o usuário pode interagir com o chatbot. Para rodar o frontend, siga os passos abaixo:

1. Vá até a pasta `src/frontend` do projeto.
2. Instale as dependências do projeto:
```bash
npm install
```
3. Após instalar as dependências, você pode rodar o frontend com o seguinte comando:
```bash
npm run dev
```

Pronto! O frontend está rodando e pronto para ser acessado no navegador. Para acessar o frontend, abra o navegador e acesse o endereço `http://localhost:5173`.

# Conclusão

Com isso, você aprendeu como utilizar o projeto, incluindo como configurar e rodar cada parte do sistema. Se você tiver alguma dúvida ou problema, sinta-se à vontade para entrar em contato com a equipe de desenvolvimento. Só é importante também relembrar que cada parte deve ser rodada em um terminal e nos fizemos os testes do projeto nso sistemas operacionais Windows 11 e Ubuntu 24. Esperamos que você tenha uma ótima experiência utilizando o projeto!






