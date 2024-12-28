---
title: Atualizações do Chatbot
sidebar_position: 1
---

## Introdução

Durante a **Sprint 4**, o chatbot do J.A.R.B.A.S foi aprimorado com a implementação da etapa de **onboarding**, que introduz os usuários ao sistema e configura interações personalizadas. Este documento descreve as principais atualizações realizadas, incluindo regras, intenções e ações implementadas no sistema.

---

## Atualizações Estruturais

### 1. Regras (`data/rules.yml`)

O arquivo `rules.yml` contém as **regras que definem o comportamento do chatbot**. Exemplos implementados incluem:

- **Onboarding inicial** para introduzir o usuário ao sistema.
- **Resposta personalizada** para despedidas e questionamentos sobre o chatbot.
- **Interações contextuais**, como envio do robô para um local específico.
- **Fallback** para lidar com entradas não compreendidas.

#### Exemplo de regras

```yaml
version: "3.1"

rules:

  - rule: Onboarding
    steps:
      - action: action_session_start
      - action: action_listen 

  - rule: Salvar Nome
    steps:
      - intent: ask_name
      - action: action_save_name
      - slot_was_set:
          - name: True
      - action: action_listen 

  - rule: Salvar Saudação
    steps:
      - intent: escolha_saudacao
      - action: action_save_saudacao
      - slot_was_set:
          - saudacao: True
      - action: action_listen 

  - rule: Ir para a Cozinha
    steps:
      - intent: ir_para_cozinha
      - action: action_ir_para_cozinha
```

---

### 2. Intenções (`data/nlu.yml`)

O arquivo `nlu.yml` define **intenções e exemplos de frases**, que permitem ao chatbot identificar a intenção do usuário com base em inputs fornecidos.

#### Intenções Implementadas

1. **ask_name**: Identificar o nome do usuário.
2. **escolha_saudacao**: Configurar uma saudação personalizada.
3. **escolha_preferencias**: Registrar os temas preferidos do usuário.

#### Exemplo de configuração de intenção

```yaml
version: "3.1"

nlu:
  - intent: ask_name
    examples: |
      - Meu nome é [Rafael Oliveira](name)
      - Pode me chamar de [Isabela](name)

  - intent: ir_para_cozinha
    examples: |
      - me leve para a cozinha
      - vá até a cozinha

  - intent: escolha_saudacao
    examples: |
      - prefiro que diga [e aí Laura](saudacao)
      - escolha dizer [oi João tudo bem](saudacao)

  - intent: escolha_preferencias
    examples: |
      - Eu gosto de [futebol](temas)
      - Adoro [tecnologia](temas)
```

---

### 3. Ações (`actions/actions.py`)

No arquivo `actions.py`, definimos **as ações que o chatbot executa** em resposta às intenções do usuário. As ações podem incluir respostas personalizadas, armazenamento de dados ou realização de tarefas específicas.

#### Ações Implementadas

- **`action_session_start`**: Inicia uma nova sessão com mensagem de boas-vindas.
- **`action_save_name`**: Salva o nome do usuário em um arquivo JSON e no slot correspondente.
- **`action_save_saudacao`**: Configura a saudação preferida pelo usuário.
- **`action_save_preferencias`**: Armazena os temas preferidos em um arquivo JSON.
- **`action_ir_para_cozinha`**: Direciona o robô para a cozinha.

#### Exemplo de implementação de ação

```python
from rasa_sdk import Action, Tracker
from rasa_sdk.executor import CollectingDispatcher
from rasa_sdk.events import SlotSet, SessionStarted
import json
import os

class ActionSaveName(Action):
    def name(self) -> str:
        return "action_save_name"

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: dict):
        user_name = next(tracker.get_latest_entity_values("name"), None)
        if user_name:
            with open("user_data.json", "w") as f:
                json.dump({"name": user_name}, f)
            dispatcher.utter_message(template="utter_confirm_name")
            return [SlotSet("name", user_name)]
        else:
            dispatcher.utter_message(text="Desculpe, não consegui captar o nome. Pode repetir?")
            return []
```

As mudanças estruturais na lógica do chatbot, possibilitou um diálogo introdutório com o usuário. Um exemplo da estrutura final da conversação pode ser acompanhado a seguir.

- J.A.R.B.A.S

  Olá! Eu sou o Jarbinhas, sua nova companhia. Estou aqui para ajudar a tornar seus dias mais agradáveis, lembrá-lo de tomar a medicação e cuidar de você. Agora vamos começar a nos conhecer melhor!"

- Usuário

 Me chame de Ana

- J.A.R.B.A.S

  Entendido, Ana! É muito bom te conhecer e fazer parte do seu dia a dia. Agora, me conte como você prefere que eu o cumprimente pela manhã? Por exemplo,
  posso dizer 'olá, Ana, tudo bem?' ou algo que você goste mais."

- Usuário

  Diga como vai Ana

- J.A.R.B.A.S

  Certo Ana, sua saudação preferida foi salva com sucesso! Durante nossa interação, eu também posso conversar sobre assuntos que você gosta, 
  e trazer atualizações. Quais são algumas coisas que você gosta de falar e tem interesse?"

- Usuário

  Eu gosto de futebol e design

- J.A.R.B.A.S

  Obrigado Ana, suas preferências foram salvas com sucesso. Agora estamos prontos, e a etapa de onboarding acabou.
  Daqui em diante, estarei sempre aqui para ajudar e fazer companhia. Se precisar de algo, basta me chamar.


Durante esse diálogo, a lógica aplicada permite que as informações de nome, saudação e temas de preferência sejam salvas em um arquivo json, para serem utilizados pelo sistema posteriormente.


---

## Conclusão

Essas implementações tornam o chatbot mais interativo e eficiente, proporcionando uma experiência personalizada ao usuário. Com as regras, intenções e ações descritas, o J.A.R.B.A.S está preparado para executar tarefas específicas e lidar com diferentes interações.
