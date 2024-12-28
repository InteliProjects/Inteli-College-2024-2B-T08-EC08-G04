---
title: Atualizações do Chatbot
sidebar_position: 1
---

## Introdução

Durante a **Sprint 5**, o chatbot do J.A.R.B.A.S foi aprimorado com a implementação da etapa de **checkup**, responsável por fazer perguntas de rotina que serão enviadas para o médico responsável. Este documento descreve as principais atualizações realizadas, incluindo regras, intenções e ações implementadas no sistema.

---

## Atualizações Estruturais

### 1. Regras (`data/rules.yml`)

O arquivo `rules.yml` contém as **regras que definem o comportamento do chatbot**. Exemplos implementados incluem:

- **Onboarding inicial** para introduzir o usuário ao sistema.
- **checkup diário** para garantir o acompanhamento do paciente
- **Resposta personalizada** para despedidas e questionamentos sobre o chatbot.
- **Interações contextuais**, como envio do robô para um local específico.

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

  - rule: Checkup
    steps:
      - intent: intro_checkup
      - action: action_intro_checkup

  - rule: Save dor
    steps:
      - intent: ask_dor
      - action: action_save_pain
      - action: action_listen  # Escuta por mais input

  - rule: Save alimentacao
    steps:
      - intent: ask_alimentation
      - action: action_save_alimentation
      - action: action_listen  # Escuta por mais input

  - rule: Save emocao
    steps:
      - intent: ask_emotion
      - action: action_save_emotion


  - rule: Ir para a cozinha
    steps:
      - intent: ir_para_cozinha
      - action: action_ir_para_cozinha  

  - rule: Emergency
    steps:
    - intent: emergencia
    - action: action_emergencia
```

---

### 2. Intenções (`data/nlu.yml`)

O arquivo `nlu.yml` define **intenções e exemplos de frases**, que permitem ao chatbot identificar a intenção do usuário com base em inputs fornecidos.

#### Intenções Implementadas

1. **ask_name**: Identificar o nome do usuário.
2. **escolha_saudacao**: Configurar uma saudação personalizada.
3. **escolha_preferencias**: Registrar os temas preferidos do usuário.
4. **intro_checkup**: Identificar quando deve ser inicialiazada a etapa de checkup.
5. **ask_dor**: Registras as dores que o usuário teve no dia, caso tenha tido. 
6. **ask_alimentation**: Registras a alimentação do usuário.
7. **ask_emotion**: Registras ossentimentos e sensações do usuário no dia.
8. **emergency**: Identificar uma situação de emergência

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
  
  - intent: intro_checkup
    examples: |
      - checkup
      - começar checkup

  - intent: ask_dor
    examples: |
      - estou com dor de cabeça
      - eu senti dor nas costas
      - eu tive enxaqueca
      - fiquei com dor no peito pela manhã
      - estou sentindo dor abdominal
      - tenho uma dor forte na perna
      - estou com dor nos ombros
      - sinto uma dor leve no joelho
      - acordei com dor na lombar
      - ontem à noite tive dor de dente
      - agora estou com dor nas articulações
      - sinto uma dor aguda no abdômen
      - tive uma dor intensa no braço
      - estou com dor de garganta
      - sinto dor crônica nas costas
      - percebi uma dor no tornozelo
      - estou com uma dor pulsante na cabeça
      - estou sentindo dor muscular depois do exercício

  - intent: ask_alimentation
    examples: |
      - minha alimentação hoje foi boa
      - eu comi de forma saudável hoje
      - minha dieta foi equilibrada hoje
      - eu me alimentei bem
      - tive uma boa refeição durante o dia
      - comi bastante fruta e vegetais
      - minha alimentação hoje foi ruim
      - acabei comendo muita besteira hoje
      - eu exagerei na alimentação hoje
      - não comi direito hoje
      - pulei algumas refeições hoje
      - comi de forma desregulada
      - tomei um café da manhã reforçado
      - almocei bem, mas jantei mal
      - fiz todas as refeições corretamente hoje
      - minha alimentação foi leve e balanceada
      - comi muito carboidrato hoje
      - tive uma alimentação rica em proteínas
      - estou satisfeito com o que comi hoje
      - acho que minha alimentação deixou a desejar

  - intent: ask_emotion
    examples: |
      - hoje me senti feliz
      - estou triste hoje
      - fiquei muito ansioso durante o dia
      - tive um dia calmo e tranquilo
      - me senti estressado com o trabalho
      - estou muito animado hoje
      - tive um dia bem cansativo
      - me senti frustrado com algumas coisas
      - fiquei contente com o que aconteceu hoje
      - tive um dia cheio de emoções
      - me senti sozinho e desanimado
      - estou grato pelo dia de hoje
      - tive momentos de raiva hoje
      - fiquei preocupado com algumas situações
      - hoje me senti mais motivado
      - me senti aliviado depois da reunião
      - estou desapontado com o que aconteceu
      - tive um dia muito estressante
      - fiquei bastante entediado durante o dia
      - me senti realizado com as conquistas de hoje


  - intent: emergencia
    examples: |
      - preciso de ajuda
      - estou em perigo
      - me ajude
      - socorro
      - estou passando mal
      - me ajuda por favor
      - ligue para o meu contato de emergência
      - preciso de ajuda médica
      - liga para o meu médico
      - me ajude agora
      - ligue para a minha família
      - ligue para a minha filha
      - ligue para o meu filho
      - ligue para o meu neto
      - ligue para a minha neta
      - ligue para o meu vizinho
      - ligue para o meu amigo
      - ligue para o meu parente
      - ligue para o
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
- **`action_intro_checkup`**: Inicia o processo de checkup.
- **`action_save_pain`**: Salva as dores registradas pelo usuário em um arquivo JSON
- **`action_save_alimentation`**: Salva os dados de alimentação egistradas pelo usuário em um arquivo JSON
- **`action_save_emotion`**: Salva aa descrição de emoções e sensações registradas pelo usuário em um arquivo JSON.
- **`action_emergencia`**: inicia a resposta emergencial para o médico.

#### Exemplo de implementação de ação

```python

class CustomSessionStart(Action):
    def name(self) -> str:
        return "action_session_start"

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: dict):
        # Envia mensagem inicial
        dispatcher.utter_message(
            text="Olá! Eu sou o Jarbinhas, sua nova companhia. Estou aqui para ajudar a tornar seus dias mais agradáveis, lembrá-lo de tomar a medicação e cuidar de você. Para nos conhecermos melhor, poderia me dizer seu nome?"
        )
        # Usa o template definido para perguntar o nome
        dispatcher.utter_message(template="utter_ask_name")
        
        # # Define eventos iniciais
        # events = [SessionStarted(), ActionExecuted("action_listen")]
        # return events
        return [SessionStarted()]


class ActionSaveName(Action):
    def name(self) -> str:
        return "action_save_name"

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: dict):
        # Extração segura do nome
        user_name = next(tracker.get_latest_entity_values("name"), None)
        if user_name:
            # Salva o nome no slot e em um arquivo
            with open("user_data.json", "w") as f:
                json.dump({"name": user_name}, f)

            # Envia mensagens de confirmação
            dispatcher.utter_message(template="utter_confirm_name")
        

            # Define o slot 'name' e retorna
            return [SlotSet("name", user_name)]
        else:
            # Caso não capture o nome, pede para repetir
            dispatcher.utter_message(
                text="Desculpe, não consegui captar o nome. Pode repetir?"
            )
            return []



class ActionSaveSaudacao(Action):
    def name(self) -> str:
        return "action_save_saudacao"

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: dict):
        # Extração segura da saudação
        user_saudacao = next(tracker.get_latest_entity_values("saudacao"), None)
        if user_saudacao:
            # Verifica se o arquivo já existe e carrega os dados existentes
            data = {}
            if os.path.exists("user_data.json"):
                with open("user_data.json", "r") as f:
                    try:
                        data = json.load(f)
                    except json.JSONDecodeError:
                        # Se o arquivo estiver corrompido ou vazio, inicia com um dicionário vazio
                        data = {}

            # Atualiza os dados com a saudação
            data["saudacao"] = user_saudacao

            # Salva os dados atualizados de volta no arquivo
            with open("user_data.json", "w") as f:
                json.dump(data, f)

            # Define o slot 'saudacao' e envia mensagem de confirmação
            dispatcher.utter_message(template="utter_confirm_saudacao")
            return [SlotSet("saudacao", user_saudacao)]
        else:
            # Caso não capture a saudação, pede para repetir
            dispatcher.utter_message(
                text="Desculpe, não consegui captar a saudação. Pode repetir?"
            )
            return []



class ActionSavePreferencias(Action):
    def name(self) -> str:
        return "action_save_preferencias"

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict[str, Any]) -> List[Dict[str, Any]]:
        # Extração segura dos temas
        user_temas = tracker.get_latest_entity_values("temas")
        temas_extraidos = list(user_temas)

        if temas_extraidos:
            # Verifica se o arquivo já existe e carrega os dados existentes
            data = {}
            if os.path.exists("user_data.json"):
                with open("user_data.json", "r") as f:
                    try:
                        data = json.load(f)
                    except json.JSONDecodeError:
                        # Se o arquivo estiver corrompido ou vazio, inicia com um dicionário vazio
                        data = {}

            # Atualiza os dados com os novos temas
            if "temas" not in data:
                data["temas"] = []

            # Adiciona os novos temas sem duplicatas
            data["temas"].extend([tema for tema in temas_extraidos if tema not in data["temas"]])

            # Salva os dados atualizados de volta no arquivo
            with open("user_data.json", "w") as f:
                json.dump(data, f)

            # Define o slot 'temas' e envia mensagem de confirmação
            dispatcher.utter_message(template="utter_confirm_preferencias")
            return [SlotSet("temas", data["temas"])]
        else:
            # Caso não capture os temas, pede para repetir
            dispatcher.utter_message(
                text="Desculpe, não consegui captar os temas. Pode repetir?"
            )
            return []




logger = logging.getLogger(__name__)

class ActionGoToKitchen(Action):

    def name(self) -> Text:
        return "action_ir_para_cozinha"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        
        url="http://127.0.0.1:8000/go_to_position/1"
        
        logger.debug("Post to: " + url )
        
        response = requests.post(url, json=1)
        
        logger.debug("Response: " + str(response))
        
        if(response.status_code == 200):
            dispatcher.utter_message(text="Vamos para a cozinha")
        else:
            dispatcher.utter_message(text="Não foi possível ir para a cozinha")
        
        return []

class ActionIntroCheckup(Action):
    def name(self) -> str:
        return "action_intro_checkup"

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: dict):
        # Envia mensagem inicial
        dispatcher.utter_message(template="utter_intro_checkup")
        
        dispatcher.utter_message(template="utter_ask_pain")
        # # Define eventos iniciais
        # events = [SessionStarted(), ActionExecuted("action_listen")]
        # return events
        return [SessionStarted()]

class ActionSavePain(Action):
    def name(self) -> str:
        return "action_save_pain"
    
    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: dict):
        # Extração segura da saudação
        complete_answer_pain = tracker.latest_message.get("text", "")
        if complete_answer_pain:
            # Verifica se o arquivo já existe e carrega os dados existentes
            data = {}
            if os.path.exists("user_data.json"):
                with open("user_data.json", "r") as f:
                    try:
                        data = json.load(f)
                    except json.JSONDecodeError:
                        # Se o arquivo estiver corrompido ou vazio, inicia com um dicionário vazio
                        data = {}

            # Atualiza os dados com a saudação
            data["resposta_sobre_dor"] = complete_answer_pain

            # Salva os dados atualizados de volta no arquivo
            with open("user_data.json", "w") as f:
                json.dump(data, f)

            # Define o slot 'saudacao' e envia mensagem de confirmação
            dispatcher.utter_message(template="utter_ask_alimentation")
            return []
        else:
            # Caso não capture a saudação, pede para repetir
            dispatcher.utter_message(
                text="Desculpe, não consegui captar a informação. Pode repetir?"
            )
            return []


class ActionSaveAlimentation(Action):
    def name(self) -> str:
        return "action_save_alimentation"
    
    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: dict):
        # Extração segura da saudação
        complete_answer_alimentacao = tracker.latest_message.get("text", "")
        if complete_answer_alimentacao:
            # Verifica se o arquivo já existe e carrega os dados existentes
            data = {}
            if os.path.exists("user_data.json"):
                with open("user_data.json", "r") as f:
                    try:
                        data = json.load(f)
                    except json.JSONDecodeError:
                        # Se o arquivo estiver corrompido ou vazio, inicia com um dicionário vazio
                        data = {}

            # Atualiza os dados com a saudação
            data["resposta_sobre_alimentacao"] = complete_answer_alimentacao

            # Salva os dados atualizados de volta no arquivo
            with open("user_data.json", "w") as f:
                json.dump(data, f)

            # Define o slot 'saudacao' e envia mensagem de confirmação
            dispatcher.utter_message(template="utter_ask_emotion")
            return []
        else:
            # Caso não capture a saudação, pede para repetir
            dispatcher.utter_message(
                text="Desculpe, não consegui captar a informação. Pode repetir?"
            )
            return []

class ActionSaveEmotion(Action):
    def name(self) -> str:
        return "action_save_emotion"
    
    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: dict):
        # Extração segura da saudação
        complete_answer_sentimentos= tracker.latest_message.get("text", "")
        if complete_answer_sentimentos:
            # Verifica se o arquivo já existe e carrega os dados existentes
            data = {}
            if os.path.exists("user_data.json"):
                with open("user_data.json", "r") as f:
                    try:
                        data = json.load(f)
                    except json.JSONDecodeError:
                        # Se o arquivo estiver corrompido ou vazio, inicia com um dicionário vazio
                        data = {}

            # Atualiza os dados com a saudação
            data["resposta_sobre_sentimentos"] = complete_answer_sentimentos

            # Salva os dados atualizados de volta no arquivo
            with open("user_data.json", "w") as f:
                json.dump(data, f)

            # Define o slot 'saudacao' e envia mensagem de confirmação
            dispatcher.utter_message(template="utter_end_checkup")
            return []
        else:
            # Caso não capture a saudação, pede para repetir
            dispatcher.utter_message(
                text="Desculpe, não consegui captar a informação. Pode repetir?"
            )
            return []

    
class ActionEmergency(Action):

    def name(self) -> Text:
        return "action_emergencia"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        
        sender_id = tracker.sender_id
        # print(sender_id)
        # url = "http://liga_led_emergencia"
        
        # logger.debug(f"Post to: {url} with sender_id: {sender_id}")
        
        # response = requests.post(url, json={"sender_id": sender_id})
        
        # logger.debug(f"Response: {response}")
        
        # if response.status_code == 200:
        #     dispatcher.utter_message(text="Ligando para o seu contato de emergência")
        # else:
        #     dispatcher.utter_message(text="Não foi possível ligar para o contato de emergência")
        
        logger.debug(f"Emergency to: {sender_id} ")
        dispatcher.utter_message(text=f"Ligando para o seu contato de emergência do {sender_id}")
        
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

Além disso, durante o processo de checkup é possível realizar o seguinte diálogo de exemplo:

- J.A.R.B.A.S

  Olá, Ana! Como você está? Estamos chegando ao final do dia, e é hora de fazermos nosso check-up!
  Lembre-se de que o objetivo desta atividade é ajudar a cuidar do seu bem-estar, então, por favor, seja sincero nas respostas.
  Durante o dia, você teve algum incômodo, dor ou desconforto? Se sim, quais?

- Usuário

  Hoje eu tive dor de cabeça pela manhã

- J.A.R.B.A.S

  E como estava seu apetite hoje? Sentiu pouca fome, muita, ou algum enjoo?

- Usuário

  Hoje eu estava com pouco apetite, mas comi de forma saudável

- J.A.R.B.A.S

  Por último, quero saber um pouco sobre como você se sentiu emocionalmente. Qual emoção descreveria melhor o seu dia?

- Usuário

  Tive um dia muito tranquilo e me senti em paz.

- J.A.R.B.A.S

  Muito obrigada, Ana! Fico feliz com suas respostas e por sua confiança.
  Lembre-se, estarei sempre por aqui. Se precisar conversar mais, basta me chamar.


Durante esse diálogo, a lógica aplicada permite que as informações de possíveis dores, alimentação e emoções sejam salvas em um arquivo json, para serem utilizados pelo sistema posteriormente.

---

## Conclusão

Essas implementações tornam o chatbot mais interativo e eficiente, proporcionando uma experiência personalizada ao usuário. Com as regras, intenções e ações descritas, o J.A.R.B.A.S está preparado para executar tarefas específicas e lidar com diferentes interações.
