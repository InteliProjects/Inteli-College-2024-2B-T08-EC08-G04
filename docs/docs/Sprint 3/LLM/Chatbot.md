---
title: Chatbot LLM
sidebar_label: Chatbot
sidebar_position: 1
---

# Chatbot

Aqui será descrita toda a estruturação do chatbot com LLM, desde a escolha do modelo até a integração com o sistema. Para isso, escolhemos algumas tecnologias: o Rasa para o chatbot e as seguintes tecnologias de suporte: IBM Text to Speech, IBM Speech to Text e Gemini.

## Rasa chatbot

O Rasa é uma plataforma de código aberto para automação de conversas. Ele é uma ferramenta open source que permite criar chatbots e assistentes virtuais. O Rasa utiliza uma abordagem de aprendizado de máquina NLU, que é entedimento de liguagem natural,para entender e responder a perguntas de forma inteligente. O Rasa é uma ferramenta muito poderosa e flexível, que permite criar chatbots altamente personalizados e escaláveis. Ele oferece uma estrutura com regras, intenções e ações.

- **Regras**: As regras são usadas para definir o comportamento do chatbot em resposta a determinadas entradas do usuário. Por exemplo, temos uma regra que sempre que o usuário fala algo como "tchau", o chatbot entra na ação de despedida.
- **Intenções**: As intenções são os objetivos, que estão subentendidos na frase, do usuário ao interagir com o chatbot. Por exemplo, na frase: “consultar a fatura do cartão”, a intenção é “consultar fatura”.
- **Ações**: As ações são as respostas que o chatbot pode dar ao usuário. Por exemplo, se o usuário pedir para consultar a fatura do cartão, o chatbot pode responder com a fatura do cartão.

Abaixo será detlhada cada uma das partes acima citadas, em relacao ao chatbot do LLM.

### Regras

Em `data/rules.yml` definimos todas as regras que o chatbot deve seguir. No nosso caso, nos fizemos uma regra de despedida, uma quando o usuário questionar se o chatbot é um robô, uma para quando o usuário quiser enviar o robô para um ponto (na cozinhar, por exemplo) e uma de fallback, que é quando o chatbot não entende o que o usuário está falando.

```yaml

version: "3.1"

rules:

- rule: Diga tchau sempre que o usuário disser tchau
  steps:
  - intent: despedida
  - action: utter_despedida

- rule: Diga 'Eu sou um robô' sempre que o usuário desafiar
  steps:
  - intent: desafio_ao_bot
  - action: utter_souumrobo

- rule: Ir para a cozinha
  steps:
  - intent: ir_para_cozinha
  - action: action_ir_para_cozinha

- rule: Fallback handling
  steps:
    - intent: nlu_fallback
    - action: utter_please_rephrase

```

### Intenções

No arquivo `data/nlu.yml` definimos todas as entidades que o chatbot deve reconhecer. No nosso caso, definimos uma entidade de saudacao, despedida, afirmar, negar, humor_bom, humor_triste, desafio_ao_bot e ir_para_cozinha. Na construção das intenções, utilizamos o conceito de exemplos, que são frases que o usuário pode falar para o chatbot, para quando o modelo for treinado, ele consiga identificar a intenção do usuário.

```yaml

version: "3.1"

nlu:
- intent: saudacao
  examples: |
    - oi
    - olá
    - e aí
    - bom dia
    - boa tarde
    - boa noite
    - como vai
    - oi, tudo bem?
    - vamos lá
    - e aí, cara
    - bomdia
    - boanoite
    - boa tarde

- intent: despedida
  examples: |
    - tchau
    - até logo
    - até mais
    - boa noite
    - até a próxima
    - adeus
    - tenha um bom dia
    - a gente se vê
    - tchau tchau
    - vejo você depois

- intent: afirmar
  examples: |
    - sim
    - s
    - com certeza
    - claro
    - parece bom
    - correto

- intent: negar
  examples: |
    - não
    - n
    - nunca
    - acho que não
    - não gosto disso
    - de jeito nenhum
    - não realmente

- intent: humor_bom
  examples: |
    - perfeito
    - ótimo
    - incrível
    - me sentindo um rei
    - maravilhoso
    - estou me sentindo muito bem
    - estou ótimo
    - estou incrível
    - vou salvar o mundo
    - super animado
    - extremamente bem
    - tão perfeito
    - muito bom
    - perfeito

- intent: humor_triste
  examples: |
    - meu dia foi horrível
    - estou triste
    - não me sinto muito bem
    - estou desapontado
    - super triste
    - estou tão triste
    - triste
    - muito triste
    - infeliz
    - não está bom
    - não muito bem
    - extremamente triste
    - tão triiiste
    - tão triste

- intent: desafio_ao_bot
  examples: |
    - você é um robô?
    - você é humano?
    - estou falando com um robô?
    - estou falando com um humano?

- intent: ir_para_cozinha
  examples: |
    - me leve para a cozinha
    - quero ir para o cozinha
    - vá até a cozinha
    - vá a cozinha
    - como eu chego na cozinha?
    - siga até a cozinha

```

### Ações

No arquivo `actions/actions.py` definimos todas as ações que o chatbot deve executar. No nosso caso, definimos uma ação ir_para_cozinha. As ações são as respostas que o chatbot pode dar ao usuário.

```python

# This files contains your custom actions which can be used to run
# custom Python code.
#
# See this guide on how to implement these action:
# https://rasa.com/docs/rasa/custom-actions


# This is a simple example for a custom action which utters "Hello World!"

from typing import Any, Text, Dict, List

from rasa_sdk import Action, Tracker
from rasa_sdk.executor import CollectingDispatcher
import logging

import requests

logger = logging.getLogger(__name__)

class ActionGoToKitchen(Action):

    def name(self) -> Text:
        return "action_ir_para_cozinha"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        
        url="http://localhost:5005/ir_para_robo"
        
        logger.debug("Post to: " + url )
        
        response = requests.post(url, json=1)
        
        logger.debug("Response: " + str(response))
        
        if(response.status_code == 200):
            dispatcher.utter_message(text="Vamos para a cozinha")
        else:
            dispatcher.utter_message(text="Não foi possível ir para a cozinha")
        
        return []

```

### Config Rasa Chatbot

No arquivo `config.yml` definimos todas as configurações do chatbot. No nosso caso, definimos o pipeline de processamento de linguagem natural, que é o modelo de linguagem que o chatbot deve utilizar para entender e responder a perguntas de forma inteligente.

```yaml

# The config recipe.
# https://rasa.com/docs/rasa/model-configuration/
recipe: default.v1

# The assistant project unique identifier
# This default value must be replaced with a unique assistant name within your deployment
assistant_id: 20241119-163055-diachronic-buffer

# Configuration for Rasa NLU.
# https://rasa.com/docs/rasa/nlu/components/
language: pt

pipeline:
# # No configuration for the NLU pipeline was provided. The following default pipeline was used to train your model.
# # If you'd like to customize it, uncomment and adjust the pipeline.
# # See https://rasa.com/docs/rasa/tuning-your-model for more information.
  - name: WhitespaceTokenizer
  - name: RegexFeaturizer
  - name: LexicalSyntacticFeaturizer
  - name: CountVectorsFeaturizer
  - name: CountVectorsFeaturizer
    analyzer: char_wb
    min_ngram: 1
    max_ngram: 4
  - name: DIETClassifier
    epochs: 100
    constrain_similarities: true
  - name: EntitySynonymMapper
  - name: ResponseSelector
    epochs: 100
    constrain_similarities: true
  - name: FallbackClassifier
    threshold: 0.7
    ambiguity_threshold: 0.1

# Configuration for Rasa Core.
# https://rasa.com/docs/rasa/core/policies/
policies: null
# # No configuration for policies was provided. The following default policies were used to train your model.
# # If you'd like to customize them, uncomment and adjust the policies.
# # See https://rasa.com/docs/rasa/policies for more information.
#   - name: MemoizationPolicy
#   - name: RulePolicy
#   - name: UnexpecTEDIntentPolicy
#     max_history: 5
#     epochs: 100
#   - name: TEDPolicy
#     max_history: 5
#     epochs: 100
#     constrain_similarities: true

```

### Como rodar

Primeiro é preciso criar um ambiente virtual e instalar o rasa, então vá até `src/rasa_chat`, rode:

```bash

python3 -m venv venv

source venv/bin/activate

pip install rasa

```

Após isso é preciso treinar o chatbot, então rode:

```bash

rasa train

```

Tendo feito isso, vamos buildar o dockerfile do chatbot, então rode:

```bash

docker build -t rasa_chat .

```

Por fim, rodamos o script `entrypoint.sh` que irá rodar o chatbot:

```bash

./entrypoint.sh

```

### Como utilizar

Quando rodamos o chatbot, ele irá rodar na porta 5005, então para interagir com ele, podemos fazer requisições HTTP para ele. Abaixo temos um exemplo de como fazer isso atráves da rota `http://127.0.0.1:5005/webhooks/rest/webhook`, com um body com essa estrutura:

```json

{
    "sender": "",
    "message": ""
}

```

# Integração

O rasa foi integrado no serviço `llm_service` orquestra o chatbot Rasa, Gemini, IBM Text to Speech e IBM Speech to Text. Atualmente, o fluxo de interação com nosso chatbot é o seguinte: 

1. O usuário fala com o chatbot no frontend, que envia um áudio para o backend.
2. O backend envia o áudio para o serviço IBM Speech to Text, que converte o áudio em texto.
3. O backend envia o texto para o chatbot Rasa, que processa o texto e retorna uma resposta.
4. Se o chatbot conseguir responder a pergunta do usuário, o backend envia a resposta para o serviço IBM Text to Speech, que converte o texto em áudio.
5. Se não, o backend envia a pergunta para o serviço Gemini, que retorna uma resposta.
6. O backend envia a resposta para o frontend, que reproduz o áudio para o usuário.

A estrategia nossa, é utilizar o chatbot para responder perguntas mais diretas que necessitam de uma resposta mais precisa ou uma ação espefica, como por exemplo fazer onboarding do paciente ou fazer o robô ir para um ponto específico. Já o Gemini, é utilizado para responder perguntas mais abertas, que não necessitam de uma resposta precisa, como por exemplo perguntar sobre o clima ou sobre o dia do usuário.

# Conclusão

Como melhoria para as próximas sprints, podemos adicionar mais regras, intenções e ações ao chatbot, para que ele possa responder a uma maior variedade de perguntas. Além disso, vamos fazer com que o chatbot também possa interagir com o usuário, como por exemplo lembrando de remedios, consultas e outras atividades do dia a dia do usuário.