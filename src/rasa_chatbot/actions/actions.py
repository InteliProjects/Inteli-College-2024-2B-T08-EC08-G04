from rasa_sdk import Action, Tracker
from rasa_sdk.events import EventType
from rasa_sdk.executor import CollectingDispatcher
from rasa_sdk.events import SlotSet
import json
import os
from rasa_sdk.events import UserUtteranceReverted
from rasa_sdk.events import SessionStarted, ActionExecuted
from typing import Any, Text, Dict, List
import logging
import requests


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

# class ActionSaveName(Action):
#     def name(self) -> str:
#         return "action_save_name"

#     def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: dict):
#         # Extração segura do nome
#         user_name = next(tracker.get_latest_entity_values("name"), None)
#         if user_name:
#             # Salva o nome no slot e em um arquivo
#             with open("user_data.json", "w") as f:
#                 json.dump({"name": user_name}, f)

#             # Define o slot 'name'
#             return [SlotSet("name", user_name), 
#                     # Confirma o nome utilizando o template
#                     dispatcher.utter_message(template="utter_confirm_name"),
#                     dispatcher.utter_message(template="utter_confirm_saudacao")]
            
#         else:
#             # Caso não capture o nome, pede para repetir
#             dispatcher.utter_message(
#                 text="Desculpe, não consegui captar o nome. Pode repetir?"
#             )
#             return []

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