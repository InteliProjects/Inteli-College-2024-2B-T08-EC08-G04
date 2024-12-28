import requests
import logging

# http://127.0.0.1:5005/webhooks/rest/webhook
# {
#   "sender": "user123",
#   "message": "Me leve para a cozinha"
# }


def send_message_rasa(sender, input_data):
    RASA_URL = "http://127.0.0.1:5005/webhooks/rest/webhook"
    try:
        logging.info(f"Enviando mensagem para Rasa: {input_data}")
        response = requests.post(RASA_URL, json={"sender": sender, "message": input_data})
        response.raise_for_status()
        return response.json()  # Resposta bem-sucedida do Rasa
    except requests.ConnectionError as ce:
        logging.error(f"Erro de conexão com o Rasa: {ce}")
        return None  # Indica falha na conexão
    except Exception as e:
        logging.error(f"Erro ao enviar mensagem para o Rasa: {e}")
        return None  # Indica falha genérica
