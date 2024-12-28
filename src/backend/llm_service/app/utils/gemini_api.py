import google.generativeai as genai
from core.config import API_GEMINI_KEY

genai.configure(api_key=API_GEMINI_KEY)

# Mapeia session_id para o objeto de chat do Gemini
sessions_map = {}

def start_chatbot(session_id: str):
    """Inicia um chatbot para a sessão"""
    model = genai.GenerativeModel('gemini-pro')
    chat = model.start_chat(history=[])
    sessions_map[session_id] = chat
    return chat

def send_message_to_gemini(session_id: str, message: str):
    """Envia uma mensagem para o chatbot da sessão"""
    chat = sessions_map.get(session_id)
    if not chat:
        raise ValueError(f"Chatbot não encontrado para session_id: {session_id}")
    response = chat.send_message(message)
    return response.text

def delete_chatbot(session_id: str):
    """Remove o chatbot associado à sessão"""
    if session_id in sessions_map:
        del sessions_map[session_id]
