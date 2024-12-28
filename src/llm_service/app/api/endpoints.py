
import os
import requests
from fastapi import APIRouter, HTTPException
from ..models.schemas import MessageInput, MessageOutput, ResponseItem
from utils.watson_assistant import create_session, delete_session, send_message
from utils.text_to_speech import synthesize_speech
from utils.audio_player import play_audio_in_thread



router = APIRouter()


BACKEND_URL = os.getenv("BACKEND_URL", "http://localhost:3000")

@router.post("/session")
def create_session_endpoint():
    try:
        session_id = create_session()
        return {"session_id": session_id}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.post("/message/{session_id}", response_model=MessageOutput)
def send_message_endpoint(session_id: str, message_input: MessageInput):
    input_data = {
        'message_type': 'text',
        'text': message_input.text
    }
    try:
        response = send_message(session_id, input_data)
        responses = []
        if response.get('output') and response['output'].get('generic'):
            for idx, resp in enumerate(response['output']['generic']):
                if resp.get('response_type') == 'text':
                    text = resp.get('text')
                    try:
                        audio_content = synthesize_speech(text)
                        audio_base64 = base64.b64encode(audio_content).decode('utf-8')
                        audio_file_path = f"response_audio_{idx}.mp3"
                        with open(audio_file_path, "wb") as audio_file:
                            audio_file.write(audio_content)
                        play_audio_in_thread(audio_file_path)
                    except Exception as e:
                        logging.error(f"Error synthesizing speech: {e}")
                        audio_base64 = ''
                    responses.append(ResponseItem(text=text, audio=audio_base64))
        return MessageOutput(responses=responses)
    except Exception as e:
        logging.error(f"Error sending message: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@router.delete("/session/{session_id}")
def delete_session_endpoint(session_id: str):
    try:
        delete_session(session_id)
        return {"message": "Session deleted successfully"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/usuario/{usuario_id}")
def get_usuario_info(usuario_id: int):
    try:
        response = requests.get(f"{BACKEND_URL}/usuarios/{usuario_id}")

        if response.status_code == 200:
            usuario_data = response.json()
            texto_resposta = f"O usuário {usuario_data['name']} toma os seguintes remédios: {usuario_data['medications']}. Sua rotina é: {usuario_data['routine']}."
            return {"text": texto_resposta}
        else:
            raise HTTPException(status_code=response.status_code, detail="Usuário não encontrado no backend.")
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.get("/saude-mental/{usuario_id}")
def get_saude_mental_info(usuario_id: int):
    try:
        response = requests.get(f"{BACKEND_URL}/saude-mental/{usuario_id}")
        if response.status_code == 200:
            saude_mental_data = response.json()
            texto_resposta = f"Dados de saúde mental: {saude_mental_data['descricao']}."
            return {"text": texto_resposta}
        else:
            raise HTTPException(status_code=response.status_code, detail="Dados de saúde mental não encontrados no backend.")
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
    
@router.get("/usuario-completo/{usuario_id}")
def get_usuario_completo(usuario_id: int):
    try:
        user_response = requests.get(f"{BACKEND_URL}/usuarios/{usuario_id}")
        mental_response = requests.get(f"{BACKEND_URL}/saude-mental/{usuario_id}")
        
        if user_response.status_code == 200 and mental_response.status_code == 200:
            user_data = user_response.json()
            mental_data = mental_response.json()
            
            texto_resposta = (
                f"O usuário {user_data['name']} toma os seguintes remédios: {user_data['medications']}."
                f"Sua rotina é: {user_data['routine']}. "
                f"Dados de saúde mental: {mental_data['descricao']}."
            )
            return {"text": texto_resposta}
        else:
            raise HTTPException(
                status_code=max(user_response.status_code, mental_response.status_code),
                detail="Erro ao consultar os dados do backend."
            )
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

