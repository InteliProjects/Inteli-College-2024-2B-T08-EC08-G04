import os
import requests
import logging
import base64
from models.schemas import MessageInput, MessageOutput, ResponseItem
from fastapi import APIRouter, HTTPException, UploadFile, File
from models.schemas import MessageInput, MessageOutput, ResponseItem
from utils.watson_assistant import create_session, delete_session, send_message
from utils.text_to_speech import synthesize_speech
from utils.audio_player import play_audio_in_thread
from utils.speech_to_text import transcribe_audio
from utils.gemini_api import start_chatbot, send_message_to_gemini, delete_chatbot
from utils.rasa_integration import send_message_rasa
import shutil
import wave
import magic
import tempfile
from markdown import markdown
from bs4 import BeautifulSoup

router = APIRouter()

@router.post("/session")
def create_session_endpoint():
    try:
        session_id = create_session()
        start_chatbot(session_id)  # Inicializa o chatbot do Gemini para essa sessão
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
        response = send_message_rasa(session_id, input_data)
        responses = []
        if response.get('output') and response['output'].get('generic'):
            for idx, resp in enumerate(response['output']['generic']):
                if resp.get('response_type') == 'text':
                    text = resp.get('text')
                    gemini_response = send_message_to_gemini(session_id, message_input.text)
                    responses.append(ResponseItem(text=f"Watson: {text}\nGemini: {gemini_response}", audio=""))
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
        delete_chatbot(session_id)
        return {"message": "Session deleted successfully"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

def get_mime_type(file_path):
    mime = magic.Magic(mime=True)
    return mime.from_file(file_path)

onboarding_fineshed = False

@router.post("/speech-to-text")
def speech_to_text_endpoint(file: UploadFile = File(...), session_id_gemini: str = None, chatbotStarted: bool = False):
    try:
        logging.info(f"Session ID Gemini: {session_id_gemini}")
        logging.info(f"Chatbot Started: {chatbotStarted}")
        # Check if a file was uploaded
        if not file:
            raise HTTPException(status_code=400, detail="No audio file detected.")

        logging.info(f"Recebendo arquivo: {file.filename} | Tipo: {file.content_type}")
        file.file.seek(0)

        # Cria um arquivo temporário para armazenar o áudio recebido

        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as temp_file:
            shutil.copyfileobj(file.file, temp_file)
            temp_file_path = temp_file.name

        # Verifica se o arquivo já é WAV
        if not temp_file_path.endswith(".wav"):
            raise HTTPException(status_code=400, detail="Please upload files in WAV format.")

        # Transcrever o áudio
        transcript = transcribe_audio(temp_file_path)
        logging.info(f"Transcrição do áudio: {transcript}")

        # Enviar transcrição para o Rasa

        rasa_response = send_message_rasa("user123", transcript)
        
        global onboarding_fineshed
        
        if "suas preferências foram salvas com sucesso. Agora estamos prontos, e a etapa de onboarding acabou. Daqui em diante, estarei sempre aqui para ajudar e fazer companhia. Se precisar de algo, basta me chamar." in rasa_response[0].get('text'):
            print("ONBOARDING FINISHED")
            onboarding_fineshed = True
    
        if rasa_response is None or (len(rasa_response) > 0 and (rasa_response[0].get('text') == "Desculpa, não entendi. Pode tentar novamente?" or rasa_response[0].get('text') == "Desculpe, não consegui captar a saudação. Pode repetir?" or rasa_response[0].get('text') == "Desculpe, não consegui captar os temas. Pode repetir?")) or onboarding_fineshed:
            logging.info("Erro ou resposta inválida do Rasa. Redirecionando para o Gemini.")

            # Initialize Gemini session and send message
            # session_id = create_session()
            if(chatbotStarted == False):
                print("Starting Gemini chatbot")
                start_chatbot(session_id_gemini)
            
            print("Gemini Já começou")
                
            gemini_response = send_message_to_gemini(session_id_gemini, transcript)

            html = markdown(gemini_response)
            
            soup = BeautifulSoup(html, "html.parser")
            gemini_response_clean = soup.get_text()
            
            print(gemini_response_clean)

            # Síntese de fala usando a resposta do Gemini

            audio_content = synthesize_speech(gemini_response_clean)

            return {
                "transcript": transcript,
                "response": gemini_response_clean,
                "audio_file": base64.b64encode(audio_content).decode('utf-8'),
            }

        else:
            rasa_text = rasa_response[0]['text']
            audio_content = synthesize_speech(rasa_text)

            return {
                "transcript": transcript,
                "response": rasa_text,
                "audio_file": base64.b64encode(audio_content).decode('utf-8'),
            }


    except HTTPException as http_exc:
        # Return HTTP exceptions as-is
        logging.error(f"HTTP error in speech-to-text endpoint: {http_exc.detail}")
        raise http_exc

    except Exception as e:
        # Handle unexpected exceptions
        logging.error(f"Erro no endpoint speech-to-text: {e}")
        raise HTTPException(status_code=500, detail="An unexpected error occurred. Please try again later.")

    finally:
        # Remove the temporary file after processing
        if 'temp_file_path' in locals() and os.path.exists(temp_file_path):
            os.remove(temp_file_path)

BACKEND_URL = os.getenv("BACKEND_URL", "http://localhost:3000")

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
        os.remove(temp_file_path)
