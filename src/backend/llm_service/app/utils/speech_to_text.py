import logging
from ibm_watson import SpeechToTextV1
from ibm_cloud_sdk_core.authenticators import IAMAuthenticator
from core.config import STT_API_KEY, STT_SERVICE_URL
import os

logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

def get_speech_to_text():
    try:
        if not STT_API_KEY or not STT_SERVICE_URL:
            raise ValueError("API key or service URL not set correctly in the environment variables.")

        stt_authenticator = IAMAuthenticator(STT_API_KEY)
        speech_to_text = SpeechToTextV1(authenticator=stt_authenticator)
        speech_to_text.set_service_url(STT_SERVICE_URL)
        return speech_to_text

    except Exception as e:
        logging.error(f"Error in setting up Speech to Text service: {e}")
        raise e


speech_to_text = get_speech_to_text()

def transcribe_audio(audio_file_path):
    try:
        with open(audio_file_path, 'rb') as audio_file:

            response = speech_to_text.recognize(
                audio=audio_file,
                content_type='audio/wav',  
                model='pt-BR_Telephony'  
            ).get_result()

        results = response.get('results', [])
        if not results:
            logging.warning(f"No transcription results found for file: {audio_file_path}")
            return ""
        transcript = results[0]['alternatives'][0]['transcript']
        logging.info(f"Transcription completed for file: {audio_file_path}")
        return transcript

    except Exception as e:
        logging.error(f"Error transcribing audio file {audio_file_path}: {e}")
        raise e
