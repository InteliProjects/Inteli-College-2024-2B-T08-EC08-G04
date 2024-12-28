import logging
from ibm_watson import TextToSpeechV1
from ibm_cloud_sdk_core.authenticators import IAMAuthenticator
from core.config import TTS_API_KEY, TTS_SERVICE_URL

def get_text_to_speech():
    tts_authenticator = IAMAuthenticator(TTS_API_KEY)
    text_to_speech = TextToSpeechV1(authenticator=tts_authenticator)
    text_to_speech.set_service_url(TTS_SERVICE_URL)
    return text_to_speech

text_to_speech = get_text_to_speech()

def synthesize_speech(text, voice='pt-BR_IsabelaV3Voice', accept='audio/mp3'):
    try:
        audio_content = text_to_speech.synthesize(
            text,
            voice=voice,
            accept=accept
        ).get_result().content
        return audio_content
    except Exception as e:
        logging.error(f"Error synthesizing speech: {e}")
        raise e
