import os
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

# Watson Assistant configurations
WATSON_API_KEY = os.getenv('WATSON_API_KEY')
WATSON_SERVICE_URL = os.getenv('WATSON_SERVICE_URL')
WATSON_ASSISTANT_ID = os.getenv('WATSON_ASSISTANT_ID')

# Text to Speech configurations
TTS_API_KEY = os.getenv('TTS_API_KEY')
TTS_SERVICE_URL = os.getenv('TTS_SERVICE_URL')

# Speech to Text configurations
STT_API_KEY = os.getenv('STT_API_KEY')
STT_SERVICE_URL = os.getenv('STT_SERVICE_URL')

# configurando a chave de API do GEMINI
API_GEMINI_KEY = os.getenv('API_GEMINI_KEY')
