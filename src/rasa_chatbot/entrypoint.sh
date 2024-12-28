#!/bin/bash

# Ativar o ambiente virtual
source /home/rodrigo-sales/Repos/2024-2B-T08-EC08-G04/src/rasa_chatbot/venv/bin/activate

# Iniciar o servidor Rasa principal
rasa run --enable-api --cors "*" --debug &

# Iniciar o servidor de ações
rasa run actions --debug