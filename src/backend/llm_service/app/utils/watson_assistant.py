import logging
from ibm_watson import AssistantV2
from ibm_cloud_sdk_core.authenticators import IAMAuthenticator
from core.config import WATSON_API_KEY, WATSON_SERVICE_URL, WATSON_ASSISTANT_ID

def get_assistant():
    authenticator = IAMAuthenticator(WATSON_API_KEY)
    assistant = AssistantV2(
        version='2021-11-27',
        authenticator=authenticator
    )
    assistant.set_service_url(WATSON_SERVICE_URL)
    return assistant

assistant = get_assistant()
ASSISTANT_ID = WATSON_ASSISTANT_ID

def create_session():
    try:
        session = assistant.create_session(assistant_id=ASSISTANT_ID).get_result()
        session_id = session.get('session_id')
        logging.info(f"Session created successfully: {session_id}")
        return session_id
    except Exception as e:
        logging.error(f"Error creating session: {e}")
        raise e

def delete_session(session_id):
    try:
        assistant.delete_session(assistant_id=ASSISTANT_ID, session_id=session_id)
    except Exception as e:
        logging.error(f"Error deleting session: {e}")
        raise e

def send_message(session_id, input_data):
    try:
        response = assistant.message(
            assistant_id=ASSISTANT_ID,
            session_id=session_id,
            input=input_data
        ).get_result()
        return response
    except Exception as e:
        logging.error(f"Error sending message: {e}")
        raise e
