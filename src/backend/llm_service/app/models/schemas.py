from pydantic import BaseModel
from typing import List

class MessageInput(BaseModel):
    text: str

class ResponseItem(BaseModel):
    text: str
    audio: str  # Base64-encoded audio data

class MessageOutput(BaseModel):
    responses: List[ResponseItem]
