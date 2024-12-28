import os
import logging
import pygame
from threading import Thread

# Initialize pygame mixer once at the start
pygame.mixer.init()

def play_audio(audio_file_path):
    try:
        pygame.mixer.music.load(audio_file_path)
        pygame.mixer.music.play()
        while pygame.mixer.music.get_busy():
            pygame.time.Clock().tick(10)
    except Exception as e:
        logging.error(f"Error playing audio: {e}")
    finally:
        # Clean up the audio file after playback
        if os.path.exists(audio_file_path):
            os.remove(audio_file_path)

def play_audio_in_thread(audio_file_path):
    thread = Thread(target=play_audio, args=(audio_file_path,))
    thread.start()
