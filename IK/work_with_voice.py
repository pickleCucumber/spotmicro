import os
import sys
import json
import pyaudio
import threading
import time
from vosk import Model, KaldiRecognizer
from oop_test_to_neutral0 import SpotMicroController
#from oop_test import SpotMicroController

# === Settings ===
MODEL_PATH = "/home/rpi/sr_model/vosk-model-small-ru-0.22/"  
SAMPLE_RATE = 16000
CHUNK_SIZE = 4096
FORMAT = pyaudio.paInt16
CHANNELS = 1

# mapping phrase to command Spot
VOICE_COMMANDS = {
    "вперёд": "forward",
    "перёд": "forward", 
    "перед": "forward", 
    "назад": "backward",
    "влево": "turn_left",
    "лево": "turn_left",
    "вправо": "turn_right",
    "права": "turn_right",
    "право": "turn_right",
    "стоп": "stop_walk",
    "стоп": "stop_walk",
    "сто": "stop_walk",
    "остановка":"stop_walk",
    "установка":"stop_walk",
    "асанов":"stop_walk",
    "привет":"hi",
    "дай лапу": "paw_right",
    "правую лапу": "paw_right",
    "левую лапу": "paw_left",
    "левая ладно": "paw_left",
    "опустил ладно": "paw_down",
    "опусти лапы": "paw_down",
    "опусти лапу": "paw_down",
    "опустил лапу": "paw_down",

    "опустил лапы": "paw_down",
    "опустела": "paw_down",
    "опустила": "paw_down",
    "блядь": "stop",
    "сосать": "stop",
    "пусти лапы": "paw_down",
    "остановить": "stop_walk",
    "собачье дело": "pee",
    "собачьи дела": "pee",
    "собачи дела": "pee",
    "встряхнуться": "twist",
    "встряхнись": "twist",
    "хороший мальчик": "twist",
    "хвост": "twist",
    "сесть": "sit",
    "сидеть": "sit",
    "лечь": "lie",
    "лежать": "lie",
    "восстать": "stand",
    "встать": "stand",
    "стать": "stand",
    "поворот влево": "turn_left",
    "поворот лево": "turn_left",
    "поворот вправо": "turn_right",
    "поворот право": "turn_right",
    "анимация": "anim",
    "движение": "move",
    "трот": "trot",
    "иму": "imu",
    "выход": "quit",
}

def voice_thread(controller):
    print("=== Voice thread STARTED ===")
    
    try:
        if not os.path.exists(MODEL_PATH):
            print(f"ERROR: Vosk model not found at '{MODEL_PATH}'!")
            return

        print("Loading Vosk model...")
        model = Model(MODEL_PATH)
        print("Vosk model loaded successfully")
        
        recognizer = KaldiRecognizer(model, SAMPLE_RATE)
        p = pyaudio.PyAudio()
        
        print("Opening audio stream...")
        stream = p.open(format=FORMAT, channels=CHANNELS, rate=SAMPLE_RATE, input=True, frames_per_buffer=CHUNK_SIZE)
        print("Audio stream opened")

        print("\n Voice control is active. Speak command")
        
        while getattr(controller, 'continuer', True):
            try:
                data = stream.read(CHUNK_SIZE, exception_on_overflow=False)
                if recognizer.AcceptWaveform(data):
                    result = json.loads(recognizer.Result())
                    text = result.get('text', '').strip().lower()
                    if text:
                        print(f"\n Recognizing: '{text}'")
                        command_found = False
                        for phrase, cmd in VOICE_COMMANDS.items():
                            if phrase in text:
                                controller.accept_command(cmd)
                                print(f" Voice commad: {cmd}")
                                command_found = True
                                break
                        if not command_found:
                            print(f" Unknows command: '{text}'")
                else:
                    partial = json.loads(recognizer.PartialResult()).get('partial', '')
                    if partial:
                        sys.stdout.write('\r Listing' + partial + ' ' * 10)
                        sys.stdout.flush()
                        
            except Exception as e:
                print(f"Error in recognize cycle: {e}")
                time.sleep(0.1)
                
    except Exception as e:
        print(f"Creatical error in voice thread: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        print("Voice thread close")
        try:
            if 'stream' in locals():
                stream.stop_stream()
                stream.close()
            if 'p' in locals():
                p.terminate()
        except:
            pass

if __name__ == "__main__":
    print("=== Starting SpotMicro with Voice Control ===")
    print("=== Default settings: Animation OFF, Servo movement ON ===")
    print(f"Python version: {sys.version}")
    print(f"Working directory: {os.getcwd()}")
    
    # check I2C
    try:
        import board
        print(" I2C доступен")
    except Exception as e:
        print(f" I2C недоступен: {e}")
    
    # create controller
    controller = SpotMicroController()
    print("=== Controller created ===")
    
    # start voice control in a separate thread
    voice_th = threading.Thread(target=voice_thread, args=(controller,), daemon=True)
    voice_th.start()
    
    # time for inizialize voice thread
    time.sleep(3)
    
    # check living thread
    if voice_th.is_alive():
        print(" Voice thread is active")
        print(" Speak command")
    else:
        print(" Voice thread is not active")
        print("Check micro or model Vosk")
    
    # start main controller in main thread
    print("=== Starting main controller ===")
    try:
        controller.start()
    except KeyboardInterrupt:
        print("\n=== Program interrupted by user ===")
    except Exception as e:
        print(f"=== Controller error: {e} ===")
        import traceback
        traceback.print_exc()
    
    print("=== Program finished ===")
