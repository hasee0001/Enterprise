import pyttsx3

def speak_text(text):
    engine = pyttsx3.init()
    voices = engine.getProperty('voices')

    for voice in voices:
        if "zira" in voice.name.lower():
            engine.setProperty('voice', voice.id)
            break

    engine.setProperty('rate', 150)    # Speed percent (can go over 100)
    engine.setProperty('volume', 1)    # Volume 0-1

    engine.say(text)
    engine.runAndWait()