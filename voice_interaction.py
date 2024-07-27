import speech_recognition as sr
from gtts import gTTS
import subprocess
import asyncio
from rasa.core.agent import Agent

# Initialize the Rasa agent
agent = Agent.load('models/20240714-020053-ivory-road.tar.gz')

# Function to convert text to speech and play it
def speak(text):
    tts = gTTS(text=text, lang='en')
    tts.save("response.mp3")
    subprocess.run(["afplay", "response.mp3"]) 

# Function to listen to user input via microphone
def listen():
    recognizer = sr.Recognizer()
    with sr.Microphone() as source:
        print("Listening...")
        audio = recognizer.listen(source)
        try:
            text = recognizer.recognize_google(audio)
            print(f"You said: {text}")
            return text
        except sr.UnknownValueError:
            print("Sorry, I did not understand that.")
            return None
        except sr.RequestError:
            print("Sorry, there was an issue with the request.")
            return None

# Asynchronous function to handle chat interactions
async def chat():
    # Initial greeting
    speak("Hello, I am Airo. How can I assist you today?")
    
    while True:
        user_input = listen()
        if user_input:
            responses = await agent.handle_text(user_input)
            if responses:
                response_text = responses[0]['text']
                speak(response_text)

if __name__ == "__main__":
    asyncio.run(chat())
