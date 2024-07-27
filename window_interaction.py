# Tkinter application code
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
import cv2
import threading
import speech_recognition as sr
from gtts import gTTS
import subprocess
import asyncio
import time
from rasa.core.agent import Agent
import json

# Initialize the Rasa agent
agent = Agent.load('models/20240725-145948-genteel-tundra.tar.gz')

# Video paths
LISTENING_VIDEO = '/Users/hasu/Documents/Rasatraining 2/botexpression.mp4'
SPEAKING_VIDEO = '/Users/hasu/Documents/Rasatraining 2/botexpression.mp4'
NORMAL_VIDEO = '/Users/hasu/Documents/Rasatraining 2/botexpression.mp4'

# Global variables
bot_state = 'normal'
stop_event = threading.Event()

# Function to play the video in the video frame
def play_video(video_path, video_label):
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"Error: Unable to open video file {video_path}")
        return

    while cap.isOpened() and not stop_event.is_set():
        ret, frame = cap.read()
        if not ret:
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # Restart video
            continue
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame = cv2.resize(frame, (640, 480))
        img = Image.fromarray(frame)
        imgtk = ImageTk.PhotoImage(image=img)
        video_label.imgtk = imgtk
        video_label.configure(image=imgtk)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()

def video_playback_loop(video_label):
    global bot_state
    while not stop_event.is_set():
        if bot_state == 'listening':
            play_video(LISTENING_VIDEO, video_label)
        elif bot_state == 'speaking':
            play_video(SPEAKING_VIDEO, video_label)
        else:
            play_video(NORMAL_VIDEO, video_label)

# Function to convert text to speech and play it
def speak(text):
    global bot_state
    bot_state = 'speaking'
    tts = gTTS(text=text, lang='en')
    tts.save("response.mp3")
    subprocess.run(["afplay", "response.mp3"])
    bot_state = 'normal'

# Function to listen to user input via microphone
def listen():
    global bot_state
    recognizer = sr.Recognizer()
    with sr.Microphone() as source:
        print("Listening...")
        bot_state = 'listening'
        audio = recognizer.listen(source)
        bot_state = 'normal'
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

# Function to display flight data in the table
def display_flight_data(flight_data):
    print("Function display_flight_data called.")  # Debug
    print("Flight data:", flight_data)  # Debug

    # Clear existing data
    for row in flight_table.get_children():
        flight_table.delete(row)

    # Insert new data
    if "flights" in flight_data:
        for flight in flight_data["flights"]:
            print(f"Adding flight: {flight}")  # Debug
            flight_table.insert("", "end", values=(
                flight.get("flight", "N/A"),
                flight.get("airlines", "N/A"),
                flight.get("status", "N/A"),
                flight.get("scheduled_time_of_arrival", "N/A"),
                flight.get("estimated_time_of_arrival", "N/A")
            ))
    else:
        print("No 'flights' key found in the flight data.")  # Debug

# Asynchronous function to handle chat interactions
async def chat(chat_text):
    global stop_event
    speak("Hello, I am Airo. How can I assist you today?")
    
    while not stop_event.is_set():
        user_input = listen()
        if user_input:
            chat_text.configure(state='normal')
            chat_text.insert(tk.END, f"You: {user_input}\n")
            chat_text.configure(state='disabled')
            chat_text.see(tk.END)

            if user_input.lower() == "stop":
                stop_event.set()
                break

            time.sleep(2)  # Simulate thinking
            
            responses = await agent.handle_text(user_input)
            if responses:
                response_text = responses[0]['text']
                speak(response_text)
                chat_text.configure(state='normal')
                chat_text.insert(tk.END, f"Airo: {response_text}\n")
                chat_text.configure(state='disabled')
                chat_text.see(tk.END)

                # Check if the response contains QR code data
                if "QR Code Data" in response_text:
                    chat_text.configure(state='normal')
                    chat_text.insert(tk.END, f"Airo: {response_text}\n")
                    chat_text.configure(state='disabled')
                    chat_text.see(tk.END)
                
                # Attempt to load and display flight data
                try:
                    print("Attempting to open JSON file...")  # Debug
                    with open("/Users/hasu/Documents/Rasatraining 2/Arrival.json") as f:
                        flight_data = json.load(f)
                        print("Loaded flight data: ", flight_data)  # Debug
                        display_flight_data(flight_data)
                except FileNotFoundError:
                    print("Error: File not found.")
                    chat_text.insert(tk.END, "Airo: Error: Flight data file not found.\n")
                except json.JSONDecodeError as e:
                    print(f"Error decoding JSON: {e}")
                    chat_text.insert(tk.END, "Airo: Error decoding flight data.\n")
                except Exception as e:
                    print(f"Unexpected error: {e}")
                    chat_text.insert(tk.END, f"Airo: An unexpected error occurred: {e}\n")
            else:
                bot_state = 'normal'
        else:
            bot_state = 'normal'

def close_window():
    global stop_event
    stop_event.set()
    root.destroy()

if __name__ == "__main__":
    # Create the main window
    root = tk.Tk()
    root.title("Airo - Your Airport Assistant")

    # Create frames for video and chat
    video_frame = ttk.Frame(root, width=640, height=480)
    video_frame.grid(row=0, column=0, padx=10, pady=10)
    chat_frame = ttk.Frame(root, width=400, height=480)
    chat_frame.grid(row=0, column=1, padx=10, pady=10)
    table_frame = ttk.Frame(root, width=1040, height=200)
    table_frame.grid(row=1, column=0, columnspan=2, padx=10, pady=10)

    # Create a label in the video frame to display the video
    video_label = ttk.Label(video_frame)
    video_label.grid(row=0, column=0)

    # Create a text widget in the chat frame to display chat
    chat_text = tk.Text(chat_frame, state='disabled', width=50, height=30, wrap='word')
    chat_text.grid(row=0, column=0)

    # Create a treeview widget for displaying flight data
    flight_table = ttk.Treeview(table_frame, columns=("Flight", "Airline", "Status", "Scheduled Arrival", "Estimated Arrival"), show='headings')
    flight_table.heading("Flight", text="Flight")
    flight_table.heading("Airline", text="Airline")
    flight_table.heading("Status", text="Status")
    flight_table.heading("Scheduled Arrival", text="Scheduled Arrival")
    flight_table.heading("Estimated Arrival", text="Estimated Arrival")
    flight_table.pack(fill=tk.BOTH, expand=True)

    # Load the JSON data immediately
    try:
        with open("/Users/hasu/Documents/Rasatraining 2/Arrival.json") as f:
            flight_data = json.load(f)
            display_flight_data(flight_data)
    except FileNotFoundError:
        print("Error: File not found.")
    except json.JSONDecodeError as e:
        print(f"Error decoding JSON: {e}")
    except Exception as e:
        print(f"Unexpected error: {e}")

    # Start video playback loop in a separate thread
    video_thread = threading.Thread(target=video_playback_loop, args=(video_label,), daemon=True)
    video_thread.start()

    # Run the chatbot interaction in a separate thread
    chat_thread = threading.Thread(target=lambda: asyncio.run(chat(chat_text)), daemon=True)
    chat_thread.start()

    # Bind the window close event
    root.protocol("WM_DELETE_WINDOW", close_window)

    # Start the Tkinter main loop
    root.mainloop()
