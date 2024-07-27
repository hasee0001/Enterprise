import tkinter as tk
from tkinter import ttk
import json

# Function to display flight data in the table
def display_flight_data(flight_data):
    # Debug: Print flight data to ensure it's being loaded
    print("Flight data:", flight_data)

    # Clear existing data
    for row in flight_table.get_children():
        flight_table.delete(row)

    # Insert new data
    for flight in flight_data.get("flights", []):
        print(f"Adding flight: {flight}")  # Debug: Print each flight being added
        flight_table.insert("", "end", values=(flight["flight"], flight["airlines"], flight["status"], flight["scheduled_time_of_arrival"], flight["estimated_time_of_arrival"]))

# Create the main window
root = tk.Tk()
root.title("Flight Information")

# Create a frame for the table
table_frame = ttk.Frame(root, width=1040, height=200)
table_frame.grid(row=0, column=0, padx=10, pady=10)

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
except Exception as e:
    print(f"Error loading flight data: {e}")

# Start the Tkinter main loop
root.mainloop()
