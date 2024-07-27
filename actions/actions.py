import json
from typing import Any, Text, Dict, List
from rasa_sdk import Action, Tracker
from rasa_sdk.executor import CollectingDispatcher
import cv2
from pyzbar.pyzbar import decode
import subprocess

import json
from typing import Any, Text, Dict, List
from rasa_sdk import Action, Tracker
from rasa_sdk.executor import CollectingDispatcher

class ActionCheckFlightStatus(Action):

    def name(self) -> Text:
        return "action_check_flight_status"

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        flight_number = tracker.get_slot("flight_number")

        if not flight_number or flight_number.lower() == "flight status":
            dispatcher.utter_message(text="Please provide a flight number or visit our website for flight details: [Softair Airlines](http://localhost:3000/)")
            return []

        try:
            with open("/Users/hasu/Documents/Rasatraining 2/Arrival.json") as f:  # Update this path to your actual file location
                flight_data = json.load(f)
        except FileNotFoundError:
            dispatcher.utter_message(text="Flight data file not found.")
            return []
        except json.JSONDecodeError:
            dispatcher.utter_message(text="Error decoding flight data.")
            return []

        for flight in flight_data.get("flights", []):
            if flight["flight"].lower() == flight_number.lower():
                status = flight["status"]
                scheduled_time = flight["scheduled_time_of_arrival"]
                estimated_time = flight["estimated_time_of_arrival"]
                airline = flight["airlines"]
                response = (f"Flight {flight_number} ({airline}) is currently {status}. "
                            f"Scheduled arrival: {scheduled_time}, "
                            f"Estimated arrival: {estimated_time}.")
                dispatcher.utter_message(text=response)
                return []

        dispatcher.utter_message(text=f"No data found for flight {flight_number}.")
        return []

class ActionCheckAirlineStatus(Action):
    def name(self) -> Text:
        return "action_check_airline_status"

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        airline = tracker.get_slot('airline')

        if not airline:
            dispatcher.utter_message(text="Please provide an airline name.")
            return []

        try:
            with open("/Users/hasu/Documents/Rasatraining 2/Arrival.json") as f:
                flight_data = json.load(f)
                print("Flight data loaded successfully")  # Debug: Confirm data loading
        except FileNotFoundError:
            dispatcher.utter_message(text="Flight data file not found.")
            return []
        except json.JSONDecodeError:
            dispatcher.utter_message(text="Error decoding flight data.")
            return []

        # Debug: Check the structure and contents of the loaded data
        print(f"Flight data content: {flight_data}")

        flights = [flight for flight in flight_data.get("flights", []) if flight['airlines'].lower() == airline.lower()]
        if flights:
            response = f"Here are the flights for {airline}:\n"
            for flight in flights:
                response += (f"Flight {flight['flight']} from {flight['origin']} scheduled to arrive at {flight['scheduled_time_of_arrival']} "
                             f"is currently {flight['status']}.\n")
            dispatcher.utter_message(text=response)
        else:
            dispatcher.utter_message(text=f"No flights found for airline {airline}.")
        return []

class ActionProvideGateInfo(Action):
    def name(self) -> Text:
        return "action_provide_gate_info"

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        flight_number = tracker.get_slot('flight_number')

        if not flight_number:
            dispatcher.utter_message(text="Please provide a flight number.")
            return []

        gate_info = "Gate A1"  # Placeholder logic
        dispatcher.utter_message(text=f"Flight {flight_number} departs from {gate_info}.")
        return []

class ActionProvideSecurityInfo(Action):
    def name(self) -> Text:
        return "action_provide_security_info"

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        security_info = "Security checks are at Terminal 1. No liquids over 100ml allowed."
        dispatcher.utter_message(text=security_info)
        return []

class ActionProvideLoungeInfo(Action):
    def name(self) -> Text:
        return "action_provide_lounge_info"

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        lounge_info = "The lounge is located on the second floor of Terminal 1."
        dispatcher.utter_message(text=lounge_info)
        return []

class ActionProvideDirections(Action):
    def name(self) -> Text:
        return "action_provide_directions"

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        directions = "Follow the signs to Terminal 2. It is a 5-minute walk from here. Lets me take you there "
        dispatcher.utter_message(text=directions)
        return []

class ActionScanQRCode(Action):
    def name(self) -> str:
        return "action_scan_qr_code"

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict[str, Any]) -> List[Dict[str, Any]]:
        try:
            dispatcher.utter_message(text="Please show your boarding QR code near the camera properly.")
            
            # Capture video from the camera
            cap = cv2.VideoCapture(0)
            if not cap.isOpened():
                dispatcher.utter_message(text="Camera could not be opened.")
                return []

            qr_code_data = None

            while cap.isOpened():
                ret, frame = cap.read()
                if not ret:
                    dispatcher.utter_message(text="Failed to capture video frame.")
                    break

                # Decode QR code from the frame
                decoded_objects = decode(frame)
                for obj in decoded_objects:
                    qr_code_data = obj.data.decode('utf-8')
                    dispatcher.utter_message(text=f"Detected QR Code Data: {qr_code_data}")
                    break

                if qr_code_data:
                    break

            cap.release()
            cv2.destroyAllWindows()

            if not qr_code_data:
                dispatcher.utter_message(text="No QR code detected.")

        except Exception as e:
            dispatcher.utter_message(text=f"An error occurred: {e}")
            print(f"Error in ActionScanQRCode: {e}")

        return []
    
    def name(self) -> str:
        return "action_scan_qr_code"

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict[str, Any]) -> List[Dict[str, Any]]:
        try:
            dispatcher.utter_message(text="Please show your boarding QR code near the camera properly.")
            
            # Capture video from the camera
            cap = cv2.VideoCapture(0)
            if not cap.isOpened():
                dispatcher.utter_message(text="Camera could not be opened.")
                return []

            qr_code_data = None

            while cap.isOpened():
                ret, frame = cap.read()
                if not ret:
                    dispatcher.utter_message(text="Failed to capture video frame.")
                    break

                # Decode QR code from the frame
                decoded_objects = decode(frame)
                for obj in decoded_objects:
                    qr_code_data = obj.data.decode('utf-8')
                    dispatcher.utter_message(text=f"QR Code Data: {qr_code_data}")
                    break

                if qr_code_data:
                    break

            cap.release()
            cv2.destroyAllWindows()

            if not qr_code_data:
                dispatcher.utter_message(text="No QR code detected.")

        except Exception as e:
            dispatcher.utter_message(text=f"An error occurred: {e}")
            print(f"Error in ActionScanQRCode: {e}")

        return []
    def name(self) -> str:
        return "action_scan_qr_code"

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict[str, any]) -> list:
        try:
            dispatcher.utter_message(text="Please show your boarding QR code near the camera properly.")
            
            # Capture video from the camera
            cap = cv2.VideoCapture(0)
            if not cap.isOpened():
                dispatcher.utter_message(text="Camera could not be opened.")
                return []

            qr_code_data = None

            while cap.isOpened():
                ret, frame = cap.read()
                if not ret:
                    dispatcher.utter_message(text="Failed to capture video frame.")
                    break

                # Decode QR code from the frame
                decoded_objects = decode(frame)
                for obj in decoded_objects:
                    qr_code_data = obj.data.decode('utf-8')
                    print(f"QR Code Data: {qr_code_data}")
                    break

                if qr_code_data:
                    break

            cap.release()
            cv2.destroyAllWindows()

            if not qr_code_data:
                dispatcher.utter_message(text="No QR code detected.")
            else:
                dispatcher.utter_message(text=f"Detected QR Code Data: {qr_code_data}")

        except Exception as e:
            dispatcher.utter_message(text=f"An error occurred: {e}")
            print(f"Error in ActionScanQRCode: {e}")

        return []
    def name(self) -> str:
        return "action_scan_qr_code"

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict[str, any]) -> list:
        try:
            dispatcher.utter_message(text="Please show your boarding QR code near the camera properly.")
            
            # Capture video from the camera
            cap = cv2.VideoCapture(0)
            if not cap.isOpened():
                dispatcher.utter_message(text="Camera could not be opened.")
                return []

            qr_code_data = None

            while cap.isOpened():
                ret, frame = cap.read()
                if not ret:
                    dispatcher.utter_message(text="Failed to capture video frame.")
                    break

                # Decode QR code from the frame
                decoded_objects = decode(frame)
                for obj in decoded_objects:
                    qr_code_data = obj.data.decode('utf-8')
                    print(f"QR Code Data: {qr_code_data}")
                    break

                if qr_code_data:
                    break

            cap.release()
            cv2.destroyAllWindows()

            if not qr_code_data:
                dispatcher.utter_message(text="No QR code detected.")
            else:
                dispatcher.utter_message(text=f"Detected QR Code Data: {qr_code_data}")

        except Exception as e:
            dispatcher.utter_message(text=f"An error occurred: {e}")

        return []
    def name(self) -> Text:
        return "action_scan_qr_code"

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        try:
            dispatcher.utter_message(text="Please show your boarding QR code near the camera properly.")
            
            # Capture video from the camera
            cap = cv2.VideoCapture(0)
            qr_code_data = None

            if not cap.isOpened():
                dispatcher.utter_message(text="Failed to open camera.")
                return []

            while cap.isOpened():
                ret, frame = cap.read()
                if not ret:
                    dispatcher.utter_message(text="Failed to capture video frame.")
                    break

                # Decode QR code from the frame
                decoded_objects = decode(frame)
                for obj in decoded_objects:
                    qr_code_data = obj.data.decode('utf-8')
                    print(f"QR Code Data: {qr_code_data}")
                    break

                if qr_code_data:
                    break

            cap.release()
            cv2.destroyAllWindows()

            if not qr_code_data:
                dispatcher.utter_message(text="No QR code detected.")
                return []

            dispatcher.utter_message(text=f"Detected QR Code Data: {qr_code_data}")
        except Exception as e:
            dispatcher.utter_message(text=f"An error occurred: {e}")
        return []

    def name(self) -> Text:
        return "action_scan_qr_code"

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        try:
            dispatcher.utter_message(text="Please show your boarding QR code near the camera properly.")
            
            # Capture video from the camera
            cap = cv2.VideoCapture(0)
            qr_code_data = None

            if not cap.isOpened():
                dispatcher.utter_message(text="Failed to open camera.")
                return []

            while cap.isOpened():
                ret, frame = cap.read()
                if not ret:
                    dispatcher.utter_message(text="Failed to capture video frame.")
                    break

                # Decode QR code from the frame
                decoded_objects = decode(frame)
                for obj in decoded_objects:
                    qr_code_data = obj.data.decode('utf-8')
                    print(f"QR Code Data: {qr_code_data}")
                    break

                if qr_code_data:
                    break

            cap.release()
            cv2.destroyAllWindows()

            if not qr_code_data:
                dispatcher.utter_message(text="No QR code detected.")
                return []

            dispatcher.utter_message(text=f"Detected QR Code Data: {qr_code_data}")
        except Exception as e:
            dispatcher.utter_message(text=f"An error occurred: {e}")
        return []

    def name(self) -> Text:
        return "action_scan_qr_code"

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        # Path to your Arrival.json file
        json_file_path = "/Users/hasu/Documents/Rasatraining 2/Arrival.json"

        dispatcher.utter_message(text="Please show your boarding QR code near the camera properly.")
        
        # Capture video from the camera
        cap = cv2.VideoCapture(0)
        qr_code_data = None

        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break

            # Decode QR code from the frame
            decoded_objects = decode(frame)
            for obj in decoded_objects:
                qr_code_data = obj.data.decode('utf-8')
                print(f"QR Code Data: {qr_code_data}")
                break

            if qr_code_data:
                break

        cap.release()
        cv2.destroyAllWindows()

        if not qr_code_data:
            dispatcher.utter_message(text="No QR code detected.")
            return []

        # Read flight data from JSON file
        try:
            with open(json_file_path, 'r') as file:
                flight_data = json.load(file)
        except FileNotFoundError:
            dispatcher.utter_message(text="Flight data file not found.")
            return []
        except json.JSONDecodeError:
            dispatcher.utter_message(text="Error decoding flight data.")
            return []

        # Find flight information based on QR code data
        flight_info = None
        for flight in flight_data.get('flights', []):
            if qr_code_data == flight.get('flight'):
                flight_info = flight
                break

        if flight_info:
            message = (f"Flight Number: {flight_info.get('flight')}\n"
                       f"Airlines: {flight_info.get('airlines')}\n"
                       f"Scheduled Arrival: {flight_info.get('scheduled_time_of_arrival')}\n"
                       f"Estimated Arrival: {flight_info.get('estimated_time_of_arrival')}\n"
                       f"Status: {flight_info.get('status')}")
            dispatcher.utter_message(text=message)
        else:
            dispatcher.utter_message(text="Flight information not found.")

        return []

    def name(self) -> Text:
        return "action_scan_qr_code"

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        # Path to your Arrival.json file
        json_file_path = "/Users/hasu/Documents/Rasatraining 2/Arrival.json"

        dispatcher.utter_message(text="Please show your boarding QR code near the camera properly.")
        
        # Capture video from the camera
        cap = cv2.VideoCapture(0)
        qr_code_data = None

        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break

            # Decode QR code from the frame
            decoded_objects = decode(frame)
            for obj in decoded_objects:
                qr_code_data = obj.data.decode('utf-8')
                print(f"QR Code Data: {qr_code_data}")
                break

            if qr_code_data:
                break

        cap.release()
        cv2.destroyAllWindows()

        if not qr_code_data:
            dispatcher.utter_message(text="No QR code detected.")
            return []

        # Read flight data from JSON file
        try:
            with open(json_file_path, 'r') as file:
                flight_data = json.load(file)
        except FileNotFoundError:
            dispatcher.utter_message(text="Flight data file not found.")
            return []
        except json.JSONDecodeError:
            dispatcher.utter_message(text="Error decoding flight data.")
            return []

        # Find flight information based on QR code data
        flight_info = None
        for flight in flight_data.get('flights', []):
            if qr_code_data == flight.get('flight'):
                flight_info = flight
                break

        if flight_info:
            message = (f"Flight Number: {flight_info.get('flight')}\n"
                       f"Airlines: {flight_info.get('airlines')}\n"
                       f"Scheduled Arrival: {flight_info.get('scheduled_time_of_arrival')}\n"
                       f"Estimated Arrival: {flight_info.get('estimated_time_of_arrival')}\n"
                       f"Status: {flight_info.get('status')}")
            dispatcher.utter_message(text=message)
        else:
            dispatcher.utter_message(text="Flight information not found.")

        return []
    
class ActionDisplayFlightInformation(Action):

    def name(self) -> Text:
        return "action_display_flight_information"

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        try:
            with open("/Users/hasu/Documents/Rasatraining 2/Arrival.json") as f:
                flight_data = json.load(f)
        except FileNotFoundError:
            dispatcher.utter_message(text="Flight data file not found.")
            return []
        except json.JSONDecodeError:
            dispatcher.utter_message(text="Error decoding flight data.")
            return []

        if not flight_data.get("flights"):
            dispatcher.utter_message(text="No flight data available.")
            return []

        flights_info = ""
        for flight in flight_data.get("flights", []):
            flights_info += (f"Airline: {flight['airlines']}\n"
                             f"Flight Number: {flight['flight']}\n"
                             f"Origin: {flight['origin']}\n"
                             f"Scheduled Time of Arrival: {flight['scheduled_time_of_arrival']}\n"
                             f"Estimated Time of Arrival: {flight['estimated_time_of_arrival']}\n"
                             f"Status: {flight['status']}\n\n")

        dispatcher.utter_message(text=f"Here is the flight information:\n\n{flights_info}")
        return []
    

class ActionNavigateToLocation(Action):
    def name(self) -> Text:
        return "action_navigate_to_location"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        intent = tracker.latest_message['intent']['name']

        # Map intents to lane detection triggers
        if intent in ['ask_for_gate', 'ask_for_terminal', 'ask_for_boarding_gate', 'ask_for_toilet']:
            try:
                print("Starting lane detection script...")  
                subprocess.run(["python", "detection.py"], check=True)               
                dispatcher.utter_message(text="Starting navigation to the requested location.")
            except Exception as e:
                print(f"Error running detection script: {e}")  # Debug: Print error if subprocess fails
                dispatcher.utter_message(text="Failed to start navigation. Please try again.")
        
        return []