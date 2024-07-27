import qrcode
import json
import os

# Path to your Arrival.json file
json_file_path = "/Users/hasu/Documents/Rasatraining 2/Arrival.json"

# Load flight data from JSON file
with open(json_file_path, 'r') as file:
    data = json.load(file)

# Ensure output directory exists
output_dir = "qrcodes"
os.makedirs(output_dir, exist_ok=True)

# Generate QR codes for each flight
for flight in data['flights']:
    # Format the flight details for the QR code
    flight_info = (
        f"Airlines: {flight['airlines']}\n"
        f"Flight: {flight['flight']}\n"
        f"Scheduled Arrival: {flight['scheduled_time_of_arrival']}\n"
        f"Estimated Arrival: {flight['estimated_time_of_arrival']}\n"
        f"Origin: {flight['origin']}\n"
        f"Status: {flight['status']}"
    )

    # Create QR code
    qr = qrcode.QRCode(
        version=1,
        error_correction=qrcode.constants.ERROR_CORRECT_L,
        box_size=10,
        border=4,
    )
    qr.add_data(flight_info)
    qr.make(fit=True)

    # Create an image from the QR Code instance
    img = qr.make_image(fill='black', back_color='white')

    # Save the image
    file_name = f"{flight['flight']}.png"
    file_path = os.path.join(output_dir, file_name)
    img.save(file_path)

    print(f"QR code for flight {flight['flight']} saved as '{file_path}'")
