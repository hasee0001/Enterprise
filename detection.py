from flask import Flask, request, jsonify
import cv2
import numpy as np
import serial
import time
import threading
from pyzbar import pyzbar

app = Flask(__name__)

# Serial communication thread class
class SerialThread(threading.Thread):
    def __init__(self):
        super(SerialThread, self).__init__()
        self.lock = threading.Lock()
        self.running = True
        self.command = None
        try:
            self.ser = serial.Serial('/dev/cu.usbmodem11201', 115200, timeout=.025, write_timeout=.025)
            time.sleep(2)
        except serial.SerialException as e:
            print(f"Error: {e}")
            exit()

    def run(self):
        while self.running:
            if self.command:
                with self.lock:
                    command_to_send = self.command
                    self.command = None
                try:
                    if self.ser.isOpen():
                        self.ser.write(command_to_send.encode())
                        self.ser.flush()
                    else:
                        print("Serial port is not open.")
                except serial.SerialException as e:
                    print(f"Serial communication error: {e}")
            time.sleep(0.025)

    def send_command(self, command):
        with self.lock:
            self.command = command

    def stop(self):
        self.running = False
        self.ser.close()

# Initialize serial communication
serial_thread = SerialThread()

# Video capture thread class
class VideoCaptureThread(threading.Thread):
    def __init__(self, src=1):
        super(VideoCaptureThread, self).__init__()
        self.capture = cv2.VideoCapture(src)
        self.capture.set(3, 300)
        self.capture.set(4, 300)
        self.lock = threading.Lock()
        self.running = True
        self.frame = None

    def run(self):
        while self.running:
            ret, frame = self.capture.read()
            if ret:
                with self.lock:
                    self.frame = frame

    def read(self):
        with self.lock:
            return self.frame

    def stop(self):
        self.running = False
        self.capture.release()

# Frame processing thread class
class FrameProcessingThread(threading.Thread):
    def __init__(self, video_thread):
        super(FrameProcessingThread, self).__init__()
        self.video_thread = video_thread
        self.running = True
        self.stop_robot = False
        self.stop_time = 0
        self.stop_duration = 0
        self.last_detected_time = 0
        self.qr_cooldown = 22
        self.allow_qr_detection = True

    def run(self):
        while self.running:
            frame = self.video_thread.read()
            if frame is None:
                continue

            # Line following logic (simplified)
            cropped = frame[100:250, 90:200]
            hsv = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, np.array([0, 0, 0]), np.array([180, 255, 70]))
            mask = cv2.dilate(mask, None, iterations=2)
            mask = cv2.erode(mask, None, iterations=2)
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                c = max(contours, key=cv2.contourArea)
                M = cv2.moments(c)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    if 45 < cx < 80:
                        serial_thread.send_command("MOVE -6 -6.9\n")
                    elif cx < 45:
                        serial_thread.send_command("MOVE -6.9 -4.5\n")
                    elif cx > 80:
                        serial_thread.send_command("MOVE -4.5 -6.9\n")
                else:
                    serial_thread.send_command("MOVE 0 0\n")

            time.sleep(0.1)

    def stop(self):
        self.running = False

# Start lane detection process
@app.route('/start_detection', methods=['POST'])
def start_detection():
    video_thread = VideoCaptureThread()
    processing_thread = FrameProcessingThread(video_thread)
    
    serial_thread.start()
    video_thread.start()
    processing_thread.start()
    
    return jsonify({"status": "Detection started"})

# Stop lane detection process
@app.route('/stop_detection', methods=['POST'])
def stop_detection():
    serial_thread.stop()
    video_thread.stop()
    processing_thread.stop()
    
    return jsonify({"status": "Detection stopped"})

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=5000)
