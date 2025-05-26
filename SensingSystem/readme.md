Predator Detection System



This project implements a predator detection system using two ESP32-based microcontrollers: a main ESP32 (e.g., ESP32 DevKit) and an ESP32-CAM. The system is designed to monitor penguin colomies for potential predators using multiple sensors and a camera. When a threat is detected, the system triggers image captures and sends sensor data and images to a server via HTTP POST requests in JSON format.
Components

Main ESP32 (Predator Detection):
Monitors sensors: RCWL-0516 (microwave motion), HC-SR501 (PIR motion), SW420 (vibration), HC-SR04 (ultrasonic distance), and TMP102 (temperature).
Triggers the ESP32-CAM to capture images when a predator is detected.
Sends sensor data and alerts to a server.


ESP32-CAM (Camera System):
Captures images when triggered by the main ESP32.
Encodes images in base64 and sends them to a server as JSON payloads.
Hosts a web server for manual photo capture and live streaming.


Server:
Receives JSON payloads containing sensor data and images.
Stores alerts and images for analysis.



How It Works

Sensor Monitoring (Main ESP32):

The main ESP32 continuously monitors sensor inputs:
RCWL-0516: Detects motion using microwave radar.
HC-SR501 (PIR): Confirms motion to reduce false positives.
SW420: Detects vibration (e.g., tampering or digging).
HC-SR04: Measures distance to determine if a detected object is within a threat range (default: <50 cm).
TMP102: Logs ambient temperature periodically (every 5 minutes).


The system uses a state machine with five states: IDLE, MOTION_DETECTED, ANALYZING, ALERT_TRIGGERED, and DEEP_SLEEP_MODE.


Threat Detection Logic:

IDLE: Waits for RCWL motion detection. If no activity occurs for 1 minute, enters DEEP_SLEEP_MODE.
MOTION_DETECTED: Confirms motion with PIR sensor to avoid false alarms.
ANALYZING: Checks ultrasonic distance. If <50 cm, triggers an alert; otherwise, monitors further or returns to IDLE.
ALERT_TRIGGERED: Activates the ESP32-CAM, sends alerts, and monitors for continued threats (30-second timeout).
DEEP_SLEEP_MODE: Enters deep sleep to conserve power, waking on



