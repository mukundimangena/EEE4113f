#include <Wire.h>
#include <ArduinoJson.h>
#include <esp_sleep.h>
#include <driver/rtc_io.h>
#include <WiFi.h>
#include <HTTPClient.h>

#define RCWL_PIN 2          // RCWL-0516 motion sensor output
#define PIR_PIN 4           // HC-SR501 PIR sensor output
#define VIBRATION_PIN 5     // SW420 vibration sensor output
#define ULTRASONIC_TRIG 12  // HC-SR04 trigger pin
#define ULTRASONIC_ECHO 13  // HC-SR04 echo pin
#define CAM_TRIGGER_PIN 14  // Trigger signal to ESP32-CAM
#define SDA_PIN 21          // I2C SDA for TMP102
#define SCL_PIN 22          // I2C SCL for TMP102

// TMP102 I2C Address
#define TMP102_ADDR 0x48

// Configuration Constants
#define PREDATOR_DISTANCE_THRESHOLD 50  // cm - distance threshold for predator detection
#define TEMP_LOG_INTERVAL 300000       // 5 minutes in milliseconds
#define DEEP_SLEEP_TIME 30             // seconds
#define MAX_DETECTION_DISTANCE 400     // cm

// WiFi Configuration 
const char* ssid = "Redmi A2";
const char* password = "12345678q";
const char* serverUrl = "http://192.168.213.130/data"; // Replace with your server endpoint

// System State Variables
enum SystemState {
  IDLE,
  MOTION_DETECTED,
  ANALYZING,
  ALERT_TRIGGERED,
  DEEP_SLEEP_MODE
};

SystemState currentState = IDLE;
unsigned long lastTempLog = 0;
unsigned long lastStateChange = 0;
bool pirEnabled = true;
int consecutiveDetections = 0;

// Sensor Data Structure
struct SensorData {
  float temperature;
  bool rcwlMotion;
  bool pirMotion;
  bool vibrationDetected;
  float distance;
  unsigned long timestamp;
};

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("Predator Detection System Starting...");
  
  // Initialize pins
  initializePins();
  
  // Initialize I2C for TMP102
  Wire.begin(SDA_PIN, SCL_PIN);
  
  // Initialize WiFi
  initializeWiFi();
  
  // Configure wake up source for deep sleep
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_2, 1); // Wake on RCWL trigger
  
  // Handle wake-up reason
  handleWakeUp();
  
  Serial.println("System initialized successfully");
  currentState = IDLE;
}

void loop() {
  SensorData data = readAllSensors();
  
  switch (currentState) {
    case IDLE:
      handleIdleState(data);
      break;
      
    case MOTION_DETECTED:
      handleMotionDetectedState(data);
      break;
      
    case ANALYZING:
      handleAnalyzingState(data);
      break;
      
    case ALERT_TRIGGERED:
      handleAlertTriggeredState(data);
      break;
      
    case DEEP_SLEEP_MODE:
      enterDeepSleep();
      break;
  }
  
  // Periodic temperature logging
  if (millis() - lastTempLog > TEMP_LOG_INTERVAL) {
    logTemperatureData(data.temperature);
    lastTempLog = millis();
  }
  
  // Check for tampering/digging continuously
  if (data.vibrationDetected) {
    handleTamperingAlert(data);
  }
  
  delay(100); // Small delay to prevent excessive polling
}

void initializePins() {
  pinMode(RCWL_PIN, INPUT);
  pinMode(PIR_PIN, INPUT);
  pinMode(VIBRATION_PIN, INPUT);
  pinMode(ULTRASONIC_TRIG, OUTPUT);
  pinMode(ULTRASONIC_ECHO, INPUT);
  pinMode(CAM_TRIGGER_PIN, OUTPUT);
  
  digitalWrite(ULTRASONIC_TRIG, LOW);
  digitalWrite(CAM_TRIGGER_PIN, LOW);
  
  Serial.println("Pins initialized");
}

void initializeWiFi() {
  WiFi.begin(ssid, password);
  int attempts = 0;
  
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.println("WiFi connected successfully");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println();
    Serial.println("WiFi connection failed - continuing in offline mode");
  }
}

SensorData readAllSensors() {
  SensorData data;
  
  data.timestamp = millis();
  data.temperature = readTMP102();
  data.rcwlMotion = digitalRead(RCWL_PIN);
  data.pirMotion = digitalRead(PIR_PIN);
  data.vibrationDetected = digitalRead(VIBRATION_PIN);
  data.distance = readUltrasonicDistance();
  
  return data;
}

float readTMP102() {
  Wire.beginTransmission(TMP102_ADDR);
  Wire.write(0x00); // Temperature register
  if (Wire.endTransmission() != 0) {
    Serial.println("TMP102 communication error");
    return -999.0; // Error value
  }
  
  Wire.requestFrom(TMP102_ADDR, 2);
  if (Wire.available() == 2) {
    int16_t temp = (Wire.read() << 8) | Wire.read();
    temp >>= 4; // TMP102 uses 12-bit resolution
    return temp * 0.0625; // Convert to Celsius
  }
  
  return -999.0; // Error value
}

float readUltrasonicDistance() {
  digitalWrite(ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG, LOW);
  
  long duration = pulseIn(ULTRASONIC_ECHO, HIGH, 30000); // 30ms timeout
  
  if (duration == 0) {
    return MAX_DETECTION_DISTANCE + 1; // Return out of range value
  }
  
  float distance = duration * 0.034 / 2; // Convert to cm
  return distance;
}

void handleIdleState(const SensorData& data) {
  if (data.rcwlMotion) {
    Serial.println("RCWL motion detected. Checking PIR...");
    changeState(MOTION_DETECTED);
    consecutiveDetections = 0;
  }
  
  // Enter deep sleep if no activity for extended period
  if (millis() - lastStateChange > 60000) { // 1 minute of inactivity
    Serial.println("Entering deep sleep mode");
    changeState(DEEP_SLEEP_MODE);
  }
}

void handleMotionDetectedState(const SensorData& data) {
  Serial.println("Confirming motion with PIR sensor...");
  
  // Wait a moment for PIR to stabilize
  delay(500);
  
  bool pirConfirmed = digitalRead(PIR_PIN);
  
  if (pirConfirmed) {
    Serial.println("PIR confirmed motion.");
    changeState(ANALYZING);
  } else {
    consecutiveDetections++;
    if (consecutiveDetections > 3) {
      Serial.println("Multiple RCWL triggers without PIR confirmation - possible false alarm");
      changeState(IDLE);
    } else {
      // Wait a bit and check again
      delay(1000);
      if (!data.rcwlMotion) {
        changeState(IDLE);
      }
    }
  }
}

void handleAnalyzingState(const SensorData& data) {
  Serial.println("Analyzing threat level...");
  Serial.printf("Distance measured: %.2f cm\n", data.distance);
  
  if (data.distance < PREDATOR_DISTANCE_THRESHOLD && data.distance > 0) {
    Serial.println("PREDATOR DETECTED! Triggering alert system");
    triggerCameraCapture();
    logDetectionEvent(data);
    transmitAlert(data);
    changeState(ALERT_TRIGGERED);
  } else if (data.distance > MAX_DETECTION_DISTANCE) {
    Serial.println("Target too far or no reliable reading - returning to idle");
    changeState(IDLE);
  } else {
    Serial.println("Motion detected but distance suggests non-threat - monitoring");
    delay(2000); // Continue monitoring for 2 seconds
    if (!digitalRead(RCWL_PIN) && !digitalRead(PIR_PIN)) {
      changeState(IDLE);
    }
  }
}

void handleAlertTriggeredState(const SensorData& data) {
  Serial.println("Alert state - monitoring for continued threat");
  
  // Stay in alert state for 30 seconds
  static unsigned long alertStartTime = millis();
  
  if (millis() - alertStartTime > 30000) {
    Serial.println("Alert timeout - returning to idle");
    changeState(IDLE);
    alertStartTime = millis(); // Reset for next alert
  }
  
  // Continue monitoring for additional threats
  if (data.rcwlMotion && data.distance < PREDATOR_DISTANCE_THRESHOLD) {
    Serial.println("Continued threat detected");
    triggerCameraCapture();
    transmitAlert(data);
    alertStartTime = millis(); // Extend alert period
  }
}

void handleTamperingAlert(const SensorData& data) {
  Serial.println("TAMPERING/DIGGING DETECTED!");
  
  // Create tampering alert message
  StaticJsonDocument<200> alertDoc;
  alertDoc["type"] = "tampering";
  alertDoc["timestamp"] = data.timestamp;
  alertDoc["temperature"] = data.temperature;
  alertDoc["vibration"] = true;
  
  String alertMessage;
  serializeJson(alertDoc, alertMessage);
  
  Serial.println("Tampering Alert: " + alertMessage);
  
  // Trigger camera for tampering
  triggerCameraCapture();
  
  // Send alert if WiFi is available
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(serverUrl);
    http.addHeader("Content-Type", "application/json");
    
    int httpCode = http.POST(alertMessage);
    
    if (httpCode > 0) {
      Serial.printf("Tampering Alert HTTP Response code: %d\n", httpCode);
      if (httpCode == HTTP_CODE_OK) {
        String response = http.getString();
        Serial.println("Server response: " + response);
      }
    } else {
      Serial.printf("Tampering Alert HTTP POST failed, error: %s\n", http.errorToString(httpCode).c_str());
    }
    
    http.end();
  }
}

void triggerCameraCapture() {
  Serial.println("Triggering ESP32-CAM capture");
  digitalWrite(CAM_TRIGGER_PIN, HIGH);
  delay(100); // Hold trigger signal
  digitalWrite(CAM_TRIGGER_PIN, LOW);
}

void logDetectionEvent(const SensorData& data) {
  StaticJsonDocument<300> logDoc;
  logDoc["timestamp"] = data.timestamp;
  logDoc["event"] = "predator_detection";
  logDoc["temperature"] = data.temperature;
  logDoc["distance"] = data.distance;
  logDoc["rcwl_motion"] = data.rcwlMotion;
  logDoc["pir_motion"] = data.pirMotion;
  logDoc["vibration"] = data.vibrationDetected;
  
  String logMessage;
  serializeJson(logDoc, logMessage);
  
  Serial.println("Detection Log: " + logMessage);
  
  // Here you could implement SD card logging or EEPROM storage
}

void logTemperatureData(float temperature) {
  Serial.printf("Temperature: %.2f°C at %lu ms\n", temperature, millis());
  
  // Implement periodic temperature logging storage here
}

void transmitAlert(const SensorData& data) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected - alert transmission skipped");
    return;
  }
  
  StaticJsonDocument<400> alertDoc;
  alertDoc["type"] = "predator_alert";
  alertDoc["timestamp"] = data.timestamp;
  alertDoc["location"] = "Chicken Coop"; // Configure as needed
  alertDoc["threat_level"] = "HIGH";
  alertDoc["distance"] = data.distance;
  alertDoc["temperature"] = data.temperature;
  alertDoc["sensors"]["rcwl"] = data.rcwlMotion;
  alertDoc["sensors"]["pir"] = data.pirMotion;
  alertDoc["sensors"]["vibration"] = data.vibrationDetected;
  
  String alertMessage;
  serializeJson(alertDoc, alertMessage);
  
  Serial.println("ALERT: " + alertMessage);
  
  HTTPClient http;
  http.begin(serverUrl);
  http.addHeader("Content-Type", "application/json");
  
  int httpCode = http.POST(alertMessage);
  
  if (httpCode > 0) {
    Serial.printf("Alert HTTP Response code: %d\n", httpCode);
    if (httpCode == HTTP_CODE_OK) {
      String response = http.getString();
      Serial.println("Server response: " + response);
    }
  } else {
    Serial.printf("Alert HTTP POST failed, error: %s\n", http.errorToString(httpCode).c_str());
  }
  
  http.end();
}

void changeState(SystemState newState) {
  Serial.printf("State change: %d -> %d\n", currentState, newState);
  currentState = newState;
  lastStateChange = millis();
}

void enterDeepSleep() {
  Serial.println("Entering deep sleep...");
  Serial.flush();
  
  // Configure pins for wake up
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_2, 1); // Wake on RCWL motion
  
  // Enter deep sleep
  esp_deep_sleep_start();
}

void handleWakeUp() {
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  
  switch(wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0:
      Serial.println("Woke up from external signal (RCWL motion)");
      currentState = MOTION_DETECTED;
      break;
    case ESP_SLEEP_WAKEUP_TIMER:
      Serial.println("Woke up from timer");
      break;
    default:
      Serial.println("Woke up for unknown reason");
      break;
  }
}