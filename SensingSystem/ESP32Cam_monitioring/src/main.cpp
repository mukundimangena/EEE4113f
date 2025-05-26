#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include "esp_camera.h"
#include "esp_timer.h"
#include "esp_http_server.h"
#include "fb_gfx.h"
#include "driver/rtc_io.h"
#include <esp_sleep.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <base64.h>

// Pin definitions for ESP32-CAM
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// Trigger pin from main ESP32
#define TRIGGER_PIN       2

// LED flash pin
#define FLASH_LED_PIN     4

// Configuration
const char* ssid = "Redmi A2";
const char* password = "12345678q";
const char* serverUrl = "http://192.168.213/"

// Web server
WebServer server(80);

// Camera configuration
camera_config_t config;

// State variables
bool cameraReady = false;
bool streamActive = false;
unsigned long lastTriggerTime = 0;
int captureCount = 0;

// Function prototypes
void setupCamera();
void handleRoot();
void handleCapture();
void handleStream();
void captureAndSave();
void handleTrigger();
void enterLightSleep();
void sendImageToServer(camera_fb_t* fb);

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  // Initialize trigger pin
  pinMode(TRIGGER_PIN, INPUT_PULLUP);
  pinMode(FLASH_LED_PIN, OUTPUT);
  digitalWrite(FLASH_LED_PIN, LOW);

  // Setup camera
  setupCamera();

  // Connect to WiFi
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  // Start web server
  server.on("/", handleRoot);
  server.on("/capture", handleCapture);
  server.on("/stream", handleStream);
  
  server.begin();
  Serial.println("HTTP server started");

  // Setup mDNS
  if (MDNS.begin("esp32cam")) {
    Serial.println("MDNS responder started");
  }

  Serial.print("Camera Stream Ready! Go to: http://");
  Serial.print(WiFi.localIP());
  Serial.println();

  
  attachInterrupt(digitalPinToInterrupt(TRIGGER_PIN), handleTrigger, FALLING);

  
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_2, 0); // Wake on trigger pin LOW

  Serial.println("ESP32-CAM ready for triggers");
}

void loop() {
  server.handleClient();
  
  // Check for trigger signal
  if (digitalRead(TRIGGER_PIN) == LOW) {
    handleTriggerCapture();
  }
  
  // Enter light sleep if no activity for 30 seconds
  if (millis() - lastTriggerTime > 30000 && !streamActive) {
    Serial.println("Entering light sleep mode");
    enterLightSleep();
  }
  
  delay(10);
}

void setupCamera() {
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // Camera quality settings
  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA; // 1600x1200
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA; // 800x600
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // Initialize camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  // Camera sensor settings
  sensor_t* s = esp_camera_sensor_get();
  if (s->id.PID == OV2640_PID) {
    s->set_vflip(s, 1);        // Flip vertically
    s->set_brightness(s, 1);   // Increase brightness slightly
    s->set_saturation(s, -2);  // Lower saturation for better night vision
    s->set_contrast(s, 2);     // Increase contrast
  }

  cameraReady = true;
  Serial.println("Camera initialized successfully");
}

void handleRoot() {
  String html = "<!DOCTYPE html><html>";
  html += "<head><title>ESP32-CAM Predator Detection</title></head>";
  html += "<body><h1>ESP32-CAM Predator Detection System</h1>";
  html += "<p><a href=\"/capture\">Capture Photo</a></p>";
  html += "<p><a href=\"/stream\">Live Stream</a></p>";
  html += "<p>Status: " + String(cameraReady ? "Ready" : "Not Ready") + "</p>";
  html += "<p>Captures taken: " + String(captureCount) + "</p>";
  html += "</body></html>";
  
  server.send(200, "text/html", html);
}

void handleCapture() {
  if (!cameraReady) {
    server.send(500, "text/plain", "Camera not ready");
    return;
  }

  // Flash LED for better visibility
  digitalWrite(FLASH_LED_PIN, HIGH);
  delay(100);

  camera_fb_t* fb = esp_camera_fb_get();
  
  digitalWrite(FLASH_LED_PIN, LOW);

  if (!fb) {
    server.send(500, "text/plain", "Camera capture failed");
    return;
  }

  // Send image to server
  sendImageToServer(fb);

  server.sendHeader("Content-Disposition", "inline; filename=capture.jpg");
  server.send_P(200, "image/jpeg", (const char*)fb->buf, fb->len);

  esp_camera_fb_return(fb);
  captureCount++;
  lastTriggerTime = millis();
  
  Serial.printf("Photo captured and served. Size: %d bytes\n", fb->len);
}

void handleStream() {
  WiFiClient client = server.client();
  String response = "HTTP/1.1 200 OK\r\n";
  response += "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
  server.sendContent(response);

  streamActive = true;
  lastTriggerTime = millis();

  while (client.connected()) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      break;
    }

    String header = "--frame\r\n";
    header += "Content-Type: image/jpeg\r\n";
    header += "Content-Length: " + String(fb->len) + "\r\n\r\n";

    server.sendContent(header);
    
    client.write((char*)fb->buf, fb->len);
    server.sendContent("\r\n");

    esp_camera_fb_return(fb);

    if (!client.connected()) {
      break;
    }
    delay(33); // ~30 FPS
  }

  streamActive = false;
  Serial.println("Stream ended");
}

void handleTriggerCapture() {
  if (!cameraReady) {
    Serial.println("Camera not ready for trigger capture");
    return;
  }

  Serial.println("Trigger received - capturing image");
  lastTriggerTime = millis();

  // Flash LED for night captures
  digitalWrite(FLASH_LED_PIN, HIGH);
  delay(50); // Brief flash

  // Capture multiple frames for better chance of good image
  for (int i = 0; i < 3; i++) {
    camera_fb_t* fb = esp_camera_fb_get();
    
    if (fb) {
      Serial.printf("Trigger capture %d successful. Size: %d bytes\n", i+1, fb->len);
      sendImageToServer(fb); // Send each captured image to the server
      
      if (i == 2) {
        captureCount++;
      }
      
      esp_camera_fb_return(fb);
    } else {
      Serial.printf("Trigger capture %d failed\n", i+1);
    }
    
    delay(200); // Small delay between captures
  }

  digitalWrite(FLASH_LED_PIN, LOW);
  
  Serial.println("Trigger capture sequence completed");
}

void IRAM_ATTR handleTrigger() {
  // Empty for now, as handleTriggerCapture is called in loop
}

void enterLightSleep() {
  Serial.println("Entering light sleep mode");
  Serial.flush();
  
  // Configure wake up sources
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_2, 0); // Wake on trigger pin
  esp_sleep_enable_timer_wakeup(60000000); // Wake up every 60 seconds to check WiFi
  
  // Enter light sleep
  esp_light_sleep_start();
  
  Serial.println("Woke up from light sleep");
  lastTriggerTime = millis();
}

void captureAndSave() {
  if (!cameraReady) return;
  
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }
  
  // Send image to server
  sendImageToServer(fb);
  
  // Add timestamp or unique identifier
  String filename = "predator_" + String(millis()) + ".jpg";
  
  Serial.println("Image captured: " + filename);
  
  esp_camera_fb_return(fb);
  captureCount++;
}

void sendImageToServer(camera_fb_t* fb) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected - cannot send image");
    return;
  }

  // Encode image to base64
  String encodedImage = base64::encode(fb->buf, fb->len);
  Serial.printf("Base64 encoded image size: %d bytes\n", encodedImage.length());

  // Create JSON payload
  DynamicJsonDocument doc(1024 + encodedImage.length()); // Adjust size based on image
  doc["timestamp"] = millis();
  doc["capture_count"] = captureCount;
  doc["image"] = encodedImage;
  doc["filename"] = "predator_" + String(millis()) + ".jpg";

  String jsonPayload;
  serializeJson(doc, jsonPayload);

  // Send HTTP POST request
  HTTPClient http;
  http.begin(serverUrl);
  http.addHeader("Content-Type", "application/json");

  int httpCode = http.POST(jsonPayload);
  
  if (httpCode > 0) {
    Serial.printf("HTTP POST Response code: %d\n", httpCode);
    if (httpCode == HTTP_CODE_OK) {
      String response = http.getString();
      Serial.println("Server response: " + response);
    }
  } else {
    Serial.printf("HTTP POST failed, error: %s\n", http.errorToString(httpCode).c_str());
  }

  http.end();
}