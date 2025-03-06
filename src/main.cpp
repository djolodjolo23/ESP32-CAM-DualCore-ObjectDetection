#include <Arduino.h>
#include "OV2640.hpp"
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <WiFiClient.h>
#include "detected_objects.hpp"

#define CAMERA_MODEL_AI_THINKER
// #define CAMERA_MODEL_WROVER_KIT
// #define CAMERA_MODEL_ESP_EYE
// #define CAMERA_MODEL_M5STACK_PSRAM
// #define CAMERA_MODEL_M5STACK_WIDE

#include "camera_pins.h"
#include "secrets.h"
#include "server.hpp"
#include "inference.hpp"
#include "shared_buffer.hpp"

OV2640 cam;
extern SharedBuffer sharedBuffer;
std::vector<DetectedObject> detectedObjects;

void setup() {
  Serial.begin(115200);
  Serial.println("Starting ESP32-CAM with inference...");
  
  sharedBuffer.frame = nullptr;
  sharedBuffer.mutex = xSemaphoreCreateMutex();
  sharedBuffer.hasNewFrame = false;
  sharedBuffer.hasNewResult = false;
  
  // Initialize camera
  camera_config_t config;
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
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_QVGA;  
  config.jpeg_quality = 12;
  config.fb_count = 2;

  size_t width = resolution[config.frame_size].width;
  size_t height = resolution[config.frame_size].height;

  Serial.println("Camera config:");
  Serial.printf("  Resolution: %dx%d\n", width, height);

  cam.init(config); 
  cam.flip(true, false); // flip vertically
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  if (MDNS.begin("esp32-cam")) {
    Serial.println("MDNS responder started");
  }
  
  IPAddress ip = WiFi.localIP();
  Serial.println("\nWiFi connected");
  Serial.println(ip);
  Serial.print("Object detection stream link: http://esp32-cam.local/object_detection\n");
  setupServer();
  setupInference(width, height);
  
  xTaskCreatePinnedToCore(
    serverTask,   
    "ServerTask", 
    4096,        
    NULL,         
    1,            
    NULL,         
    1             
  );
  
  xTaskCreatePinnedToCore(
    inferenceTask,
    "InferenceTask",
    16384,  
    NULL,
    1,
    NULL,
    0  
  );
  
  Serial.println("System initialized");
}

void loop() {
  delay(1000); // watchdog timer will reset the ESP32 if loop is empty, this prevents unnecessary CPU usage
}