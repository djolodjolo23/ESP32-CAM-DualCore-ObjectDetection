#include <Arduino.h>
#include "OV2640.h"
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiClient.h>

#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"
#include "home_wifi_multi.h"
#include "server.hpp"
#include "inference.hpp"

// Camera instance
OV2640 cam;

// Shared buffer for coordination between streaming and inference
SharedBuffer sharedBuffer;

//coordinates for centroid
float x, y;

void setup() {
  Serial.begin(115200);
  Serial.println("Starting ESP32-CAM with inference...");
  
  // Initialize shared buffer
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
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_QVGA;  // 320x240 - match Edge Impulse model input size
  config.jpeg_quality = 12;
  config.fb_count = 2;
  
  // Initialize camera
  cam.init(config);
  
  // Connect to WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID1, PWD1);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  // Print connection details
  IPAddress ip = WiFi.localIP();
  Serial.println("\nWiFi connected");
  Serial.println(ip);
  Serial.print("Stream Link: http://");
  Serial.print(ip);
  Serial.println("/mjpeg/1");
  
  // Initialize server
  setupServer();
  
  // Initialize inference
  setupInference();
  
  // Create server task on Core 1
  xTaskCreatePinnedToCore(
    serverTask,   
    "ServerTask", 
    4096,        
    NULL,         
    1,            
    NULL,         
    1             
  );
  
  // Create inference task on Core 0
  xTaskCreatePinnedToCore(
    inferenceTask,
    "InferenceTask",
    8192,  // Increased stack size for inference
    NULL,
    1,
    NULL,
    0  // Run on Core 0
  );
  
  Serial.println("System initialized");
}

void loop() {
  // Nothing to do here as everything is handled by tasks
  //delay(1000);
}