

#include <Arduino.h>
/*

  This is a simple MJPEG streaming webserver implemented for AI-Thinker ESP32-CAM and
  ESP32-EYE modules.
  This is tested to work with VLC and Blynk video widget.

  Inspired by and based on this Instructable: $9 RTSP Video Streamer Using the ESP32-CAM Board
  (https://www.instructables.com/id/9-RTSP-Video-Streamer-Using-the-ESP32-CAM-Board/)

  Board: AI-Thinker ESP32-CAM

*/

#include "OV2640.h"
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiClient.h>

// Select camera model
//#define CAMERA_MODEL_WROVER_KIT
//#define CAMERA_MODEL_ESP_EYE
//#define CAMERA_MODEL_M5STACK_PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE
#define CAMERA_MODEL_AI_THINKER

#include "camera_pins.h"

#include "home_wifi_multi.h"

OV2640 cam;

WebServer server(80);

const char HEADER[] = "HTTP/1.1 200 OK\r\n" \
                      "Access-Control-Allow-Origin: *\r\n" \
                      "Content-Type: multipart/x-mixed-replace; boundary=123456789000000000000987654321\r\n";
const char BOUNDARY[] = "\r\n--123456789000000000000987654321\r\n";
const char CTNTTYPE[] = "Content-Type: image/jpeg\r\nContent-Length: ";
const int hdrLen = strlen(HEADER);
const int bdrLen = strlen(BOUNDARY);
const int cntLen = strlen(CTNTTYPE);


void streamTask(void *pvParameters) {
  WiFiClient *clientPtr = (WiFiClient *)pvParameters;
  WiFiClient client = *clientPtr;
  free(clientPtr);

  client.write(HEADER, hdrLen);
  client.write(BOUNDARY, bdrLen);

  while (client.connected()) {
    cam.run();
    int s = cam.getSize();
    client.write(CTNTTYPE, cntLen);
    char buf[32];
    sprintf(buf, "%d\r\n\r\n", s);
    client.write(buf, strlen(buf));
    client.write((char *)cam.getfb(), s);
    client.write(BOUNDARY, bdrLen);
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }

  client.stop();
  vTaskDelete(NULL);

}

void printTask(void *pvParameters) {
  while (true) {
    Serial.println("Hello from FreeRTOS!");
    vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay for 1 second
  }
}

void handle_jpg_stream(void)
{
  // Get the client from the server.
  WiFiClient client = server.client();
  
  // Use move semantics to transfer ownership of the client.
  WiFiClient *clientPtr = new WiFiClient(std::move(client));
  if (clientPtr == nullptr) {
    Serial.println("Allocation failed");
    return;
  }
  
  // Create the streaming task pinned to core 1.
  xTaskCreatePinnedToCore(
      streamTask,    // Task function.
      "streamTask",  // Task name.
      8192,          // Stack size.
      clientPtr,     // Parameter to pass (moved client pointer).
      1,             // Task priority.
      NULL,          // Task handle.
      1              // Pin to core 1.
  );
}

void handleNotFound()
{
  String message = "Server is running!\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  server.send(200, "text / plain", message);
}

void setup()
{

  Serial.begin(9600);
  //while (!Serial);            //wait for serial connection.

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

  // Frame parameters
  //  config.frame_size = FRAMESIZE_UXGA;
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 12;
  config.fb_count = 2;

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  cam.init(config);

  IPAddress ip;

  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID1, PWD1);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(F("."));
  }
  ip = WiFi.localIP();
  Serial.println(F("WiFi connected"));
  Serial.println("");
  Serial.println(ip);
  Serial.print("Stream Link: http://");
  Serial.print(ip);
  Serial.println("/mjpeg/1");
  server.on("/mjpeg/1", HTTP_GET, handle_jpg_stream);
  server.onNotFound(handleNotFound);
  server.begin();

  xTaskCreatePinnedToCore(
      printTask,    // Task function.
      "printTask",  // Task name.
      2048,         // Stack size.
      NULL,         // Parameter to pass.
      1,            // Task priority.
      NULL,         // Task handle.
      0             // Pin to core 0.
  );
}

void loop()
{
  server.handleClient();
}