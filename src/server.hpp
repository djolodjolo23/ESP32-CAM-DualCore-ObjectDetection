#ifndef SERVER_HPP
#define SERVER_HPP

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiClient.h>
#include "OV2640.hpp"
#include "detected_objects.hpp"
#include "test_inferencing.h" // use your own model here
#include "shared_buffer.hpp"


extern OV2640 cam;
SharedBuffer sharedBuffer;
extern std::vector<DetectedObject> detectedObjects;


const char HEADER[] = "HTTP/1.1 200 OK\r\n" \
                      "Access-Control-Allow-Origin: *\r\n" \
                      "Content-Type: multipart/x-mixed-replace; boundary=123456789000000000000987654321\r\n";
const char BOUNDARY[] = "\r\n--123456789000000000000987654321\r\n";
const char CTNTTYPE[] = "Content-Type: image/jpeg\r\nContent-Length: ";
const int hdrLen = strlen(HEADER);
const int bdrLen = strlen(BOUNDARY);
const int cntLen = strlen(CTNTTYPE);

WebServer server(80);

void setupServer();
void serverTask(void *pvParameters);
void handle_jpg_stream(void);
void handleNotFound(void);
void streamTask(void *pvParameters);
void handleCentroid(void);
void handleGetIP(void);

void setupServer() {
  server.on("/object_detection", HTTP_GET, handle_jpg_stream);
  server.on("/centroid", HTTP_GET, handleCentroid);
  server.on("/get_ip", HTTP_GET, handleGetIP);
  server.onNotFound(handleNotFound);
  server.begin();
}

void serverTask(void *pvParameters) {
  for (;;) {
    server.handleClient();
    vTaskDelay(1);
  }
}

void handle_jpg_stream(void) {
  WiFiClient client = server.client();
  WiFiClient *clientPtr = new WiFiClient(std::move(client));
  
  if (clientPtr == nullptr) {
    Serial.println("Client allocation failed");
    return;
  }
  
  xTaskCreatePinnedToCore(
    streamTask,
    "StreamTask",
    8192,
    clientPtr,
    1,
    NULL,
    1  // Run on Core 1
  );
}

void handleNotFound() {
  String message = "Server is running!\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  server.send(200, "text/plain", message);
}

void handleCentroid() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  if (detectedObjects.empty()) {
    server.send(200, "application/json", "[]"); 
    return;
  }

  String json = "[";
  for (size_t i = 0; i < detectedObjects.size(); i++) {
    const DetectedObject& obj = detectedObjects[i];
    if (i > 0) json += ",";
    json += "{";
    json += "\"label\":\"" + obj.label + "\",";
    json += "\"x\":" + String(obj.x) + ",";
    json += "\"y\":" + String(obj.y) + ",";
    json += "\"width\":" + String(obj.width) + ",";
    json += "\"height\":" + String(obj.height) + ",";
    json += "\"conf\":" + String(obj.conf);
    json += "}";
  }
  json += "]";
  server.send(200, "application/json", json);
}

void handleGetIP() {
  IPAddress ip = WiFi.localIP();
  String json = "{\"ip\":\"" + ip.toString() + "\"}";
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", json);
}


// Task that streams MJPEG to clients
void streamTask(void *pvParameters) {
  WiFiClient *clientPtr = (WiFiClient *)pvParameters;
  WiFiClient client = *clientPtr;
  delete clientPtr;
  
  client.write(HEADER, hdrLen);
  client.write(BOUNDARY, bdrLen);
  
  while (client.connected()) {
    cam.run();
    int jpeg_size = cam.getSize();
    uint8_t* jpeg_buf = cam.getfb();
    
    if (xSemaphoreTake(sharedBuffer.mutex, portMAX_DELAY) == pdTRUE) {
      client.write(CTNTTYPE, cntLen);
      char buf[32];
      sprintf(buf, "%d\r\n\r\n", jpeg_size);
      client.write(buf, strlen(buf));
      
      client.write((char *)jpeg_buf, jpeg_size);
      
      sharedBuffer.hasNewFrame = true;
      
      xSemaphoreGive(sharedBuffer.mutex);
    }
    
    client.write(BOUNDARY, bdrLen);
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
  
  client.stop();
  vTaskDelete(NULL);
}

#endif // SERVER_HPP