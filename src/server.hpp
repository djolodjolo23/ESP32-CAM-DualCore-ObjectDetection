#include <Arduino.h>
#include "OV2640.h"
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiClient.h>

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
  
  xTaskCreatePinnedToCore(
      streamTask,   
      "streamTask",  
      8192,          
      clientPtr,     
      1,             
      NULL,          
      1             
  );
}

void serverTask(void *pvParameters)
{
  for (;;) {
    server.handleClient();
    vTaskDelay(1);
  }
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