#include <Arduino.h>

void printTask(void *pvParameters) {
    while (true) {
      Serial.println("Hello from FreeRTOS!");
      vTaskDelay(1000 / portTICK_PERIOD_MS); 
    }
  }