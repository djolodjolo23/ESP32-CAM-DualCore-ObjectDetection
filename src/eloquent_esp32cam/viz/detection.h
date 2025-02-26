// detection.h
#ifndef DETECTION_H
#define DETECTION_H

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"



struct DetectedObject {
  int x;
  int y;
  int width;
  int height;
  float proba;
  String label;
  bool valid;
};

extern DetectedObject lastDetected;

extern SemaphoreHandle_t detectionMutex;

extern camera_fb_t* latestFrame;

#endif