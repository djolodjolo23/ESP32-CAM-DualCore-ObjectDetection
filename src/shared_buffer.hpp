#ifndef SHARED_BUFFER_HPP
#define SHARED_BUFFER_HPP

#include <Arduino.h>
#include "esp_camera.h"
#include "FINAL_-_Object_Detection_128x128_ORANGE_inferencing.h" // use your own model here

typedef struct {
  camera_fb_t* frame;
  SemaphoreHandle_t mutex;
  bool hasNewFrame;
  ei_impulse_result_t lastResult;
  bool hasNewResult;
} SharedBuffer;

extern SharedBuffer sharedBuffer;

#endif // SHARED_BUFFER_HPP
