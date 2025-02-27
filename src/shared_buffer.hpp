#ifndef SHARED_BUFFER_HPP
#define SHARED_BUFFER_HPP

#include <Arduino.h>
#include "esp_camera.h"
#include "esp32-cam-banana-test_inferencing.h"

typedef struct {
  camera_fb_t* frame;
  SemaphoreHandle_t mutex;
  bool hasNewFrame;
  ei_impulse_result_t lastResult;
  bool hasNewResult;
} SharedBuffer;

extern SharedBuffer sharedBuffer;

#endif // SHARED_BUFFER_HPP
