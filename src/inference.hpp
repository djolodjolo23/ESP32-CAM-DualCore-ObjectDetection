#ifndef INFERENCE_HPP
#define INFERENCE_HPP

#include <Arduino.h>
#include "esp_camera.h"
#include "OV2640.h"

#include <esp32-cam-banana-test_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"

extern OV2640 cam;
extern SharedBuffer sharedBuffer;

// Inference input: 96x96 RGB
const size_t inferenceWidth = EI_CLASSIFIER_INPUT_WIDTH;
const size_t inferenceHeight = EI_CLASSIFIER_INPUT_HEIGHT;
const size_t rgb_buffer_size = inferenceWidth * inferenceHeight * 3;

// Full resolution (QVGA): 320x240 RGB
const size_t fullWidth = 320;
const size_t fullHeight = 240;
const size_t full_rgb_buffer_size = fullWidth * fullHeight * 3;

uint8_t *rgb_buffer = nullptr;      
uint8_t *full_rgb_buffer = nullptr; 


void setupInference();
void inferenceTask(void *pvParameters);
int ei_camera_get_data(size_t offset, size_t length, float *out_ptr);
void resizeImage(uint8_t* src, int srcWidth, int srcHeight, uint8_t* dst, int dstWidth, int dstHeight);


void setupInference() {
    rgb_buffer = (uint8_t*)heap_caps_malloc(rgb_buffer_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!rgb_buffer) {
      Serial.println("Failed to allocate inference RGB buffer!");
      return;
    }
    
    // Allocate buffer for full resolution image (320x240 RGB)
    full_rgb_buffer = (uint8_t*)heap_caps_malloc(full_rgb_buffer_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!full_rgb_buffer) {
      Serial.println("Failed to allocate full resolution RGB buffer!");
      return;
    }
    
    Serial.println("Inference setup complete");
}


void resizeImage(uint8_t* src, int srcWidth, int srcHeight, uint8_t* dst, int dstWidth, int dstHeight) {
    for (int y = 0; y < dstHeight; y++) {
      int srcY = y * srcHeight / dstHeight;
      for (int x = 0; x < dstWidth; x++) {
        int srcX = x * srcWidth / dstWidth;
        int srcIndex = (srcY * srcWidth + srcX) * 3;
        int dstIndex = (y * dstWidth + x) * 3;
        dst[dstIndex]     = src[srcIndex];       // Red
        dst[dstIndex + 1] = src[srcIndex + 1];   // Green
        dst[dstIndex + 2] = src[srcIndex + 2];   // Blue
      }
    }
}


int ei_camera_get_data(size_t offset, size_t length, float *out_ptr) {
    size_t pixel_ix = offset * 3;
    for (size_t i = 0; i < length; i++) {
      out_ptr[i] = (rgb_buffer[pixel_ix] << 16) + 
                   (rgb_buffer[pixel_ix + 1] << 8) + 
                   rgb_buffer[pixel_ix + 2];
      pixel_ix += 3;
    }
    return 0;
}


void inferenceTask(void *pvParameters) {
    vTaskDelay(pdMS_TO_TICKS(2000));
    Serial.println("Inference task started");
    
    while (true) {
      if (xSemaphoreTake(sharedBuffer.mutex, portMAX_DELAY) == pdTRUE) {
        if (sharedBuffer.hasNewFrame) {
          uint8_t* jpeg_buf = cam.getfb();
          int jpeg_len = cam.getSize();
          
          bool converted = fmt2rgb888(jpeg_buf, jpeg_len, PIXFORMAT_JPEG, full_rgb_buffer);
          
          sharedBuffer.hasNewFrame = false;
          xSemaphoreGive(sharedBuffer.mutex);
          
          if (converted) {
            resizeImage(full_rgb_buffer, fullWidth, fullHeight, rgb_buffer, inferenceWidth, inferenceHeight);
            
            ei::signal_t signal;
            signal.total_length = inferenceWidth * inferenceHeight;
            signal.get_data = &ei_camera_get_data;
            
            ei_impulse_result_t result = { 0 };
            EI_IMPULSE_ERROR err = run_classifier(&signal, &result, false);
            
            if (err == EI_IMPULSE_OK) {
              if (xSemaphoreTake(sharedBuffer.mutex, portMAX_DELAY) == pdTRUE) {
                memcpy(&sharedBuffer.lastResult, &result, sizeof(ei_impulse_result_t));
                sharedBuffer.hasNewResult = true;
                
                Serial.println("Inference results:");
    #if EI_CLASSIFIER_OBJECT_DETECTION == 0
                for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
                  Serial.printf("  %s: %.2f\n", result.classification[ix].label, result.classification[ix].value);
                }
    #endif
    #if EI_CLASSIFIER_OBJECT_DETECTION == 1
                for (size_t ix = 0; ix < result.bounding_boxes_count; ix++) {
                  auto bb = result.bounding_boxes[ix];
                  if (bb.value == 0) continue;
                  Serial.printf("  %s (%.2f) [ x: %u, y: %u, width: %u, height: %u ]\n", 
                                bb.label, bb.value, bb.x, bb.y, bb.width, bb.height);
                }
    #endif
                xSemaphoreGive(sharedBuffer.mutex);
              }
            } else {
              Serial.printf("Inference failed with error: %d\n", err);
            }
          } else {
            Serial.println("JPEG to RGB conversion failed");
          }
        } else {
          xSemaphoreGive(sharedBuffer.mutex);
        }
      }
      
      // Delay to prevent overloading the CPU
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }
  
  #endif // INFERENCE_HPP