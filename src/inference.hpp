#ifndef INFERENCE_HPP
#define INFERENCE_HPP

#include <Arduino.h>
#include "esp_camera.h"
#include "OV2640.hpp"
#include "detected_objects.hpp"

#include "test_augmented_inferencing.h" // use your own model here
//#include "edge-impulse-sdk/dsp/image/image.hpp"

extern OV2640 cam;
extern SharedBuffer sharedBuffer;
extern std::vector<DetectedObject> detectedObjects;

const size_t inferenceWidth = EI_CLASSIFIER_INPUT_WIDTH;   
const size_t inferenceHeight = EI_CLASSIFIER_INPUT_HEIGHT; 

const size_t rgb_buffer_size = inferenceWidth * inferenceHeight * 3;

size_t m_fullWidth = 0;  
size_t m_fullHeight = 0;

uint8_t *rgb_buffer = nullptr;      
uint8_t *full_rgb_buffer = nullptr; 
void setupInference(size_t fullWidth, size_t fullHeight);
void inferenceTask(void *pvParameters);
int ei_camera_get_data(size_t offset, size_t length, float *out_ptr);
void cropAndResizeImage(uint8_t* src, int srcWidth, int srcHeight, 
                        uint8_t* dst, int dstWidth, int dstHeight);

void setupInference(size_t fullWidth, size_t fullHeight) {
  // Allocate inference buffer (actual train image size, ex 96x96 RGB)
  rgb_buffer = (uint8_t*)heap_caps_malloc(rgb_buffer_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!rgb_buffer) {
    Serial.println("Failed to allocate inference RGB buffer!");
    return;
  }

  const size_t full_rgb_buffer_size = fullWidth * fullHeight * 3;
  m_fullWidth = fullWidth;
  m_fullHeight = fullHeight;
  
  // Allocate full resolution buffer (ex 320x240 RGB)
  full_rgb_buffer = (uint8_t*)heap_caps_malloc(full_rgb_buffer_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!full_rgb_buffer) {
    Serial.println("Failed to allocate full resolution RGB buffer!");
    return;
  }
  
  Serial.println("Inference setup complete");
}

// Crop the source image to a centered square using the shortest axis,
// then resize that square to the destination dimensions (e.g. 96x96).
void cropAndResizeImage(uint8_t* src, int srcWidth, int srcHeight, 
                        uint8_t* dst, int dstWidth, int dstHeight) {
  // Determine the square crop dimensions (center crop)
  int cropSize = (srcWidth < srcHeight) ? srcWidth : srcHeight;
  int cropX = (srcWidth - cropSize) / 2;
  int cropY = (srcHeight - cropSize) / 2;
  
  // Nearest-neighbor scaling from the crop to the destination size.
  for (int y = 0; y < dstHeight; y++) {
    // Map the destination y to a source y within the crop
    int srcY = cropY + (y * cropSize) / dstHeight;
    for (int x = 0; x < dstWidth; x++) {
      // Map the destination x to a source x within the crop
      int srcX = cropX + (x * cropSize) / dstWidth;
      int srcIndex = (srcY * srcWidth + srcX) * 3;
      int dstIndex = (y * dstWidth + x) * 3;
      dst[dstIndex]     = src[srcIndex];       // Red
      dst[dstIndex + 1] = src[srcIndex + 1];   // Green
      dst[dstIndex + 2] = src[srcIndex + 2];   // Blue
    }
  }
}

// Function to supply data to the Edge Impulse SDK
int ei_camera_get_data(size_t offset, size_t length, float *out_ptr) {
  size_t pixel_ix = offset * 3;
  for (size_t i = 0; i < length; i++) {
    out_ptr[i] = (rgb_buffer[pixel_ix] << 16) | 
                 (rgb_buffer[pixel_ix + 1] << 8) | 
                  rgb_buffer[pixel_ix + 2];
    pixel_ix += 3;
  }
  return 0;
}


// The inference task: decode JPEG, crop & resize image, then run inference
void inferenceTask(void *pvParameters) {
    // Allow time for camera and system initialization
    vTaskDelay(pdMS_TO_TICKS(2000));
    Serial.println("Inference task started");

    while (true) {
        if (xSemaphoreTake(sharedBuffer.mutex, portMAX_DELAY) == pdTRUE) {
            if (sharedBuffer.hasNewFrame) {
                uint8_t* jpeg_buf = cam.getfb();
                int jpeg_len = cam.getSize();

                if (!jpeg_buf || jpeg_len <= 0) {
                    Serial.println("Failed to get a valid JPEG frame.");
                    xSemaphoreGive(sharedBuffer.mutex);
                    vTaskDelay(pdMS_TO_TICKS(100));
                    continue;
                }

                // need to copy the JPEG buffer since the camera will overwrite it, a lot of synchronization issues
                // probably better to use semaphore with good synchronization
                // but this also works
                uint8_t* jpeg_copy = (uint8_t*)heap_caps_malloc(jpeg_len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
                if (!jpeg_copy) {
                    Serial.println("Failed to allocate JPEG buffer");
                    cam.returnfb();  
                    xSemaphoreGive(sharedBuffer.mutex);
                    vTaskDelay(pdMS_TO_TICKS(100));
                    continue;
                }

                memcpy(jpeg_copy, jpeg_buf, jpeg_len);
                cam.returnfb(); 

                sharedBuffer.hasNewFrame = false;  // reset the flag
                xSemaphoreGive(sharedBuffer.mutex);

                bool converted = fmt2rgb888(jpeg_copy, jpeg_len, PIXFORMAT_JPEG, full_rgb_buffer);
                free(jpeg_copy);  

                if (converted) {
                    cropAndResizeImage(full_rgb_buffer, m_fullWidth, m_fullHeight,
                                       rgb_buffer, inferenceWidth, inferenceHeight);

                    ei::signal_t signal;
                    signal.total_length = inferenceWidth * inferenceHeight;
                    signal.get_data = &ei_camera_get_data;

                    ei_impulse_result_t result = {0};
                    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, false);

                    if (err == EI_IMPULSE_OK) {
                        if (xSemaphoreTake(sharedBuffer.mutex, portMAX_DELAY) == pdTRUE) {
                            memcpy(&sharedBuffer.lastResult, &result, sizeof(ei_impulse_result_t));
                            sharedBuffer.hasNewResult = true;

                            detectedObjects.clear();

                            Serial.println("Inference results:");
    #if EI_CLASSIFIER_OBJECT_DETECTION == 0
                            for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
                                Serial.printf("  %s: %.2f\n", result.classification[ix].label, result.classification[ix].value);
                            }
    #endif
    #if EI_CLASSIFIER_OBJECT_DETECTION == 1
                            for (size_t ix = 0; ix < result.bounding_boxes_count; ix++) {
                                auto bb = result.bounding_boxes[ix];
                                if (bb.value > 0.7) { // Only update if confidence is above a threshold
                                    DetectedObject obj;
                                    obj.label = String(bb.label);
                                    obj.x = bb.x;
                                    obj.y = bb.y;
                                    obj.width = bb.width;
                                    obj.height = bb.height;
                                    obj.conf = bb.value;
                                    detectedObjects.push_back(obj);
                                } 
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

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


#endif // INFERENCE_HPP
