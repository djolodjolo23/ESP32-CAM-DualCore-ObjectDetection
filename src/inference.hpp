#ifndef INFERENCE_HPP
#define INFERENCE_HPP

#include <Arduino.h>
#include "esp_camera.h"
#include "OV2640.hpp"
#include "detected_objects.hpp"

#include "Object_Detection_with_Image_Transformation_-_Orange_inferencing.h" // use your own model here

extern OV2640 cam;
extern SharedBuffer sharedBuffer;
extern std::vector<DetectedObject> detectedObjects;

const size_t inferenceWidth = EI_CLASSIFIER_INPUT_WIDTH;   
const size_t inferenceHeight = EI_CLASSIFIER_INPUT_HEIGHT; 

const size_t rgb_buffer_size = EI_CLASSIFIER_NN_INPUT_FRAME_SIZE;

size_t m_fullWidth = 0;  
size_t m_fullHeight = 0;

int m_cropWidth = 0;
int m_cropHeight = 0;
int m_offsetX = 0;
int m_offsetY = 0;


uint8_t *rgb_buffer = nullptr;      
uint8_t *full_rgb_buffer = nullptr; 
void setupInference(size_t fullWidth, size_t fullHeight);
void inferenceTask(void *pvParameters);
int ei_camera_get_data(size_t offset, size_t length, float *out_ptr);

void setupInference(size_t fullWidth, size_t fullHeight) {
  rgb_buffer = (uint8_t*)heap_caps_malloc(rgb_buffer_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!rgb_buffer) {
    Serial.println("Failed to allocate inference RGB buffer!");
    return;
  }

  const size_t full_rgb_buffer_size = fullWidth * fullHeight * 3;
  
  m_fullWidth = fullWidth;
  m_fullHeight = fullHeight;
  
  full_rgb_buffer = (uint8_t*)heap_caps_malloc(full_rgb_buffer_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!full_rgb_buffer) {
    Serial.println("Failed to allocate full resolution RGB buffer!");
    return;
  }
  
  Serial.println("Inference setup complete");
}

// Resize the image while preserving aspect ratio with center cropping
void resizeImage(uint8_t* src, int srcWidth, int srcHeight, 
  uint8_t* dst, int dstWidth, int dstHeight) {
  // Determine the crop dimensions to maintain aspect ratio

  float srcAspect = (float)srcWidth / srcHeight;
  float dstAspect = (float)dstWidth / dstHeight;


  if (srcAspect > dstAspect) {
    // Source is wider - crop width
    m_cropHeight = srcHeight;
    m_cropWidth = srcHeight * dstAspect;
    m_offsetX = (srcWidth - m_cropWidth) / 2;
  } else {
    // Source is taller - crop height
    m_cropWidth = srcWidth;
    m_cropHeight = srcWidth / dstAspect;
    m_offsetY = (srcHeight - m_cropHeight) / 2;
  }

  // Resize with bilinear interpolation for better quality
  for (int y = 0; y < dstHeight; y++) {
    for (int x = 0; x < dstWidth; x++) {
    // Map destination coordinates to source cropped area
      float srcX = m_offsetX + (float)x * m_cropWidth / dstWidth;
      float srcY = m_offsetY + (float)y * m_cropHeight / dstHeight;

      int x0 = (int)srcX;
      int y0 = (int)srcY;
      int x1 = min(x0 + 1, srcWidth - 1);
      int y1 = min(y0 + 1, srcHeight - 1);
      float wx = srcX - x0;
      float wy = srcY - y0;

        // For each color channel
      for (int c = 0; c < 3; c++) {
      // Get the four surrounding pixels
        uint8_t p00 = src[(y0 * srcWidth + x0) * 3 + c];
        uint8_t p01 = src[(y0 * srcWidth + x1) * 3 + c];
        uint8_t p10 = src[(y1 * srcWidth + x0) * 3 + c];
        uint8_t p11 = src[(y1 * srcWidth + x1) * 3 + c];

        // Bilinear interpolation
        float top = p00 * (1 - wx) + p01 * wx;
        float bottom = p10 * (1 - wx) + p11 * wx;
        uint8_t pixel = (uint8_t)(top * (1 - wy) + bottom * wy);

        dst[(y * dstWidth + x) * 3 + c] = pixel;
      }
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


void inferenceTask(void *pvParameters) {
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

                bool converted = fmt2rgb888(jpeg_copy, jpeg_len, PIXFORMAT_JPEG, full_rgb_buffer); // not sure if this is needed
                free(jpeg_copy);  

                // if (converted) {
                    resizeImage(full_rgb_buffer, m_fullWidth, m_fullHeight, rgb_buffer, inferenceWidth, inferenceHeight);

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
                                if (bb.value > 0.7) {
                                  DetectedObject obj;
                                  obj.label = String(bb.label);
                                  
                                  // Use the stored crop dimensions
                                  float xScale = (float)m_cropWidth / inferenceWidth;
                                  float yScale = (float)m_cropHeight / inferenceHeight;

                                  Serial.printf("Crop: %d x %d\n", m_cropWidth, m_cropHeight);

                                  Serial.printf("xScale: %.2f, yScale: %.2f\n", xScale, yScale);
                                  
                                  // Map back to original image coordinates
                                  obj.x = bb.x * xScale + m_offsetX;
                                  obj.y = bb.y * yScale + m_offsetY;
                                  obj.width = bb.width * xScale;
                                  obj.height = bb.height * yScale;
                                  
                                  obj.conf = bb.value;
                                  Serial.printf("  %s (%.2f) [ x: %u, y: %u, width: %u, height: %u ]\n",
                                                obj.label.c_str(), obj.conf, obj.x, obj.y, obj.width, obj.height);
                                  Serial.printf("  %s (%.2f) [ x: %u, y: %u, width: %u, height: %u ]\n",
                                                bb.label, bb.value, bb.x, bb.y, bb.width, bb.height);
                                  detectedObjects.push_back(obj);
                                }
                                if (bb.value == 0) continue;
                                //Serial.printf("  %s (%.2f) [ x: %u, y: %u, width: %u, height: %u ]\n",
                               //               bb.label, bb.value, bb.x, bb.y, bb.width, bb.height);
                            }
    #endif
                            xSemaphoreGive(sharedBuffer.mutex);
                        }
                    } else {
                        Serial.printf("Inference failed with error: %d\n", err);
                    }
                // } else {
                //     Serial.println("JPEG to RGB conversion failed");
                // }
            } else {
                xSemaphoreGive(sharedBuffer.mutex);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


#endif // INFERENCE_HPP
