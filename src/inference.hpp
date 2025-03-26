#ifndef INFERENCE_HPP
#define INFERENCE_HPP

#include <Arduino.h>
#include "esp_camera.h"
#include "OV2640.hpp"
#include "detected_objects.hpp"
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "shared_buffer.hpp"

#include "FINAL_-_Object_Detection_128x128_ORANGE_inferencing.h" // use your own model here

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

int64_t resize_time = 0;
int64_t decode_time = 0;
int64_t inference_time = 0;

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

  float srcAspect = (float)fullWidth / fullHeight;
  float dstAspect = (float)inferenceWidth / inferenceHeight;

  if (srcAspect > dstAspect) {
    m_cropHeight = fullHeight;
    m_cropWidth = fullHeight * dstAspect;
    m_offsetX = (fullWidth - m_cropWidth) / 2;
    m_offsetY = 0;
  } else {
    m_cropWidth = fullWidth;
    m_cropHeight = fullWidth / dstAspect;
    m_offsetX = 0;
    m_offsetY = (fullHeight - m_cropHeight) / 2;
  }

  Serial.printf("Crop dimensions: w=%d, h=%d, offsetX=%d, offsetY=%d\n", 
    m_cropWidth, m_cropHeight, m_offsetX, m_offsetY);
  Serial.println("Optimized inference setup complete");
}



// Function to supply data to the Edge Impulse SDK
int ei_camera_get_data(size_t offset, size_t length, float *out_ptr) {
    // we already have a RGB888 buffer, so recalculate offset into pixel index
    size_t pixel_ix = offset * 3;
    size_t pixels_left = length;
    size_t out_ptr_ix = 0;

    while (pixels_left != 0) {
        out_ptr[out_ptr_ix] = (rgb_buffer[pixel_ix] << 16) + (rgb_buffer[pixel_ix + 1] << 8) + rgb_buffer[pixel_ix + 2];

        // go to the next pixel
        out_ptr_ix++;
        pixel_ix+=3;
        pixels_left--;
    }
    // and done!
    return 0;
}


void inferenceTask(void *pvParameters) {
  vTaskDelay(pdMS_TO_TICKS(2000));
  Serial.println("Optimized inference task started");

  while (true) {
      if (xSemaphoreTake(sharedBuffer.mutex, portMAX_DELAY) == pdTRUE) {
          if (sharedBuffer.hasNewFrame) {
              uint8_t* jpeg_buf = cam.getfb();
              int jpeg_len = cam.getSize();

              if (!jpeg_buf || jpeg_len <= 0) {
                  Serial.println("Failed to get a valid JPEG frame.");
                  cam.returnfb();
                  sharedBuffer.hasNewFrame = false;
                  xSemaphoreGive(sharedBuffer.mutex);
                  vTaskDelay(pdMS_TO_TICKS(100));
                  continue;
              }

              static uint8_t* jpeg_copy = nullptr;
              static int last_jpeg_size = 0;
              
              if (!jpeg_copy || last_jpeg_size < jpeg_len) {
                  if (jpeg_copy) {
                      heap_caps_free(jpeg_copy);
                  }
                  jpeg_copy = (uint8_t*)heap_caps_malloc(jpeg_len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
                  last_jpeg_size = jpeg_len;
                  
                  if (!jpeg_copy) {
                      Serial.println("Failed to allocate JPEG buffer");
                      cam.returnfb();
                      sharedBuffer.hasNewFrame = false;
                      xSemaphoreGive(sharedBuffer.mutex);
                      vTaskDelay(pdMS_TO_TICKS(100));
                      continue;
                  }
              }

              memcpy(jpeg_copy, jpeg_buf, jpeg_len);
              cam.returnfb();
              sharedBuffer.hasNewFrame = false;
              xSemaphoreGive(sharedBuffer.mutex);

              int64_t start_time = esp_timer_get_time();
              bool converted = fmt2rgb888(jpeg_copy, jpeg_len, PIXFORMAT_JPEG, full_rgb_buffer);
              decode_time = esp_timer_get_time() - start_time;
              
              if (converted) {
                  start_time = esp_timer_get_time();
                  //fastResizeImageNearestNeighbor(full_rgb_buffer, rgb_buffer);
                  ei::image::processing::crop_and_interpolate_rgb888(
                    full_rgb_buffer,           // source buffer
                    m_fullWidth,               // source width
                    m_fullHeight,              // source height
                    rgb_buffer,                // destination buffer
                    inferenceWidth,            // destination width
                    inferenceHeight            // destination height
                  );
                  
                  resize_time = esp_timer_get_time() - start_time;
                  
                  start_time = esp_timer_get_time();
                  ei::signal_t signal;
                  signal.total_length = inferenceWidth * inferenceHeight;
                  signal.get_data = &ei_camera_get_data;

                  ei_impulse_result_t result = {0};
                  EI_IMPULSE_ERROR err = run_classifier(&signal, &result, false);
                  inference_time = esp_timer_get_time() - start_time;
                  
                  static int frame_count = 0;
                  if (++frame_count % 3 == 0) {
                      Serial.printf("Performance: Decode=%.2f ms, Resize=%.2f ms, Inference=%.2f ms, Total=%.2f ms\n",
                                    decode_time / 1000.0, resize_time / 1000.0, inference_time / 1000.0,
                                    (decode_time + resize_time + inference_time) / 1000.0);
                  }

                  if (err == EI_IMPULSE_OK) {
                      if (xSemaphoreTake(sharedBuffer.mutex, portMAX_DELAY) == pdTRUE) {
                          memcpy(&sharedBuffer.lastResult, &result, sizeof(ei_impulse_result_t));
                          sharedBuffer.hasNewResult = true;
                          detectedObjects.clear();

#if EI_CLASSIFIER_OBJECT_DETECTION == 0
                          for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
                              Serial.printf("  %s: %.2f\n", result.classification[ix].label, result.classification[ix].value);
                          }
#endif
#if EI_CLASSIFIER_OBJECT_DETECTION == 1
                          for (size_t ix = 0; ix < result.bounding_boxes_count; ix++) {
                            Serial.printf("  %s: %.2f at (%d, %d) size %dx%d\n", result.bounding_boxes[ix].label, result.bounding_boxes[ix].value, result.bounding_boxes[ix].x, result.bounding_boxes[ix].y, result.bounding_boxes[ix].width, result.bounding_boxes[ix].height);
                              auto bb = result.bounding_boxes[ix];
                              if (bb.value > 0.5) {
                                  DetectedObject obj;
                                  obj.label = String(bb.label);
                                
                                  float xScale = (float)m_cropWidth / inferenceWidth;
                                  float yScale = (float)m_cropHeight / inferenceHeight;

                                  obj.x = bb.x * xScale + m_offsetX;
                                  obj.y = bb.y * yScale + m_offsetY;
                                  obj.width = bb.width * xScale;
                                  obj.height = bb.height * yScale;
                                  //Serial.printf("Object: %s at (%d, %d) size %dx%d\n", obj.label.c_str(), obj.x, obj.y, obj.width, obj.height);
                                
                                  obj.conf = bb.value;
                                  detectedObjects.push_back(obj);
                              }
                              //Serial.printf("  %s: %.2f at (%d, %d) size %dx%d\n", bb.label, bb.value, bb.x, bb.y, bb.width, bb.height);
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

      vTaskDelay(pdMS_TO_TICKS(50));
  }
}



#endif // INFERENCE_HPP
