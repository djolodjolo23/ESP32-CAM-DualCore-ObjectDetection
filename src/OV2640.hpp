#ifndef OV2640_HPP
#define OV2640_HPP

#include <Arduino.h>
#include <pgmspace.h>
#include <stdio.h>
#include "esp_log.h"
#include "esp_attr.h"
#include "esp_camera.h"

#define TAG "OV2640"

// Camera Configurations for Different ESP32-CAM Boards
extern camera_config_t esp32cam_config;
extern camera_config_t esp32cam_aithinker_config;
extern camera_config_t esp32cam_ttgo_t_config;

class OV2640 {
public:
    OV2640() { fb = NULL; }
    ~OV2640() {}

    esp_err_t init(camera_config_t config);
    void run();
    size_t getSize();
    uint8_t *getfb();
    int getWidth();
    int getHeight();
    framesize_t getFrameSize();
    pixformat_t getPixelFormat();
    void setFrameSize(framesize_t size);
    void setPixelFormat(pixformat_t format);
    void returnfb();
    void flip(bool vertical, bool horizontal);

private:
    void runIfNeeded(); // Grab a frame if we don't already have one
    camera_config_t _cam_config;
    camera_fb_t *fb;
};

// Camera Configurations
camera_config_t esp32cam_config{
    .pin_pwdn = -1, .pin_reset = 15, .pin_xclk = 27,
    .pin_sscb_sda = 25, .pin_sscb_scl = 23,
    .pin_d7 = 19, .pin_d6 = 36, .pin_d5 = 18, .pin_d4 = 39,
    .pin_d3 = 5, .pin_d2 = 34, .pin_d1 = 35, .pin_d0 = 17,
    .pin_vsync = 22, .pin_href = 26, .pin_pclk = 21,
    .xclk_freq_hz = 20000000, .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0, .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_SVGA, .jpeg_quality = 12, .fb_count = 2
};

camera_config_t esp32cam_aithinker_config{
    .pin_pwdn = 32, .pin_reset = -1, .pin_xclk = 0,
    .pin_sscb_sda = 26, .pin_sscb_scl = 27,
    .pin_d7 = 35, .pin_d6 = 34, .pin_d5 = 39, .pin_d4 = 36,
    .pin_d3 = 21, .pin_d2 = 19, .pin_d1 = 18, .pin_d0 = 5,
    .pin_vsync = 25, .pin_href = 23, .pin_pclk = 22,
    .xclk_freq_hz = 20000000, .ledc_timer = LEDC_TIMER_1,
    .ledc_channel = LEDC_CHANNEL_1, .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_SVGA, .jpeg_quality = 12, .fb_count = 2
};

camera_config_t esp32cam_ttgo_t_config{
    .pin_pwdn = 26, .pin_reset = -1, .pin_xclk = 32,
    .pin_sscb_sda = 13, .pin_sscb_scl = 12,
    .pin_d7 = 39, .pin_d6 = 36, .pin_d5 = 23, .pin_d4 = 18,
    .pin_d3 = 15, .pin_d2 = 4, .pin_d1 = 14, .pin_d0 = 5,
    .pin_vsync = 27, .pin_href = 25, .pin_pclk = 19,
    .xclk_freq_hz = 20000000, .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0, .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_SVGA, .jpeg_quality = 12, .fb_count = 2
};

// OV2640 Implementation

esp_err_t OV2640::init(camera_config_t config) {
    memset(&_cam_config, 0, sizeof(_cam_config));
    memcpy(&_cam_config, &config, sizeof(config));

    esp_err_t err = esp_camera_init(&_cam_config);
    if (err != ESP_OK) {
        printf("Camera probe failed with error 0x%x", err);
        return err;
    }
    return ESP_OK;
}

void OV2640::run() {
    if (fb) esp_camera_fb_return(fb);
    fb = esp_camera_fb_get();
}

void OV2640::runIfNeeded() {
    if (!fb) run();
}

int OV2640::getWidth() {
    runIfNeeded();
    return fb ? fb->width : 0;
}

int OV2640::getHeight() {
    runIfNeeded();
    return fb ? fb->height : 0;
}

size_t OV2640::getSize() {
    runIfNeeded();
    return fb ? fb->len : 0;
}

uint8_t* OV2640::getfb() {
    runIfNeeded();
    return fb ? fb->buf : NULL;
}

framesize_t OV2640::getFrameSize() {
    return _cam_config.frame_size;
}

void OV2640::setFrameSize(framesize_t size) {
    _cam_config.frame_size = size;
}

void OV2640::returnfb() {
    if (fb) {
        esp_camera_fb_return(fb);
        fb = NULL;
    }
}

pixformat_t OV2640::getPixelFormat() {
    return _cam_config.pixel_format;
}

void OV2640::setPixelFormat(pixformat_t format) {
    switch (format) {
        case PIXFORMAT_RGB565:
        case PIXFORMAT_YUV422:
        case PIXFORMAT_GRAYSCALE:
        case PIXFORMAT_JPEG:
            _cam_config.pixel_format = format;
            break;
        default:
            _cam_config.pixel_format = PIXFORMAT_GRAYSCALE;
            break;
    }
}

void OV2640::flip(bool vertical, bool horizontal) {
    sensor_t *s = esp_camera_sensor_get();
    s->set_vflip(s, vertical);
    s->set_hmirror(s, horizontal);
}

#endif // OV2640_HPP
