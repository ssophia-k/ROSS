// camera.cpp — isolated translation unit for esp_camera.h
// Do NOT include any Adafruit headers here (sensor_t conflict).

#include "camera.h"

#include <Arduino.h>
#include "esp_camera.h"

// AI-Thinker ESP32-CAM pin definitions
#define PWDN_GPIO_NUM   32
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM    0
#define SIOD_GPIO_NUM   26
#define SIOC_GPIO_NUM   27
#define Y9_GPIO_NUM     35
#define Y8_GPIO_NUM     34
#define Y7_GPIO_NUM     39
#define Y6_GPIO_NUM     36
#define Y5_GPIO_NUM     21
#define Y4_GPIO_NUM     19
#define Y3_GPIO_NUM     18
#define Y2_GPIO_NUM      5
#define VSYNC_GPIO_NUM  25
#define HREF_GPIO_NUM   23
#define PCLK_GPIO_NUM   22

bool camera_init() {
    // Power-cycle the camera to ensure clean SCCB detection
    pinMode(PWDN_GPIO_NUM, OUTPUT);
    digitalWrite(PWDN_GPIO_NUM, HIGH);  // Power down
    delay(100);
    digitalWrite(PWDN_GPIO_NUM, LOW);   // Power up
    delay(100);

    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_4;  // Channels 0-3 used by motors
    config.ledc_timer   = LEDC_TIMER_1;
    config.pin_d0       = Y2_GPIO_NUM;
    config.pin_d1       = Y3_GPIO_NUM;
    config.pin_d2       = Y4_GPIO_NUM;
    config.pin_d3       = Y5_GPIO_NUM;
    config.pin_d4       = Y6_GPIO_NUM;
    config.pin_d5       = Y7_GPIO_NUM;
    config.pin_d6       = Y8_GPIO_NUM;
    config.pin_d7       = Y9_GPIO_NUM;
    config.pin_xclk     = XCLK_GPIO_NUM;
    config.pin_pclk     = PCLK_GPIO_NUM;
    config.pin_vsync    = VSYNC_GPIO_NUM;
    config.pin_href     = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn     = PWDN_GPIO_NUM;
    config.pin_reset    = RESET_GPIO_NUM;
    config.xclk_freq_hz = 10000000;  // 10 MHz — more reliable SCCB detection
    config.pixel_format = PIXFORMAT_JPEG;
    config.grab_mode    = CAMERA_GRAB_LATEST;

    // Use PSRAM for higher resolution
    if (psramFound()) {
        config.frame_size   = FRAMESIZE_VGA;   // 640x480
        config.jpeg_quality = 12;               // 0-63, lower = better
        config.fb_count     = 2;
    } else {
        config.frame_size   = FRAMESIZE_QVGA;  // 320x240
        config.jpeg_quality = 15;
        config.fb_count     = 1;
    }

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("[CAM] Init failed: 0x%x\n", err);
        return false;
    }
    return true;
}

camera_frame camera_capture() {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        return {nullptr, 0, nullptr};
    }
    return {fb->buf, fb->len, fb};
}

void camera_release(camera_frame &frame) {
    if (frame._fb) {
        esp_camera_fb_return(static_cast<camera_fb_t *>(frame._fb));
        frame._fb = nullptr;
    }
}
