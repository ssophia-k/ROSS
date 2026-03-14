#pragma once

#include <cstddef>
#include <cstdint>

// Opaque wrapper — keeps esp_camera.h out of translation units that
// include Adafruit_Sensor.h (both define sensor_t).

bool camera_init();

struct camera_frame {
    const uint8_t *buf;
    size_t len;
    void *_fb;  // opaque handle for return
};

camera_frame camera_capture();
void camera_release(camera_frame &frame);
