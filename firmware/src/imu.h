#pragma once

// LSM6DS3TR-C IMU over I2C (GPIO 2 = SDA, GPIO 3 = SCL).
// Isolated translation unit — Adafruit_Sensor.h defines sensor_t
// which conflicts with esp_camera.h (same pattern as camera.h/.cpp).

struct imu_data {
    float accel_x, accel_y, accel_z;  // m/s²
    float gyro_x, gyro_y, gyro_z;    // rad/s
    float temp;                        // °C
    bool valid;
};

bool imu_init();
imu_data imu_read();
