#include "imu.h"

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LSM6DS3TRC.h>

#include "config.h"

static Adafruit_LSM6DS3TRC lsm;
static bool initialized = false;

bool imu_init() {
    Wire.begin(PIN_IMU_SDA, PIN_IMU_SCL);
    delay(50);  // let I2C bus settle after pin reassignment

    // Retry — GPIO 3 transitions from UART RX to SCL at boot
    for (int attempt = 0; attempt < 3; attempt++) {
        if (lsm.begin_I2C(LSM6DS_I2CADDR_DEFAULT, &Wire)) {
            goto success;
        }
        Serial.printf("[IMU] Init attempt %d failed, retrying...\n", attempt + 1);
        delay(100);
    }
    Serial.println("[IMU] LSM6DS3TR-C not found after 3 attempts");
    return false;

success:
    lsm.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
    lsm.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
    lsm.setAccelDataRate(LSM6DS_RATE_104_HZ);
    lsm.setGyroDataRate(LSM6DS_RATE_104_HZ);

    initialized = true;
    Serial.println("[IMU] LSM6DS3TR-C ready");
    return true;
}

imu_data imu_read() {
    imu_data d = {};
    if (!initialized) return d;

    sensors_event_t accel, gyro, temp;
    if (!lsm.getEvent(&accel, &gyro, &temp)) return d;

    d.accel_x = accel.acceleration.x;
    d.accel_y = accel.acceleration.y;
    d.accel_z = accel.acceleration.z;
    d.gyro_x  = gyro.gyro.x;
    d.gyro_y  = gyro.gyro.y;
    d.gyro_z  = gyro.gyro.z;
    d.temp    = temp.temperature;
    d.valid   = true;
    return d;
}
