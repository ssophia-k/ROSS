#pragma once

#include <Arduino.h>
#include "config.h"

// LEDC channels for ESP32 PWM
#define CH_AIN1 0
#define CH_AIN2 1
#define CH_BIN1 2
#define CH_BIN2 3

inline void motors_init() {
    ledcSetup(CH_AIN1, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
    ledcSetup(CH_AIN2, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
    ledcSetup(CH_BIN1, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
    ledcSetup(CH_BIN2, MOTOR_PWM_FREQ, MOTOR_PWM_RES);

    ledcAttachPin(PIN_AIN1, CH_AIN1);
    ledcAttachPin(PIN_AIN2, CH_AIN2);
    ledcAttachPin(PIN_BIN1, CH_BIN1);
    ledcAttachPin(PIN_BIN2, CH_BIN2);

    // Start stopped
    ledcWrite(CH_AIN1, 0);
    ledcWrite(CH_AIN2, 0);
    ledcWrite(CH_BIN1, 0);
    ledcWrite(CH_BIN2, 0);

    Serial.println("[MOT] Initialized");
}

// Set motor speeds. Positive = forward, negative = reverse, 0 = coast.
// Range: -255 to 255 (clamped).
inline void motors_set(int left, int right) {
    left  = constrain(left,  -255, 255);
    right = constrain(right, -255, 255);

    // Left motor
    if (left >= 0) {
        ledcWrite(CH_AIN1, left);
        ledcWrite(CH_AIN2, 0);
    } else {
        ledcWrite(CH_AIN1, 0);
        ledcWrite(CH_AIN2, -left);
    }

    // Right motor
    if (right >= 0) {
        ledcWrite(CH_BIN1, right);
        ledcWrite(CH_BIN2, 0);
    } else {
        ledcWrite(CH_BIN1, 0);
        ledcWrite(CH_BIN2, -right);
    }
}

inline void motors_stop() {
    ledcWrite(CH_AIN1, 0);
    ledcWrite(CH_AIN2, 0);
    ledcWrite(CH_BIN1, 0);
    ledcWrite(CH_BIN2, 0);
}
