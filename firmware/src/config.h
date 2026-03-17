#pragma once

// ── WiFi ─────────────────────────────────────────────────────────────────────
// WIFI_SSID and WIFI_PASS are injected as build flags from ../.env
// Run 'make setup-env' from the repo root to configure.
#ifndef WIFI_SSID
#error "WIFI_SSID not defined — run 'make setup-env' and rebuild"
#endif
#ifndef WIFI_PASS
#error "WIFI_PASS not defined — run 'make setup-env' and rebuild"
#endif

// ── I2C (IMU) ────────────────────────────────────────────────────────────────
#define PIN_SDA  2
#define PIN_SCL  3
#define IMU_ADDR 0x6A

// ── Motor driver (DRV8833) ───────────────────────────────────────────────────
#define PIN_AIN1 12  // Left motor forward
#define PIN_AIN2 13  // Left motor reverse
#define PIN_BIN1 14  // Right motor forward
#define PIN_BIN2 15  // Right motor reverse

// PWM settings
#define MOTOR_PWM_FREQ 20000  // 20 kHz — above audible range
#define MOTOR_PWM_RES  8      // 8-bit resolution (0–255)

// WiFi
#define WIFI_MAX_RETRIES   80       // Total connection attempts before reboot
#define WIFI_RETRY_MS      500      // Delay between retries

// Serial
#define SERIAL_BUF_SIZE    64       // Serial command buffer length

// Stream
#define STREAM_FRAME_MIN_MS 30      // ~30fps cap on MJPEG stream
