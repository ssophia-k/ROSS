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

// WiFi
#define WIFI_MAX_RETRIES   80       // Total connection attempts before reboot
#define WIFI_RETRY_MS      500      // Delay between retries

// Stream
#define STREAM_FRAME_MIN_MS 30

// ── Motors (DRV8833) ────────────────────────────────────────────────────────
#define PIN_AIN1  12   // Left forward   (GPIO 12 — strapping pin, floats LOW)
#define PIN_AIN2  13   // Left reverse
#define PIN_BIN1  14   // Right forward
#define PIN_BIN2  15   // Right reverse  (GPIO 15 — strapping pin)

#define MOTOR_PWM_FREQ  1000
#define MOTOR_PWM_RES   8    // 8-bit → 0–255

#define MOTOR_TIMEOUT_MS 500  // Auto-stop if no command received
