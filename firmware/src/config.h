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
#define STREAM_FRAME_MIN_MS 30      // ~30fps cap on MJPEG stream
