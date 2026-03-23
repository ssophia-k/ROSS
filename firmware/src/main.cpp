/*
 * ROSS ESP32-CAM Firmware
 *
 * Streams MJPEG video over WiFi.
 *
 * Additional hardware (IMU, motors) is wired but not yet enabled
 * in firmware — this build validates camera + WiFi only.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WebServer.h>

#include "camera.h"
#include "config.h"

// ── Globals ──────────────────────────────────────────────────────────────────

WebServer server(80);

// ── HTTP Handlers ────────────────────────────────────────────────────────────

void handle_root() {
    String html = "<html><body>"
                  "<h1>ROSS</h1>"
                  "<p><a href=\"/stream\">MJPEG Stream</a></p>"
                  "</body></html>";
    server.send(200, "text/html", html);
}

// ── MJPEG Stream ─────────────────────────────────────────────────────────────

void handle_stream() {
    WiFiClient client = server.client();
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: multipart/x-mixed-replace; boundary=frame");
    client.println();

    while (client.connected()) {
        unsigned long frame_start = millis();

        camera_frame frame = camera_capture();
        if (!frame.buf) continue;

        client.printf("--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", frame.len);
        client.write(frame.buf, frame.len);
        client.println();
        camera_release(frame);

        unsigned long elapsed = millis() - frame_start;
        if (elapsed < STREAM_FRAME_MIN_MS) {
            delay(STREAM_FRAME_MIN_MS - elapsed);
        }
    }
}

// ── Setup & Loop ─────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    Serial.println();
    Serial.println("[ROSS] Booting...");

    // Camera
    if (!camera_init()) {
        Serial.println("[CAM] Init failed — halting");
        while (true) delay(1000);
    }
    Serial.println("[CAM] Ready");

    // WiFi
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.printf("[WiFi] Connecting to %s", WIFI_SSID);
    int wifi_attempts = 0;
    while (WiFi.status() != WL_CONNECTED && wifi_attempts < WIFI_MAX_RETRIES) {
        delay(WIFI_RETRY_MS);
        Serial.print(".");
        wifi_attempts++;
        if (wifi_attempts == WIFI_MAX_RETRIES / 2) {
            Serial.println("\n[WiFi] Retrying...");
            WiFi.disconnect();
            delay(1000);
            WiFi.begin(WIFI_SSID, WIFI_PASS);
        }
    }
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\n[WiFi] Failed after max retries — rebooting in 5s");
        delay(5000);
        ESP.restart();
    }
    Serial.printf("\n[WiFi] Connected — http://%s\n", WiFi.localIP().toString().c_str());

    // mDNS
    if (MDNS.begin("ross")) {
        MDNS.addService("http", "tcp", 80);
        Serial.println("[mDNS] ross.local");
    } else {
        Serial.println("[mDNS] Failed to start");
    }

    // HTTP routes
    server.on("/",       HTTP_GET, handle_root);
    server.on("/stream", HTTP_GET, handle_stream);
    server.begin();
    Serial.println("[HTTP] Server started");
}

void loop() {
    server.handleClient();
}
