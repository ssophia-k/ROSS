/*
 * ROSS ESP32-CAM Firmware
 *
 * Streams MJPEG video and accepts motor commands over WiFi.
 * Teleoperation via HTTP: /motor?l=X&r=Y and /stop.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WebServer.h>

#include "camera.h"
#include "config.h"
#include "imu.h"
#include "motors.h"

// ── Globals ──────────────────────────────────────────────────────────────────

WebServer server(80);        // Commands: /, /motor, /stop
WebServer stream_srv(81);   // MJPEG stream on separate port
unsigned long last_motor_cmd = 0;

// ── HTTP Handlers ────────────────────────────────────────────────────────────

void handle_root() {
    String ip = WiFi.localIP().toString();
    String html = "<html><body>"
                  "<h1>ROSS</h1>"
                  "<p><a href=\"http://" + ip + ":81/stream\">MJPEG Stream</a></p>"
                  "</body></html>";
    server.send(200, "text/html", html);
}

// ── MJPEG Stream ─────────────────────────────────────────────────────────────

void handle_stream() {
    WiFiClient client = stream_srv.client();
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

// ── Motor HTTP Handlers ─────────────────────────────────────────────────────

void handle_motor() {
    if (!server.hasArg("l") || !server.hasArg("r")) {
        server.send(400, "text/plain", "need l and r");
        return;
    }
    int left  = server.arg("l").toInt();
    int right = server.arg("r").toInt();
    motors_set(left, right);
    last_motor_cmd = millis();
    server.send(200, "text/plain", "ok");
}

void handle_stop() {
    motors_stop();
    last_motor_cmd = 0;
    server.send(200, "text/plain", "ok");
}

void handle_imu() {
    imu_data d = imu_read();
    if (!d.valid) {
        server.send(503, "text/plain", "imu not available");
        return;
    }
    char buf[256];
    snprintf(buf, sizeof(buf),
        "{\"accel\":[%.2f,%.2f,%.2f],\"gyro\":[%.2f,%.2f,%.2f],\"temp\":%.1f}",
        d.accel_x, d.accel_y, d.accel_z,
        d.gyro_x, d.gyro_y, d.gyro_z,
        d.temp);
    server.send(200, "application/json", buf);
}

// ── Stream Task (runs on core 0) ─────────────────────────────────────────────

void stream_task(void *) {
    stream_srv.on("/stream", HTTP_GET, handle_stream);
    stream_srv.begin();
    Serial.println("[HTTP] Stream server started on :81");
    for (;;) {
        stream_srv.handleClient();
        vTaskDelay(1);
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

    // Motors
    motors_init();

    // IMU (I2C on GPIO 2/3 — GPIO 3 is UART RX, safe to repurpose after boot)
    if (!imu_init()) {
        Serial.println("[IMU] Continuing without IMU");
    }

    // Command server (port 80)
    server.on("/",       HTTP_GET, handle_root);
    server.on("/motor",  HTTP_GET, handle_motor);
    server.on("/stop",   HTTP_GET, handle_stop);
    server.on("/imu",    HTTP_GET, handle_imu);
    server.begin();
    Serial.println("[HTTP] Command server started on :80");

    // Stream server (port 81, on core 0 so it doesn't block commands)
    xTaskCreatePinnedToCore(stream_task, "stream", 4096, NULL, 1, NULL, 0);
}

void loop() {
    server.handleClient();

    // Safety: stop motors if no command received recently
    if (last_motor_cmd > 0 && (millis() - last_motor_cmd) > MOTOR_TIMEOUT_MS) {
        motors_stop();
        last_motor_cmd = 0;
    }
}
