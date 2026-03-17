/*
 * ROSS ESP32-CAM Firmware
 *
 * Streams MJPEG video and IMU data over WiFi.
 * Receives motor velocity commands from the Linux workstation.
 *
 * Pin assignments (from README):
 *   GPIO 2  — I2C SDA (IMU)
 *   GPIO 3  — I2C SCL (IMU) — shared with UART RX, safe at runtime
 *   GPIO 12 — DRV8833 AIN1 (left motor forward)
 *   GPIO 13 — DRV8833 AIN2 (left motor reverse)
 *   GPIO 14 — DRV8833 BIN1 (right motor forward)
 *   GPIO 15 — DRV8833 BIN2 (right motor reverse)
 */

#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_LSM6DS3TRC.h>

#include "camera.h"
#include "motors.h"
#include "config.h"

// ── Globals ──────────────────────────────────────────────────────────────────

WebServer server(80);
Adafruit_LSM6DS3TRC imu;
bool imu_ok = false;
bool serial_mode = false;

// ── IMU ──────────────────────────────────────────────────────────────────────

void setup_imu() {
    Wire.begin(PIN_SDA, PIN_SCL);
    if (imu.begin_I2C(IMU_ADDR, &Wire)) {
        imu_ok = true;
        imu.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
        imu.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
        imu.setAccelDataRate(LSM6DS_RATE_104_HZ);
        imu.setGyroDataRate(LSM6DS_RATE_104_HZ);
        Serial.println("[IMU] LSM6DS3TR-C ready");
    } else {
        Serial.println("[IMU] LSM6DS3TR-C not found at 0x6A");
    }
}

// ── HTTP Handlers ────────────────────────────────────────────────────────────

void handle_root() {
    String html = "<html><body>"
                  "<h1>ROSS</h1>"
                  "<p><a href=\"/stream\">MJPEG Stream</a></p>"
                  "<p><a href=\"/imu\">IMU Data</a></p>"
                  "</body></html>";
    server.send(200, "text/html", html);
}

void handle_imu() {
    if (!imu_ok) {
        server.send(503, "application/json", "{\"error\":\"IMU not available\"}");
        return;
    }

    sensors_event_t accel, gyro, temp;
    imu.getEvent(&accel, &gyro, &temp);

    char buf[256];
    snprintf(buf, sizeof(buf),
             "{\"accel\":{\"x\":%.4f,\"y\":%.4f,\"z\":%.4f},"
             "\"gyro\":{\"x\":%.4f,\"y\":%.4f,\"z\":%.4f},"
             "\"temp\":%.2f}",
             accel.acceleration.x, accel.acceleration.y, accel.acceleration.z,
             gyro.gyro.x, gyro.gyro.y, gyro.gyro.z,
             temp.temperature);
    server.send(200, "application/json", buf);
}

void handle_motor() {
    if (!server.hasArg("l") || !server.hasArg("r")) {
        server.send(400, "text/plain", "Need ?l=<-255..255>&r=<-255..255>");
        return;
    }

    int left  = constrain(server.arg("l").toInt(), -255, 255);
    int right = constrain(server.arg("r").toInt(), -255, 255);
    motors_set(left, right);

    char buf[64];
    snprintf(buf, sizeof(buf), "{\"left\":%d,\"right\":%d}", left, right);
    server.send(200, "application/json", buf);
}

void handle_stop() {
    motors_stop();
    server.send(200, "application/json", "{\"left\":0,\"right\":0}");
}

// ── MJPEG Stream ─────────────────────────────────────────────────────────────

void handle_stream() {
    WiFiClient client = server.client();
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: multipart/x-mixed-replace; boundary=frame");
    client.println();

    while (client.connected()) {
        camera_frame frame = camera_capture();
        if (!frame.buf) continue;

        client.printf("--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", frame.len);
        client.write(frame.buf, frame.len);
        client.println();
        camera_release(frame);
    }
}

// ── Setup & Loop ─────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    Serial.println();
    Serial.println("[ROSS] Booting...");

    // Motors first (ensure stopped)
    motors_init();

    // Wait briefly for serial handshake — if received, enter serial-only mode
    // (skips IMU and WiFi so GPIO 3 stays available for UART RX)
    Serial.println("[ROSS] Send 'S' within 2s for serial mode...");
    unsigned long deadline = millis() + 2000;
    while (millis() < deadline) {
        if (Serial.available()) {
            char c = Serial.read();
            if (c == 'S' || c == 's') {
                // Drain remaining bytes (e.g. newline)
                delay(50);
                while (Serial.available()) Serial.read();
                serial_mode = true;
                Serial.println("[ROSS] Serial mode — UART teleop active");
                Serial.println("[ROSS] Commands: M <left> <right> | S (stop) | I (IMU off)");
                return;
            }
        }
        delay(10);
    }

    // Camera
    if (!camera_init()) {
        Serial.println("[CAM] Init failed — halting");
        while (true) delay(1000);
    }
    Serial.println("[CAM] Ready");

    // IMU (claims GPIO 3 — UART RX no longer available)
    setup_imu();

    // WiFi
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.printf("[WiFi] Connecting to %s", WIFI_SSID);
    int wifi_attempts = 0;
    while (WiFi.status() != WL_CONNECTED && wifi_attempts < 40) {
        delay(500);
        Serial.print(".");
        wifi_attempts++;
    }
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\n[WiFi] Failed — retrying...");
        WiFi.disconnect();
        delay(1000);
        WiFi.begin(WIFI_SSID, WIFI_PASS);
        while (WiFi.status() != WL_CONNECTED) {
            delay(500);
            Serial.print(".");
        }
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
    server.on("/",      HTTP_GET, handle_root);
    server.on("/imu",   HTTP_GET, handle_imu);
    server.on("/motor", HTTP_GET, handle_motor);
    server.on("/stop",  HTTP_GET, handle_stop);
    server.on("/stream",HTTP_GET, handle_stream);
    server.begin();
    Serial.println("[HTTP] Server started");
}

// ── Serial Command Parser ────────────────────────────────────────────────────

static char serial_buf[32];
static uint8_t serial_pos = 0;

void handle_serial_line(const char* line) {
    if (line[0] == 'M' || line[0] == 'm') {
        // Motor command: M <left> <right>
        int left = 0, right = 0;
        if (sscanf(line + 1, "%d %d", &left, &right) == 2) {
            left = constrain(left, -255, 255);
            right = constrain(right, -255, 255);
            motors_set(left, right);
            Serial.printf("OK M %d %d\n", left, right);
        } else {
            Serial.println("ERR usage: M <left> <right>");
        }
    } else if (line[0] == 'S' || line[0] == 's') {
        motors_stop();
        Serial.println("OK S");
    } else if (line[0] == '?') {
        Serial.println("OK ROSS serial mode");
    } else {
        Serial.println("ERR unknown command");
    }
}

void poll_serial() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (serial_pos > 0) {
                serial_buf[serial_pos] = '\0';
                handle_serial_line(serial_buf);
                serial_pos = 0;
            }
        } else if (serial_pos < sizeof(serial_buf) - 1) {
            serial_buf[serial_pos++] = c;
        }
    }
}

// ── Main Loop ────────────────────────────────────────────────────────────────

void loop() {
    if (serial_mode) {
        poll_serial();
    } else {
        server.handleClient();
    }
}
