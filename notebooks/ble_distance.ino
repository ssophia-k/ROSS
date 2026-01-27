#include <ArduinoBLE.h>

const char targetName[] = "Proximity";

unsigned long nextPrintMs = 0;
const unsigned long PRINT_PERIOD_MS = 100;     // steady output cadence
const unsigned long STALE_AFTER_MS   = 1000;   // mark stale if no adv seen

int lastRSSI = 0;
unsigned long lastSeenMs = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1) {}
  }

  BLE.scan(true);  // continuous scan
  Serial.println("time_ms\tRSSI_dBm");
}

void loop() {
  BLE.poll();  // service BLE stack

  // Update RSSI whenever an advertisement is received
  BLEDevice p = BLE.available();
  if (p && p.hasLocalName() && p.localName() == targetName) {
    lastRSSI = p.rssi();
    lastSeenMs = millis();
  }

  // Print at a fixed rate regardless of new packets
  unsigned long now = millis();
  if (now >= nextPrintMs) {
    nextPrintMs = now + PRINT_PERIOD_MS;

    if (now - lastSeenMs <= STALE_AFTER_MS) {
      Serial.print(now);
      Serial.print('\t');
      Serial.println(lastRSSI);
    } else {
      // No recent packets: output NaN or a sentinel
      Serial.print(now);
      Serial.print('\t');
      Serial.println("nan");  // Serial Plotter handles "nan"
    }
  }
}
