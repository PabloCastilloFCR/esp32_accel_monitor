# ESP32-C6 BME680 Office Monitor Implementation Plan

## Goal
- Convert the existing ESP32-S3 accelerometer firmware into an ESP32-C6 BME680 office-hours monitor.
- Preserve the existing high-level product flow: filesystem-backed JSON config, provisioning portal, WiFi, time sync, MQTT over TLS, and deep sleep.

## Execution Steps
1. Replace the PlatformIO target with an ESP32-C6 ESP-IDF environment and remove obsolete accelerometer-specific dependencies.
2. Split the firmware into focused modules for config, web portal, WiFi, time, scheduling, sensor access, MQTT publish, and sleep.
3. Replace LIS331/FFT/CBOR acquisition logic with BME680 I2C sampling and JSON MQTT payload publishing.
4. Implement office-hours scheduling with timezone-aware local time and safe deep-sleep fallback behavior.
5. Update documentation and examples: README, config example, migration notes, and test checklist.

## Key Decisions
- Framework: `espidf` in PlatformIO.
- Default board: `esp32-c6-devkitc-1`.
- Default I2C pins: `GPIO6` SDA and `GPIO7` SCL, overridable in config.
- Config format: JSON only.
- MQTT payload format: single human-readable JSON message.
- Filesystem: use the minimum viable ESP-IDF-compatible approach; if LittleFS is not practical in the selected stack, switch to SPIFFS and document the change.

## Validation Targets
- Project builds for the ESP32-C6 target.
- Missing or invalid config enters provisioning.
- Valid config supports WiFi, SNTP, timezone conversion, MQTT over TLS, BME680 read, office-hours scheduling, and deep sleep.
