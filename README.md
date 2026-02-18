# ESP32 Accelerometer Monitor

This project implements a high-frequency vibration monitoring system using an **Adafruit ESP32-S3 Feather** and an **Adafruit LIS331HH** triaxial accelerometer. It features secure MQTT communication over TLS, CBOR data serialization, and power-efficient operation via deep sleep.

## Hardware Requirements

- **Microcontroller**: Adafruit ESP32-S3 Feather (with 4MB Flash / 2MB PSRAM).
- **Sensor**: Adafruit LIS331HH (High-G Accelerometer) or any I2C sensor.
- **Status Indicator**: On-board NeoPixel (GPIO 17 for Adafruit ESP32-S3).

## Core Functionality (`main.cpp`)

The [main.cpp] script follows a linear execution flow before entering deep sleep:

1.  **Configuration**: Loads system parameters (WiFi, MQTT, Sensor settings) from `data/config.json` stored in the LittleFS filesystem.
2.  **Connectivity**: 
    - Connects to WiFi.
    - Synchronizes time via NTP (required for TLS certificate validation).
    - Established a secure MQTT connection over TLS (using `data/ca.pem`).
3.  **Data Acquisition**: 
    - Initializes the LIS331HH via I2C.
    - Captures a burst of N samples (defined in config) at a target frequency (e.g., 1000Hz).
    - Microsecond-accurate timing using `esp_timer_get_time()`.
4.  **RMS Gating**: Calculates the RMS magnitude of the acquired burst. Data is only transmitted if the vibration exceeds a predefined threshold (`MAG_RMS_THRESHOLD`), significantly saving bandwidth and power.
5.  **Data Packing**: Serializes metadata and raw burst data into **CBOR** (Concise Binary Object Representation) for efficient transmission.
6.  **Transmission**: Publishes to MQTT topics in multiple blobs (Meta, X, Y, Z, and timing data).
7.  **Power Management**: Enters **Deep Sleep** for a configured duration after successful transmission or if the threshold isn't met.

### Status LED (NeoPixel)
- **Solid Green**: Initializing and acquiring data.
- **Blinking Green**: Success, entering sleep.
- **Blinking Yellow**: Data acquired but below threshold (no transmission).
- **Blinking Red**: Error state (1: WiFi, 2: NTP, 3: MQTT, 4: Sensor, 5: Config).

## Partition Table

The project uses a custom partition scheme [partitions_adafruit_no_ota.csv] to maximize space for the application and the filesystem:

- **nvs**: Non-volatile storage for WiFi data (20KB).
- **app0**: The main application firmware (approx. 1.4MB).
- **spiffs/littlefs**: Filesystem for configuration and certificates (approx. 1.3MB).

## Configuration (`config.json`)

You must provide a `config.json` file in the `data/` folder. Use [config_example.json] as a template:

```json
{
  "wifi": { "ssid": "...", "password": "..." },
  "mqtt": { "host": "...", "username": "...", "password": "...", "topic": "..." },
  "sensor": { "i2c_addr": 24, "range_g": 24 },
  "acq": { "n_samples": 500, "fs_hz": 1000 },
  "sleep": { "seconds": 300 }
}
```

> [!IMPORTANT]
> A valid `ca.pem` file must also be present in the `data/` folder for TLS MQTT connections.

## How to Upload

This project uses **PlatformIO**.

1.  **Preparation**:
    - Build your `config.json` and place it in the `data/` directory.
    - Ensure your CA certificate is named `ca.pem` in the `data/` directory.
2.  **Upload Firmware**:
    - Connect the ESP32.
    - Run `pio run -t upload` or use the "Upload" button in VS Code.
3.  **Upload Filesystem**:
    - **This is critical** for loading the configuration.
    - Run `pio run -t uploadfs` or use the "Upload Filesystem Image" task in PlatformIO.
