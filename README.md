# ESP32 Accelerometer Monitor

This project implements a high-frequency vibration monitoring system using an **Adafruit ESP32-S3 Feather** and an **Adafruit LIS331HH** triaxial accelerometer. It features secure MQTT communication over TLS, CBOR data serialization, a web-based configuration portal, and power-efficient operation via deep sleep.

## Hardware Requirements

- **Microcontroller**: Adafruit ESP32-S3 Feather (with 4MB Flash / 2MB PSRAM).
- **Sensor**: Adafruit LIS331HH (High-G Accelerometer) or any compatible I2C sensor.
- **Status Indicator**: On-board NeoPixel (GPIO 17 for Adafruit ESP32-S3).

## Core Functionality (`main.cpp`)

The `main.cpp` script follows a structured execution flow designed for reliability and ease of use:

1.  **Configuration**: Loads parameters from `data/config.json`. If configuration fails or required fields are missing, it launches the **Web Configuration Portal**.
2.  **Web Configuration Portal (AP Mode)**:
    - Creates a WiFi Access Point named `DIMITRI-[ClientID]`.
    - Features a **Captive Portal** (automatically opens the config page).
    - Modern, themed (Dark/Light/High-Contrast) web interface for updating all parameters (WiFi, MQTT, Sensor, NTP, Sleep, etc.) without reflashing.
3.  **Connectivity**: 
    - Connects to WiFi and syncs time via NTP (essential for TLS).
    - Establishes a secure MQTT connection over TLS using `data/ca.pem`.
4.  **Data Acquisition**: 
    - Captures a burst of high-frequency samples (e.g., 1000Hz) using the LIS331HH.
    - Uses `esp_timer_get_time()` for microsecond-precise sampling intervals.
5.  **Dynamic Threshold Gating**:
    - Calculates the RMS magnitude of the vibration burst.
    - Compares it against `mag_rms_threshold` (configurable via web/JSON).
    - Skips transmission if vibration is too low, saving energy and MQQT bandwidth.
6.  **CBOR Serialization & Transmission**: 
    - Packs raw data and metadata into CBOR format.
    - Publishes data in blobs (X, Y, Z, and timing) to the configured MQTT topic.
7.  **Deep Sleep**: Powers down the device for a set interval (`sleep.seconds`) to preserve battery.

### Status LED (NeoPixel)
- **Solid Green**: Normal operation (Init/Acquisition).
- **Blinking Green**: Successful transmission, entering sleep.
- **Blinking Yellow**: Data below threshold, skipping transmission.
- **Blinking Purple**: **Web Configuration Portal Active**.
- **Blinking Red**: Error state (1: WiFi, 2: NTP, 3: MQTT, 4: Sensor, 5: Config).

## Partition Table

The project uses a custom partition scheme `partitions_adafruit_no_ota.csv` to maximize space:

- **nvs**: Non-volatile storage for WiFi data (20KB).
- **app0**: Main application firmware (approx. 1.4MB).
- **spiffs/littlefs**: Filesystem for `config.json` and `ca.pem` (approx. 1.3MB).

## Web Configuration Portal

If the device cannot connect or find `config.json`, it enters "Provisioning Mode":
1.  Connect your phone/laptop to the WiFi `DIMITRI-esp32...` (Password: `dimitri1234`).
2.  A configuration page should pop up automatically. If not, go to `192.168.4.1`.
3.  Fill in the fields and click **Save & Restart**.

## Configuration (`config.json`)

Alternatively, you can manually upload [config_example.json] renamed to `config.json` in the `data/` folder:

```json
{
  "wifi": { "ssid": "...", "password": "..." },
  "mqtt": { "host": "...", "username": "...", "password": "...", "topic": "..." },
  "sensor": { "i2c_addr": 24, "range_g": 24 },
  "acq": { "n_samples": 500, "fs_hz": 1000, "mag_rms_threshold": 10.78 },
  "sleep": { "seconds": 300 }
}
```

## How to Upload

This project uses **PlatformIO**.

1.  **Preparation**: Ensure `data/config.json` and `data/ca.pem` are ready.
2.  **Upload Firmware**: `pio run -t upload`.
3.  **Upload Filesystem**: `pio run -t uploadfs` (**Crucial** for the web portal and config).
