// main.cpp
// ESP32-S3 Feather + LIS331HH (I2C) + HiveMQ TLS + NTP + MQTT + TinyCBOR (TOKITA)
//
// Fixes applied:
// 1) meta epoch_s + iso are derived from t0_us (acquisition start), so timestamps are coherent.
// 2) dt timing uses esp_timer_get_time() (monotonic, no wrap issues) instead of micros().
// 3) Added on-device validation prints: dt stats + duration + end-time cross-check.

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <LittleFS.h>
#include <ArduinoJson.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_LIS331HH.h>

#include <time.h>
#include <sys/time.h>

#include <esp_timer.h>   // esp_timer_get_time()

#include <cbor.h>
#include <Adafruit_NeoPixel.h>

// -------------------------
// Config
// -------------------------
struct Config {
  String client_id;

  String wifi_ssid;
  String wifi_password;

  String mqtt_host;
  uint16_t mqtt_port = 8883;
  String mqtt_user;
  String mqtt_pass;
  String mqtt_topic = "dimitri_esp32";

  String ca_path = "/ca.pem";

  uint8_t i2c_addr = 0x18;
  uint8_t range_g = 24;

  // NTP
  String ntp_server1 = "pool.ntp.org";
  String ntp_server2 = "time.nist.gov";
  String ntp_server3 = "time.google.com";
  uint16_t ntp_timeout_s = 15;

  // Acquisition
  uint16_t n_samples = 500;      // 500 samples
  uint16_t fs_hz = 1000;         // target rate
  // Sleep
  uint32_t sleep_s = 300;        // 5 minutes
};

static Config cfg;
static String ca_pem;

static WiFiClientSecure tlsClient;
static PubSubClient mqtt(tlsClient);

static Adafruit_LIS331HH lis;

// -------------------------
// NeoPixel status (GPIO 17)
// -------------------------
static constexpr uint8_t NEOPIXEL_PIN = 33;

static Adafruit_NeoPixel pixel(1, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

static inline uint32_t C_GREEN() { return pixel.Color(0, 255, 0); }
static inline uint32_t C_RED()   { return pixel.Color(255, 0, 0); }
static inline uint32_t C_YELLOW(){ return pixel.Color(255, 255, 0); }
static inline uint32_t C_OFF()   { return pixel.Color(0, 0, 0); }

static void pixelBegin() {
  pixel.begin();
  pixel.setBrightness(40);   // adjust if needed (0..255)
  pixel.clear();
  pixel.show();
}

static void pixelSetSolid(uint32_t c) {
  pixel.setPixelColor(0, c);
  pixel.show();
}

static void pixelBlink(uint32_t c, uint8_t times, uint16_t on_ms = 200, uint16_t off_ms = 200) {
  for (uint8_t i = 0; i < times; i++) {
    pixelSetSolid(c);
    delay(on_ms);
    pixelSetSolid(C_OFF());
    delay(off_ms);
  }
}

// Failure pattern: blink red N times every 3 seconds, for 15 seconds, then restart
static void failAndRestart(uint8_t red_blinks) {
  const uint32_t start = millis();

  // keep trying to signal for 15 seconds total
  while (millis() - start < 15000UL) {
    // one "cycle" ~3s
    uint32_t cycleStart = millis();

    // blink pattern
    pixelBlink(C_RED(), red_blinks, 200, 200);

    // wait until 3 seconds since cycle start (approx)
    while (millis() - cycleStart < 3000UL) {
      delay(20);
    }
  }

  // ensure LED off before reset (optional)
  pixelSetSolid(C_OFF());
  delay(100);
  ESP.restart();
}

// -------------------------
// Helpers: FS
// -------------------------
static bool readFileToString(const char* path, String& out) {
  File f = LittleFS.open(path, "r");
  if (!f) return false;
  out = f.readString();
  f.close();
  return true;
}

static bool mountFS() {
  if (!LittleFS.begin(false)) {
    Serial.println("LittleFS mount failed");
    return false;
  }
  return true;
}

static bool loadConfig() {
  if (!mountFS()) return false;

  String json;
  if (!readFileToString("/config.json", json)) {
    Serial.println("Missing /config.json");
    return false;
  }

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, json);
  if (err) {
    Serial.print("config.json parse error: ");
    Serial.println(err.c_str());
    return false;
  }

  cfg.client_id     = doc["device"]["client_id"] | String("esp32s3-lis331-01");

  cfg.wifi_ssid     = doc["wifi"]["ssid"] | String("");
  cfg.wifi_password = doc["wifi"]["password"] | String("");

  cfg.mqtt_host     = doc["mqtt"]["host"] | String("");
  cfg.mqtt_port     = doc["mqtt"]["port"] | 8883;
  cfg.mqtt_user     = doc["mqtt"]["username"] | String("");
  cfg.mqtt_pass     = doc["mqtt"]["password"] | String("");
  cfg.mqtt_topic    = doc["mqtt"]["topic"] | String("dimitri_esp32");

  cfg.ca_path       = doc["tls"]["ca_path"] | String("/ca.pem");

  cfg.i2c_addr      = doc["sensor"]["i2c_addr"] | 0x18;
  cfg.range_g       = doc["sensor"]["range_g"] | 24;

  cfg.ntp_server1   = doc["ntp"]["server1"] | String("pool.ntp.org");
  cfg.ntp_server2   = doc["ntp"]["server2"] | String("time.nist.gov");
  cfg.ntp_server3   = doc["ntp"]["server3"] | String("time.google.com");
  cfg.ntp_timeout_s = doc["ntp"]["timeout_s"] | 15;

  cfg.n_samples     = doc["acq"]["n_samples"] | 500;
  cfg.fs_hz         = doc["acq"]["fs_hz"] | 1000;

  cfg.sleep_s       = doc["sleep"]["seconds"] | 300;

  if (cfg.wifi_ssid.isEmpty() || cfg.mqtt_host.isEmpty()) {
    Serial.println("Config missing required fields (wifi.ssid or mqtt.host)");
    return false;
  }

  if (cfg.ntp_timeout_s < 3) cfg.ntp_timeout_s = 3;
  if (cfg.ntp_timeout_s > 60) cfg.ntp_timeout_s = 60;

  if (cfg.n_samples < 10) cfg.n_samples = 10;
  if (cfg.n_samples > 2000) cfg.n_samples = 2000; // safety
  if (cfg.fs_hz < 50) cfg.fs_hz = 50;
  if (cfg.fs_hz > 2000) cfg.fs_hz = 2000;

  if (cfg.sleep_s < 5) cfg.sleep_s = 5;

  return true;
}

static bool loadCA() {
  if (!readFileToString(cfg.ca_path.c_str(), ca_pem)) {
    Serial.print("Missing CA file: ");
    Serial.println(cfg.ca_path);
    return false;
  }
  tlsClient.setCACert(ca_pem.c_str());
  return true;
}

// -------------------------
// Helpers: WiFi / MQTT
// -------------------------
static bool connectWiFi(uint32_t timeout_ms = 20000) {
  WiFi.mode(WIFI_STA);
  WiFi.begin(cfg.wifi_ssid.c_str(), cfg.wifi_password.c_str());

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    if (millis() - t0 > timeout_ms) {
      Serial.println("WiFi timeout");
      return false;
    }
  }

  Serial.print("WiFi connected, IP=");
  Serial.println(WiFi.localIP());
  return true;
}

static bool connectMQTT(uint32_t timeout_ms = 20000) {
  mqtt.setServer(cfg.mqtt_host.c_str(), cfg.mqtt_port);

  uint32_t t0 = millis();
  while (!mqtt.connected()) {
    bool ok = mqtt.connect(cfg.client_id.c_str(),
                           cfg.mqtt_user.c_str(),
                           cfg.mqtt_pass.c_str());
    if (ok) {
      Serial.println("MQTT connected (TLS verified)");
      return true;
    }
    delay(1000);
    if (millis() - t0 > timeout_ms) {
      Serial.print("MQTT timeout, state=");
      Serial.println(mqtt.state());
      return false;
    }
  }
  return true;
}

// -------------------------
// Helpers: NTP / time
// -------------------------
static bool syncTimeNTP() {
  configTime(0, 0,
             cfg.ntp_server1.c_str(),
             cfg.ntp_server2.c_str(),
             cfg.ntp_server3.c_str());

  const uint32_t deadline = millis() + (uint32_t)cfg.ntp_timeout_s * 1000UL;

  time_t now = 0;
  while (millis() < deadline) {
    time(&now);
    if (now > 1577836800) { // 2020-01-01
      Serial.print("NTP OK, epoch=");
      Serial.println((uint32_t)now);
      return true;
    }
    delay(250);
  }

  Serial.println("NTP sync timeout");
  return false;
}

static uint64_t epochUsNow() {
  timeval tv;
  gettimeofday(&tv, nullptr);
  return (uint64_t)tv.tv_sec * 1000000ULL + (uint64_t)tv.tv_usec;
}

static void formatISO8601UTC(time_t t, char* out, size_t out_len) {
  struct tm tm_utc;
  gmtime_r(&t, &tm_utc);
  strftime(out, out_len, "%Y-%m-%dT%H:%M:%SZ", &tm_utc);
}

// ISO with microseconds, based on epoch_us
static void formatISO8601UTC_us(uint64_t epoch_us, char* out, size_t out_len) {
  time_t s = (time_t)(epoch_us / 1000000ULL);
  uint32_t us = (uint32_t)(epoch_us % 1000000ULL);

  char base[32];
  formatISO8601UTC(s, base, sizeof(base)); // "...Z"

  // Replace trailing 'Z' with ".%06uZ"
  // base is like "YYYY-MM-DDTHH:MM:SSZ"
  size_t n = strlen(base);
  if (n < 2 || base[n - 1] != 'Z') {
    snprintf(out, out_len, "%s", base);
    return;
  }
  base[n - 1] = '\0';
  snprintf(out, out_len, "%s.%06luZ", base, (unsigned long)us);
}

// -------------------------
// Helpers: Sensor
// -------------------------
static bool initLIS331() {
  Wire.begin();
  Wire.setClock(400000);

  if (!lis.begin_I2C(cfg.i2c_addr)) {
    Serial.println("LIS331HH begin_I2C failed");
    return false;
  }

  if (cfg.range_g == 6) lis.setRange(LIS331HH_RANGE_6_G);
  else if (cfg.range_g == 12) lis.setRange(LIS331HH_RANGE_12_G);
  else lis.setRange(LIS331HH_RANGE_24_G);

  // Configure ODR closer to target
  if (cfg.fs_hz >= 1000) lis.setDataRate(LIS331_DATARATE_1000_HZ);
  else if (cfg.fs_hz >= 400) lis.setDataRate(LIS331_DATARATE_400_HZ);
  else if (cfg.fs_hz >= 100) lis.setDataRate(LIS331_DATARATE_100_HZ);
  else lis.setDataRate(LIS331_DATARATE_50_HZ);

  return true;
}

// Convert m/s^2 -> mg (int16)
static int16_t mps2_to_mg(float a_mps2) {
  const float g0 = 9.80665f;
  float mg = (a_mps2 / g0) * 1000.0f;
  long v = lroundf(mg);
  if (v > 32767) v = 32767;
  if (v < -32768) v = -32768;
  return (int16_t)v;
}

// -------------------------
// Packing: little-endian buffers
// -------------------------
static inline void put_u16_le(uint8_t* dst, uint16_t v) {
  dst[0] = (uint8_t)(v & 0xFF);
  dst[1] = (uint8_t)((v >> 8) & 0xFF);
}
static inline void put_i16_le(uint8_t* dst, int16_t v) {
  put_u16_le(dst, (uint16_t)v);
}

// -------------------------
// CBOR publish helpers
// -------------------------
static bool mqttPublishCbor(const uint8_t* payload, size_t len) {
  bool ok = mqtt.publish(cfg.mqtt_topic.c_str(), (const uint8_t*)payload, len, false);
  // Give time to flush before next message
  uint32_t t0 = millis();
  while (millis() - t0 < 200) {
    mqtt.loop();
    delay(5);
  }
  return ok;
}

static bool publishMetaCbor(const char* id_msg,
                            uint64_t epoch_us0,
                            time_t epoch_s,
                            const char* iso_utc,
                            bool ntp_ok,
                            uint16_t n_samples,
                            uint16_t fs_hz) {
  uint8_t buf[512];
  CborEncoder root, map;

  cbor_encoder_init(&root, buf, sizeof(buf), 0);
  CborError err = cbor_encoder_create_map(&root, &map, 12);
  if (err) return false;

  err = cbor_encode_text_stringz(&map, "type"); if (err) return false;
  err = cbor_encode_text_stringz(&map, "meta"); if (err) return false;

  err = cbor_encode_text_stringz(&map, "id"); if (err) return false;
  err = cbor_encode_text_stringz(&map, id_msg); if (err) return false;

  err = cbor_encode_text_stringz(&map, "dev"); if (err) return false;
  err = cbor_encode_text_stringz(&map, cfg.client_id.c_str()); if (err) return false;

  err = cbor_encode_text_stringz(&map, "ip"); if (err) return false;
  String ip = WiFi.localIP().toString();
  err = cbor_encode_text_stringz(&map, ip.c_str()); if (err) return false;

  err = cbor_encode_text_stringz(&map, "ntp"); if (err) return false;
  err = cbor_encode_uint(&map, ntp_ok ? 1 : 0); if (err) return false;

  err = cbor_encode_text_stringz(&map, "epoch_s"); if (err) return false;
  err = cbor_encode_uint(&map, (uint32_t)epoch_s); if (err) return false;

  err = cbor_encode_text_stringz(&map, "iso"); if (err) return false;
  err = cbor_encode_text_stringz(&map, iso_utc); if (err) return false;

  err = cbor_encode_text_stringz(&map, "t0_us"); if (err) return false;
  err = cbor_encode_uint(&map, (uint64_t)epoch_us0); if (err) return false;

  err = cbor_encode_text_stringz(&map, "n"); if (err) return false;
  err = cbor_encode_uint(&map, n_samples); if (err) return false;

  err = cbor_encode_text_stringz(&map, "fs"); if (err) return false;
  err = cbor_encode_uint(&map, fs_hz); if (err) return false;

  err = cbor_encode_text_stringz(&map, "dt_fmt"); if (err) return false;
  err = cbor_encode_text_stringz(&map, "u16le_us"); if (err) return false;

  err = cbor_encode_text_stringz(&map, "a_fmt"); if (err) return false;
  err = cbor_encode_text_stringz(&map, "i16le_mg"); if (err) return false;

  err = cbor_encoder_close_container(&root, &map); if (err) return false;

  size_t nbytes = cbor_encoder_get_buffer_size(&root, buf);
  return mqttPublishCbor(buf, nbytes);
}

static bool publishBlobCbor(const char* type,
                            const char* id_msg,
                            const char* key,
                            const uint8_t* blob,
                            size_t blob_len,
                            uint16_t idx,
                            uint16_t total_parts) {
  uint8_t buf[1400];
  CborEncoder root, map;

  cbor_encoder_init(&root, buf, sizeof(buf), 0);

  CborError err = cbor_encoder_create_map(&root, &map, 5);
  if (err) return false;

  err = cbor_encode_text_stringz(&map, "type"); if (err) return false;
  err = cbor_encode_text_stringz(&map, type); if (err) return false;

  err = cbor_encode_text_stringz(&map, "id"); if (err) return false;
  err = cbor_encode_text_stringz(&map, id_msg); if (err) return false;

  err = cbor_encode_text_stringz(&map, "idx"); if (err) return false;
  err = cbor_encode_uint(&map, idx); if (err) return false;

  err = cbor_encode_text_stringz(&map, "parts"); if (err) return false;
  err = cbor_encode_uint(&map, total_parts); if (err) return false;

  err = cbor_encode_text_stringz(&map, key); if (err) return false;
  err = cbor_encode_byte_string(&map, blob, blob_len); if (err) return false;

  err = cbor_encoder_close_container(&root, &map); if (err) return false;

  size_t nbytes = cbor_encoder_get_buffer_size(&root, buf);
  Serial.print("CBOR msg bytes="); Serial.println(nbytes);
  return mqttPublishCbor(buf, nbytes);
}

// -------------------------
// Acquisition (N samples)
// -------------------------
static bool acquireN(uint16_t N,
                     uint16_t fs_hz,
                     uint64_t& epoch_us0,
                     uint16_t* dt_us,   // length N-1
                     int16_t* ax_mg,    // length N
                     int16_t* ay_mg,
                     int16_t* az_mg,
                     uint64_t& dt_sum_us_out) {

  if (N < 2) return false;

  // Absolute wall-clock start (NTP-derived if synced)
  epoch_us0 = epochUsNow();

  // Monotonic reference for relative timing
  int64_t t0_rel_us = esp_timer_get_time();
  int64_t last_t_us = t0_rel_us;

  const uint32_t period_us = (uint32_t)(1000000UL / (uint32_t)fs_hz);
  uint64_t dt_sum_us = 0;

  for (uint16_t i = 0; i < N; i++) {
    // Soft schedule: target time since t0
    if (i > 0) {
      int64_t target = t0_rel_us + (int64_t)i * (int64_t)period_us;
      while (esp_timer_get_time() < target) {
        delayMicroseconds(50);
      }
    }

    int64_t t_now_us = esp_timer_get_time();

    if (i > 0) {
      int64_t d64 = t_now_us - last_t_us;
      if (d64 < 0) d64 = 0;
      uint32_t d = (uint32_t)d64;
      if (d > 65535UL) d = 65535UL;
      dt_us[i - 1] = (uint16_t)d;
      dt_sum_us += d;
    }
    last_t_us = t_now_us;

    sensors_event_t event;
    lis.getEvent(&event);

    ax_mg[i] = mps2_to_mg((float)event.acceleration.x);
    ay_mg[i] = mps2_to_mg((float)event.acceleration.y);
    az_mg[i] = mps2_to_mg((float)event.acceleration.z);
  }

  dt_sum_us_out = dt_sum_us;
  return true;
}

// -------------------------
// Deep sleep
// -------------------------
static void goToSleep(uint32_t seconds) {
  pixelSetSolid(C_OFF());   // add this
  mqtt.disconnect();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);

  esp_sleep_enable_timer_wakeup((uint64_t)seconds * 1000000ULL);
  Serial.print("Sleeping for ");
  Serial.print(seconds);
  Serial.println(" s");
  Serial.flush();
  delay(100);
  esp_deep_sleep_start();
}

// -------------------------
// main
// -------------------------
static void makeIdMsg(char* out, size_t out_len, uint64_t epoch_us0) {
  uint32_t low = (uint32_t)(epoch_us0 & 0xFFFFFFFFULL);
  snprintf(out, out_len, "%s-%lu", cfg.client_id.c_str(), (unsigned long)low);
}

static float computeMagRms_mps2(const int16_t* ax_mg,
                               const int16_t* ay_mg,
                               const int16_t* az_mg,
                               uint16_t N) {
  // Convert mg -> m/s^2: (mg/1000)*g0
  const float g0 = 9.80665f;
  double sum_sq = 0.0;

  for (uint16_t i = 0; i < N; i++) {
    const float ax = ((float)ax_mg[i] / 1000.0f) * g0;
    const float ay = ((float)ay_mg[i] / 1000.0f) * g0;
    const float az = ((float)az_mg[i] / 1000.0f) * g0;
    const double mag2 = (double)ax * ax + (double)ay * ay + (double)az * az;
    sum_sq += mag2;
  }

  const double mean_sq = sum_sq / (double)N;
  return (float)sqrt(mean_sq);
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  // Start NeoPixel ASAP
  pixelBegin();

  // During setup + measurement: solid green
  pixelSetSolid(C_GREEN());

  // Config/FS/CA errors -> treat as generic init error (4 blinks)
  if (!loadConfig()) { failAndRestart(5); }
  if (!loadCA())     { failAndRestart(5); }

  // WiFi (1 blink red)
  if (!connectWiFi()) { failAndRestart(1); }

  bool ntp_ok = syncTimeNTP();
  if (!ntp_ok) { failAndRestart(2); }
  Serial.printf("epochUsNow=%llu (ntp_ok=%u)\n",
                (unsigned long long)epochUsNow(),
                ntp_ok ? 1 : 0);

  // MQTT connect (3 blinks red)
  if (!connectMQTT()) { failAndRestart(3); }

  // Sensor init (4 blinks red)
  if (!initLIS331())  { failAndRestart(4); }

  const uint16_t N = cfg.n_samples;

  static uint16_t dt_us_buf[2000];   // max N-1
  static int16_t ax_mg_buf[2000];
  static int16_t ay_mg_buf[2000];
  static int16_t az_mg_buf[2000];

  uint64_t epoch_us0 = 0;
  uint64_t dt_sum_us = 0;

  bool ok_acq = acquireN(N, cfg.fs_hz, epoch_us0,
                         dt_us_buf,
                         ax_mg_buf, ay_mg_buf, az_mg_buf,
                         dt_sum_us);

  if (!ok_acq) {
    Serial.println("Acquisition failed");
    // Not specified; treat as sensor-ish failure (4)
    failAndRestart(4);
  }

  // -------------------------
  // Timestamp coherence for meta (derived from t0_us)
  // -------------------------
  time_t t0_s = (time_t)(epoch_us0 / 1000000ULL);
  char iso_us[48];
  formatISO8601UTC_us(epoch_us0, iso_us, sizeof(iso_us));

  // -------------------------
  // On-device validation prints
  // -------------------------
  const uint32_t target_period_us = 1000000UL / (uint32_t)cfg.fs_hz;

  uint32_t dt_min = 0xFFFFFFFF, dt_max = 0;
  uint64_t dt_sum_check = 0;
  uint32_t sat_cnt = 0;

  for (uint16_t i = 0; i < (N - 1); i++) {
    uint32_t d = dt_us_buf[i];
    dt_sum_check += d;
    if (d < dt_min) dt_min = d;
    if (d > dt_max) dt_max = d;
    if (d == 65535) sat_cnt++;
  }

  double dt_mean = (N > 1) ? ((double)dt_sum_check / (double)(N - 1)) : 0.0;
  Serial.printf("dt stats: min=%lu us, max=%lu us, mean=%.2f us, target=%lu us, sat=%lu\n",
                (unsigned long)dt_min,
                (unsigned long)dt_max,
                dt_mean,
                (unsigned long)target_period_us,
                (unsigned long)sat_cnt);

  Serial.printf("duration_est = %.3f ms (expected %.3f ms)\n",
                (double)dt_sum_check / 1000.0,
                (double)(N - 1) * (double)target_period_us / 1000.0);

  // Cross-check end epoch vs t0 + sum(dt)
  uint64_t epoch_us_end_now = epochUsNow();
  uint64_t epoch_us_end_est = epoch_us0 + dt_sum_check;
  int64_t err_us = (int64_t)(epoch_us_end_now - epoch_us_end_est);

  Serial.printf("end check: now=%llu, est=%llu, err=%lld us\n",
                (unsigned long long)epoch_us_end_now,
                (unsigned long long)epoch_us_end_est,
                (long long)err_us);
  // -------------------------
  // RMS magnitude gate
  // -------------------------
  const float MAG_RMS_THRESHOLD = 10.78f; // m/s^2

  float mag_rms = computeMagRms_mps2(ax_mg_buf, ay_mg_buf, az_mg_buf, N);
  Serial.printf("mag_rms=%.3f m/s^2 (threshold=%.2f)\n", mag_rms, MAG_RMS_THRESHOLD);

  if (mag_rms < MAG_RMS_THRESHOLD) {
    // Do not publish
    pixelBlink(C_YELLOW(), 3, 400, 400);  // 3 yellow blinks, 400 ms
    pixelSetSolid(C_OFF());
    goToSleep(cfg.sleep_s);
    return;
  }
  // -------------------------
  // Publish
  // -------------------------
  char id_msg[64];
  makeIdMsg(id_msg, sizeof(id_msg), epoch_us0);

  // 1) meta (coherent with acquisition t0)
  bool ok = publishMetaCbor(id_msg, epoch_us0, t0_s, iso_us, ntp_ok, N, cfg.fs_hz);
  Serial.print("pub meta: "); Serial.println(ok ? "ok" : "fail");

  // Pack dt (N-1) into bytes (u16le)
  const uint16_t dt_count = (N > 0) ? (N - 1) : 0;
  static uint8_t dt_bytes[2 * 2000];
  for (uint16_t i = 0; i < dt_count; i++) {
    put_u16_le(&dt_bytes[2 * i], dt_us_buf[i]);
  }

  // Pack axes into bytes (i16le)
  static uint8_t x_bytes[2 * 2000];
  static uint8_t y_bytes[2 * 2000];
  static uint8_t z_bytes[2 * 2000];

  for (uint16_t i = 0; i < N; i++) {
    put_i16_le(&x_bytes[2 * i], ax_mg_buf[i]);
    put_i16_le(&y_bytes[2 * i], ay_mg_buf[i]);
    put_i16_le(&z_bytes[2 * i], az_mg_buf[i]);
  }

  // 2..5) blobs (parts=1 for now)
  const uint16_t parts = 1;

  ok = publishBlobCbor("dt", id_msg, "dt", dt_bytes, (size_t)dt_count * 2, 0, parts);
  Serial.print("pub dt: "); Serial.println(ok ? "ok" : "fail");

  delay(4000);
  ok = publishBlobCbor("x", id_msg, "a", x_bytes, (size_t)N * 2, 0, parts);
  Serial.print("pub x: "); Serial.println(ok ? "ok" : "fail");

  delay(4000);
  ok = publishBlobCbor("y", id_msg, "a", y_bytes, (size_t)N * 2, 0, parts);
  Serial.print("pub y: "); Serial.println(ok ? "ok" : "fail");

  delay(4000);
  ok = publishBlobCbor("z", id_msg, "a", z_bytes, (size_t)N * 2, 0, parts);
  Serial.print("pub z: "); Serial.println(ok ? "ok" : "fail");

  // Final flush window before sleep
  uint32_t t0 = millis();
  while (millis() - t0 < 5000) {
    mqtt.loop();
    delay(10);
  }

  // Success pattern: 3 green blinks, then sleep
  pixelBlink(C_GREEN(), 5, 350, 350);
  pixelSetSolid(C_OFF());

  goToSleep(cfg.sleep_s);
}

void loop() {
  // unused
}
