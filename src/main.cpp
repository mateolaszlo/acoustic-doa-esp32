#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <DNSServer.h>
#include <WebServer.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <Adafruit_GFX.h>
#include <SPI.h>
#include <Adafruit_ST7735.h>
#include <Adafruit_BME280.h>
#include <esp_task_wdt.h>
#include <atomic>
#include <ctime>
#include <cstring>
#include <sys/time.h>

#include "gcc_phat.h"

// I2C Pini za ESP32-S3 (BME280)
#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9

// SPI Pini za KMR-1.8 TFT (ST7735)
#define TFT_CS_PIN 10
#define TFT_MOSI_PIN 11
#define TFT_SCLK_PIN 12
#define TFT_DC_PIN 13
#define TFT_RST_PIN 14
#define TFT_INVERT_COLORS false
#define SETUP_BUTTON_PIN 0
#define SETUP_BOOT_DELAY_MS 3000
#define SETUP_AP_NAME "Acoustic-Locator-Setup"
#define SETUP_AP_PASSWORD "configure123"
#define CONFIG_REFRESH_INTERVAL_MS 60000UL
#define HTTP_RETRY_COUNT 3
#define NETWORK_MAX_REQUEST_INTERVAL_MS 200UL
#define NETWORK_BATCH_MAX_MEASUREMENTS 8
#define NETWORK_BATCH_FLUSH_INTERVAL_MS 200UL
#define NETWORK_QUEUE_LENGTH 64
#define DSP_MAX_WINDOW_SIZE 2048
#define DSP_DIAGNOSTIC_INTERVAL_MS 10000UL
#define TASK_WATCHDOG_TIMEOUT_SECONDS 30

// Globalne periferne enote
Adafruit_BME280 bme; // I2C

// Deljene spremenljivke med jedroma (Thread-safe)
std::atomic<float> current_sound_speed(343.9f);
std::atomic<float> current_temperature(20.0f);
std::atomic<float> current_humidity(50.0f);
std::atomic<float> current_pressure(101325.0f);

// Funkcije iz i2s_sampler.cpp
bool init_i2s_microphones(uint32_t sample_rate);
void deinit_i2s_microphones();
bool read_i2s_stereo(int32_t* left, int32_t* right, size_t count);

// Nastavitve sistema
static constexpr char DEFAULT_WIFI_SSID[] = "";
static constexpr char DEFAULT_WIFI_PASSWORD[] = "";
static constexpr char DEFAULT_API_URL[] = "";
static constexpr char DEFAULT_API_KEY[] = "";
static constexpr uint8_t MEASUREMENT_BATCH_VERSION = 1;
static constexpr size_t MEASUREMENT_BATCH_HEADER_SIZE = 12;
static constexpr size_t MEASUREMENT_RECORD_SIZE = 20;
static constexpr time_t MIN_VALID_EPOCH = 1704067200; // 2024-01-01 UTC
static constexpr uint16_t CLOCK_FLAG_UTC_SYNCHRONIZED = 0x0001;
static constexpr uint16_t CLOCK_FLAG_SERVER_SYNCHRONIZED = 0x0002;
static constexpr uint16_t CLOCK_FLAG_MONOTONIC_UPTIME = 0x0004;

struct DeviceConfig {
    String wifi_ssid;
    String wifi_password;
    String api_url;
    String api_key;
    uint32_t device_id = 0;
    float microphone_distance_m = 0.08f;
    float correlation_threshold = 0.35f;
    float sound_level_threshold_db_spl = 55.0f;
    float db_spl_offset = 0.0f;
    uint32_t sample_rate = 44100;
    uint32_t window_size = N_SAMPLES;
};

struct NetworkMeasurement {
    DoaResult result;
    uint64_t timestamp_ms;
    uint16_t clock_flags;
};

static GccPhat doa(44100, 0.08f, 20.0f); // 44.1 kHz, 8 cm, 20 °C
static QueueHandle_t display_queue;
static QueueHandle_t network_queue;
static SPIClass tft_spi(FSPI);
static Adafruit_ST7735 display(&tft_spi, TFT_CS_PIN, TFT_DC_PIN, TFT_RST_PIN);
static GFXcanvas16 canvas(128, 160);
static SemaphoreHandle_t doa_mutex;
static SemaphoreHandle_t config_mutex;
static SemaphoreHandle_t i2s_mutex;
static Preferences preferences;
static DeviceConfig device_config;
static std::atomic<float> db_spl_offset(0.0f);
static std::atomic<bool> server_available(false);
static std::atomic<int64_t> ntp_utc_offset_ms(0);
static std::atomic<bool> ntp_time_valid(false);
static std::atomic<int64_t> server_utc_offset_ms(0);
static std::atomic<bool> server_time_valid(false);
static bool display_initialized = false;
static bool bme_initialized = false;
static bool setup_mode = false;
static bool device_configured = false;
static DNSServer setup_dns;
static WebServer setup_server(80);
static std::atomic<uint32_t> dsp_deadline_misses(0);
static std::atomic<uint32_t> dsp_i2s_read_failures(0);
static std::atomic<uint32_t> display_queue_drops(0);
static std::atomic<uint32_t> network_queue_drops(0);
static std::atomic<uint32_t> dsp_blocks_processed(0);
static std::atomic<uint32_t> dsp_max_processing_us(0);

static bool is_supported_audio_config(uint32_t sample_rate, uint32_t window_size) {
    return (sample_rate == 44100 || sample_rate == 48000) &&
           (window_size == 512 || window_size == 1024 || window_size == 2048);
}

static uint32_t audio_block_duration_us(uint32_t sample_rate, uint32_t window_size) {
    return static_cast<uint32_t>(
        (static_cast<uint64_t>(window_size) * 1000000ULL) / sample_rate);
}

static String get_preference_string(const char *key, const char *fallback) {
    return preferences.isKey(key) ? preferences.getString(key) : String(fallback);
}

static float get_preference_float(const char *key, float fallback) {
    return preferences.isKey(key) ? preferences.getFloat(key) : fallback;
}

static void load_device_config() {
    preferences.begin("acoustic", false);

    device_config.wifi_ssid = get_preference_string("wifi_ssid", DEFAULT_WIFI_SSID);
    device_config.wifi_password =
        get_preference_string("wifi_pass", DEFAULT_WIFI_PASSWORD);
    device_config.api_url = get_preference_string("api_url", DEFAULT_API_URL);
    device_config.api_key = get_preference_string("api_key", DEFAULT_API_KEY);
    device_config.device_id = preferences.getUInt("device_id", 0);
    device_config.microphone_distance_m = get_preference_float("mic_distance", 0.08f);
    device_config.correlation_threshold =
        get_preference_float("corr_threshold", 0.35f);
    device_config.sound_level_threshold_db_spl =
        get_preference_float("spl_threshold", 55.0f);
    device_config.db_spl_offset = get_preference_float("spl_offset", 0.0f);
    device_config.sample_rate = preferences.getUInt("sample_rate", 44100);
    device_config.window_size = preferences.getUInt("window_size", N_SAMPLES);
    db_spl_offset.store(device_config.db_spl_offset);
    device_configured = device_config.wifi_ssid.length() > 0 &&
                        device_config.api_url.length() > 0 &&
                        device_config.api_key.length() > 0 &&
                        device_config.device_id != 0;
}

static void save_device_config() {
    preferences.putString("wifi_ssid", device_config.wifi_ssid);
    preferences.putString("wifi_pass", device_config.wifi_password);
    preferences.putString("api_url", device_config.api_url);
    preferences.putString("api_key", device_config.api_key);
    preferences.putUInt("device_id", device_config.device_id);
    preferences.putFloat("mic_distance", device_config.microphone_distance_m);
    preferences.putFloat("corr_threshold", device_config.correlation_threshold);
    preferences.putFloat("spl_threshold", device_config.sound_level_threshold_db_spl);
    preferences.putFloat("spl_offset", device_config.db_spl_offset);
    preferences.putUInt("sample_rate", device_config.sample_rate);
    preferences.putUInt("window_size", device_config.window_size);
}

static String setup_page() {
    String page;
    page.reserve(1800);
    page += "<!doctype html><html><head><meta name='viewport' content='width=device-width'>";
    page += "<title>Acoustic Locator Setup</title></head><body>";
    page += "<h1>Acoustic Locator Setup</h1><form method='POST' action='/save'>";
    page += "Wi-Fi SSID<br><input name='wifi_ssid' value='" + device_config.wifi_ssid + "'><br>";
    const String pass_hint = device_config.wifi_password.length() > 0 ? " (leave blank to retain stored)" : "";
    page += "Wi-Fi password" + pass_hint + "<br><input type='password' name='wifi_password'><br>";
    page += "API URL<br><input name='api_url' value='" + device_config.api_url + "'><br>";
    const String api_key_hint = device_config.api_key.length() > 0 ? " (leave blank to retain stored)" : "";
    page += "API key" + api_key_hint + "<br><input type='password' name='api_key'><br>";
    page += "Device ID<br><input type='number' name='device_id' value='" +
            String(device_config.device_id) + "'><br>";
    page += "Microphone distance (m)<br><input name='mic_distance' value='" +
            String(device_config.microphone_distance_m, 4) + "'><br>";
    page += "dB SPL offset<br><input name='spl_offset' value='" +
            String(device_config.db_spl_offset, 2) + "'><br><br>";
    page += "<button type='submit'>Save and restart</button></form></body></html>";
    return page;
}

static void handle_setup_root() {
    setup_server.send(200, "text/html", setup_page());
}

static void handle_setup_save() {
    if (!setup_server.hasArg("wifi_ssid") ||
        !setup_server.hasArg("api_url") ||
        !setup_server.hasArg("device_id") ||
        !setup_server.hasArg("mic_distance") ||
        !setup_server.hasArg("spl_offset")) {
        setup_server.send(400, "text/plain", "Missing required fields");
        return;
    }

    String wifi_password = setup_server.arg("wifi_password");
    String api_key = setup_server.arg("api_key");
    device_config.wifi_ssid = setup_server.arg("wifi_ssid");
    if (wifi_password.length() > 0) {
        device_config.wifi_password = wifi_password;
    }
    if (api_key.length() > 0) {
        device_config.api_key = api_key;
    }
    device_config.api_url = setup_server.arg("api_url");
    device_config.device_id = setup_server.arg("device_id").toInt();
    device_config.microphone_distance_m = setup_server.arg("mic_distance").toFloat();
    device_config.db_spl_offset = setup_server.arg("spl_offset").toFloat();

    if (device_config.wifi_ssid.length() == 0 ||
        device_config.api_url.length() == 0 ||
        device_config.api_key.length() == 0 ||
        device_config.device_id == 0 ||
        device_config.microphone_distance_m <= 0.0f) {
        setup_server.send(400, "text/plain", "Invalid configuration");
        return;
    }

    save_device_config();
    setup_server.send(200, "text/plain", "Saved. Restarting...");
    delay(500);
    ESP.restart();
}

static void redirect_to_setup() {
    setup_server.sendHeader(
        "Location", "http://" + WiFi.softAPIP().toString(), true);
    setup_server.send(302, "text/plain", "Redirecting to setup");
}

static void log_i2c_devices() {
    bool found_device = false;

    Serial.println("I2C scan:");
    for (uint8_t address = 1; address < 127; ++address) {
        Wire.beginTransmission(address);
        if (Wire.endTransmission() == 0) {
            Serial.printf("  Found device at 0x%02X\n", address);
            found_device = true;
        }
    }

    if (!found_device) {
        Serial.println("  No I2C devices found");
    }
}

static void show_boot_prompt(uint32_t remaining_ms, uint32_t total_ms) {
    if (!display_initialized) return;
    canvas.fillScreen(ST77XX_BLACK);

    // Naslov
    canvas.setTextSize(1);
    canvas.setTextColor(ST77XX_CYAN);
    canvas.setCursor(16, 18);
    canvas.println("ACOUSTIC LOCATOR");
    canvas.drawFastHLine(10, 30, 108, 0x7BEF);

    // Poziv za setup
    canvas.setTextColor(ST77XX_WHITE);
    canvas.setCursor(14, 46);
    canvas.println("Press BOOT button");
    canvas.setCursor(14, 58);
    canvas.println("to enter Setup");

    // Vrstica napredka
    const int bar_x = 14;
    const int bar_y = 82;
    const int bar_w = 100;
    const int bar_h = 8;
    canvas.drawRect(bar_x, bar_y, bar_w, bar_h, ST77XX_WHITE);
    int fill_w = (remaining_ms * (bar_w - 2)) / total_ms;
    if (fill_w > 0) {
        canvas.fillRect(bar_x + 1, bar_y + 1, fill_w, bar_h - 2, ST77XX_CYAN);
    }

    // Besedilo odštevanja
    canvas.setTextColor(0x7BEF);
    canvas.setCursor(14, 104);
    int sec = (remaining_ms + 999) / 1000;
    canvas.printf("Starting in %d s...", sec);

    display.drawRGBBitmap(0, 0, canvas.getBuffer(), 128, 160);
}

static void show_setup_portal_screen() {
    if (!display_initialized) return;
    canvas.fillScreen(ST77XX_BLACK);

    canvas.setTextSize(1);
    canvas.setTextColor(ST77XX_YELLOW);
    canvas.setCursor(16, 12);
    canvas.println("=== SETUP MODE ===");
    canvas.drawFastHLine(10, 24, 108, 0x7BEF);

    canvas.setTextColor(ST77XX_WHITE);
    canvas.setCursor(8, 34);
    canvas.println("Connect to Wi-Fi AP:");
    canvas.setTextColor(ST77XX_CYAN);
    canvas.setCursor(8, 46);
    canvas.println(SETUP_AP_NAME);

    canvas.setTextColor(ST77XX_WHITE);
    canvas.setCursor(8, 64);
    canvas.println("Password:");
    canvas.setTextColor(ST77XX_CYAN);
    canvas.setCursor(8, 76);
    canvas.println(SETUP_AP_PASSWORD);

    canvas.setTextColor(ST77XX_WHITE);
    canvas.setCursor(8, 94);
    canvas.println("Open in browser:");
    canvas.setTextColor(ST77XX_GREEN);
    canvas.setCursor(8, 106);
    canvas.println("http://192.168.4.1");

    canvas.setTextColor(0x7BEF);
    canvas.setCursor(8, 130);
    canvas.println("Save & reboot");
    canvas.setCursor(8, 142);
    canvas.println("when finished.");

    display.drawRGBBitmap(0, 0, canvas.getBuffer(), 128, 160);
}

static void enter_setup_mode() {
    setup_mode = true;
    show_setup_portal_screen();
    WiFi.mode(WIFI_AP);
    if (!WiFi.softAP(SETUP_AP_NAME, SETUP_AP_PASSWORD)) {
        Serial.println("Setup access point could not be started");
        while (true) {
            delay(1000);
        }
    }

    setup_dns.start(53, "*", WiFi.softAPIP());
    setup_server.on("/", HTTP_GET, handle_setup_root);
    setup_server.on("/save", HTTP_POST, handle_setup_save);
    setup_server.on("/generate_204", HTTP_GET, redirect_to_setup);
    setup_server.on("/hotspot-detect.html", HTTP_GET, redirect_to_setup);
    setup_server.on("/connecttest.txt", HTTP_GET, redirect_to_setup);
    setup_server.on("/ncsi.txt", HTTP_GET, redirect_to_setup);
    setup_server.onNotFound(redirect_to_setup);
    setup_server.begin();
    Serial.print("Setup AP active at ");
    Serial.println(WiFi.softAPIP());

    while (setup_mode) {
        setup_dns.processNextRequest();
        setup_server.handleClient();
        delay(2);
    }
}

static void check_setup_button() {
    pinMode(SETUP_BUTTON_PIN, INPUT_PULLUP);
    const uint32_t total_ms = SETUP_BOOT_DELAY_MS;
    const uint32_t start_ms = millis();
    bool button_pressed = false;

    while (millis() - start_ms < total_ms) {
        if (digitalRead(SETUP_BUTTON_PIN) == LOW) {
            button_pressed = true;
            break;
        }
        uint32_t elapsed = millis() - start_ms;
        uint32_t remaining = (elapsed < total_ms) ? (total_ms - elapsed) : 0;
        show_boot_prompt(remaining, total_ms);
        delay(40);
    }

    if (button_pressed) {
        enter_setup_mode();
    }
}

static bool enqueue_latest(QueueHandle_t queue, const DoaResult &result,
                           std::atomic<uint32_t> &drop_counter) {
    if (xQueueSend(queue, &result, 0) == pdPASS) {
        return true;
    }

    DoaResult discarded;
    if (xQueueReceive(queue, &discarded, 0) != pdPASS ||
        xQueueSend(queue, &result, 0) != pdPASS) {
        drop_counter.fetch_add(1);
        return false;
    }

    drop_counter.fetch_add(1);
    return true;
}

static bool enqueue_network_measurement(const NetworkMeasurement &measurement) {
    if (measurement.timestamp_ms == 0 || measurement.clock_flags == 0) {
        network_queue_drops.fetch_add(1);
        return false;
    }

    if (xQueueSend(network_queue, &measurement, 0) == pdPASS) {
        return true;
    }

    NetworkMeasurement discarded;
    if (xQueueReceive(network_queue, &discarded, 0) != pdPASS ||
        xQueueSend(network_queue, &measurement, 0) != pdPASS) {
        network_queue_drops.fetch_add(1);
        return false;
    }

    network_queue_drops.fetch_add(1);
    return true;
}

static NetworkMeasurement make_network_measurement(const DoaResult &result) {
    const int64_t monotonic_ms =
        static_cast<int64_t>(esp_timer_get_time() / 1000ULL);
    if (ntp_time_valid.load()) {
        const int64_t estimated_utc_ms =
            monotonic_ms + ntp_utc_offset_ms.load();
        if (estimated_utc_ms > 0) {
            return {
                result,
                static_cast<uint64_t>(estimated_utc_ms),
                CLOCK_FLAG_UTC_SYNCHRONIZED};
        }
    }

    if (server_time_valid.load()) {
        const int64_t estimated_utc_ms =
            monotonic_ms + server_utc_offset_ms.load();
        if (estimated_utc_ms > 0) {
            return {
                result,
                static_cast<uint64_t>(estimated_utc_ms),
                static_cast<uint16_t>(CLOCK_FLAG_UTC_SYNCHRONIZED |
                                      CLOCK_FLAG_SERVER_SYNCHRONIZED)};
        }
    }

    return {
        result,
        static_cast<uint64_t>(monotonic_ms),
        CLOCK_FLAG_MONOTONIC_UPTIME};
}

static void append_u16_le(uint8_t *buffer, size_t &offset, uint16_t value) {
    buffer[offset++] = static_cast<uint8_t>(value);
    buffer[offset++] = static_cast<uint8_t>(value >> 8);
}

static void append_u32_le(uint8_t *buffer, size_t &offset, uint32_t value) {
    for (unsigned int byte = 0; byte < sizeof(value); ++byte) {
        buffer[offset++] = static_cast<uint8_t>(value >> (byte * 8));
    }
}

static void append_u64_le(uint8_t *buffer, size_t &offset, uint64_t value) {
    for (unsigned int byte = 0; byte < sizeof(value); ++byte) {
        buffer[offset++] = static_cast<uint8_t>(value >> (byte * 8));
    }
}

static void append_float_le(uint8_t *buffer, size_t &offset, float value) {
    static_assert(sizeof(float) == sizeof(uint32_t), "Binary protocol requires 32-bit floats");
    uint32_t bits = 0;
    memcpy(&bits, &value, sizeof(bits));
    append_u32_le(buffer, offset, bits);
}

static size_t serialize_measurement_batch(
    const NetworkMeasurement *measurements, size_t count, uint8_t *payload,
    size_t payload_capacity) {
    if (measurements == nullptr || payload == nullptr || count == 0 ||
        count > NETWORK_BATCH_MAX_MEASUREMENTS ||
        payload_capacity < MEASUREMENT_BATCH_HEADER_SIZE +
                                count * MEASUREMENT_RECORD_SIZE) {
        return 0;
    }

    const uint64_t base_timestamp_ms = measurements[0].timestamp_ms;
    const uint16_t clock_flags = measurements[0].clock_flags;
    if (base_timestamp_ms == 0 || clock_flags == 0) {
        return 0;
    }

    size_t offset = 0;
    payload[offset++] = MEASUREMENT_BATCH_VERSION;
    payload[offset++] = static_cast<uint8_t>(count);
    append_u16_le(payload, offset, clock_flags);
    append_u64_le(payload, offset, base_timestamp_ms);

    for (size_t index = 0; index < count; ++index) {
        if (measurements[index].clock_flags != clock_flags ||
            measurements[index].timestamp_ms < base_timestamp_ms ||
            measurements[index].timestamp_ms - base_timestamp_ms >
                UINT32_MAX) {
            return 0;
        }

        append_u32_le(
            payload, offset,
            static_cast<uint32_t>(measurements[index].timestamp_ms -
                                  base_timestamp_ms));
        append_float_le(payload, offset, measurements[index].result.angle_deg);
        append_float_le(payload, offset, measurements[index].result.delay_us);
        append_float_le(
            payload, offset,
            measurements[index].result.rms_db + db_spl_offset.load());
        append_float_le(payload, offset, measurements[index].result.confidence);
    }

    return offset;
}

static void log_dsp_diagnostics() {
    static uint32_t last_log_ms = 0;
    const uint32_t now = millis();
    if (now - last_log_ms < DSP_DIAGNOSTIC_INTERVAL_MS) {
        return;
    }

    last_log_ms = now;

    uint32_t sample_rate;
    uint32_t window_size;
    if (xSemaphoreTake(config_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }
    sample_rate = device_config.sample_rate;
    window_size = device_config.window_size;
    xSemaphoreGive(config_mutex);

    Serial.printf(
        "DSP stats: blocks=%lu, deadline_misses=%lu, max_processing=%lu us, "
        "budget=%lu us, i2s_failures=%lu, display_drops=%lu, network_drops=%lu, "
        "stack_free=%u\n",
        static_cast<unsigned long>(dsp_blocks_processed.load()),
        static_cast<unsigned long>(dsp_deadline_misses.load()),
        static_cast<unsigned long>(dsp_max_processing_us.load()),
        static_cast<unsigned long>(audio_block_duration_us(sample_rate, window_size)),
        static_cast<unsigned long>(dsp_i2s_read_failures.load()),
        static_cast<unsigned long>(display_queue_drops.load()),
        static_cast<unsigned long>(network_queue_drops.load()),
        static_cast<unsigned int>(uxTaskGetStackHighWaterMark(nullptr)));
}

static void initialize_task_watchdog() {
    esp_err_t result = esp_task_wdt_init(TASK_WATCHDOG_TIMEOUT_SECONDS, false);
    if (result != ESP_OK && result != ESP_ERR_INVALID_STATE) {
        Serial.printf("Task watchdog initialization failed: %s\n",
                      esp_err_to_name(result));
    }
}

// --- NALOGA NA JEDRU 1: DSP OBDELAVA ---
void dsp_task(void *param) {
    static int32_t left_buf[DSP_MAX_WINDOW_SIZE];
    static int32_t right_buf[DSP_MAX_WINDOW_SIZE];
    esp_task_wdt_add(nullptr);

    while (true) {
        uint32_t window_size;
        uint32_t sample_rate;
        if (xSemaphoreTake(config_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
            esp_task_wdt_reset();
            continue;
        }
        sample_rate = device_config.sample_rate;
        window_size = device_config.window_size;
        xSemaphoreGive(config_mutex);

        bool read_ok = false;
        if (xSemaphoreTake(i2s_mutex, portMAX_DELAY) == pdTRUE) {
            read_ok = read_i2s_stereo(left_buf, right_buf, window_size);
            xSemaphoreGive(i2s_mutex);
        }
        if (read_ok) {
            const uint32_t process_start = micros();
            if (xSemaphoreTake(doa_mutex, portMAX_DELAY) != pdTRUE) {
                esp_task_wdt_reset();
                continue;
            }
            DoaResult res = doa.process(left_buf, right_buf);
            xSemaphoreGive(doa_mutex);
            const uint32_t processing_us = micros() - process_start;
            const uint32_t block_budget_us =
                audio_block_duration_us(sample_rate, window_size);
            dsp_blocks_processed.fetch_add(1);
            uint32_t previous_max = dsp_max_processing_us.load();
            while (processing_us > previous_max &&
                   !dsp_max_processing_us.compare_exchange_weak(
                       previous_max, processing_us)) {
            }
            if (processing_us > block_budget_us) {
                dsp_deadline_misses.fetch_add(1);
            }
            
            // Vsak potrošnik prejme svojo kopijo dogodka.
            if (res.valid) {
                enqueue_latest(display_queue, res, display_queue_drops);
                enqueue_network_measurement(make_network_measurement(res));
            }
        } else {
            dsp_i2s_read_failures.fetch_add(1);
        }
        log_dsp_diagnostics();
        esp_task_wdt_reset();
    }
}

static bool apply_dsp_calibration(float temperature_c,
                                  float humidity_pct = 50.0f,
                                  float pressure_pa = 101325.0f) {
    if (xSemaphoreTake(config_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        Serial.println("Could not lock device configuration");
        return false;
    }

    const float microphone_distance_m = device_config.microphone_distance_m;
    const float correlation_threshold = device_config.correlation_threshold;
    const float sound_level_threshold_db_spl =
        device_config.sound_level_threshold_db_spl;
    const float configured_offset = device_config.db_spl_offset;
    const uint32_t sample_rate = device_config.sample_rate;
    const uint32_t window_size = device_config.window_size;
    xSemaphoreGive(config_mutex);

    if (microphone_distance_m <= 0.0f ||
        correlation_threshold <= 0.0f ||
        sound_level_threshold_db_spl <= -200.0f ||
        !is_supported_audio_config(sample_rate, window_size)) {
        Serial.println("Invalid or unsupported remote calibration");
        return false;
    }

    if (xSemaphoreTake(doa_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        Serial.println("Could not lock DSP calibration");
        return false;
    }

    doa.updateCalibration(
        microphone_distance_m,
        temperature_c,
        humidity_pct,
        pressure_pa,
        correlation_threshold,
        sound_level_threshold_db_spl - configured_offset);
    xSemaphoreGive(doa_mutex);
    db_spl_offset.store(configured_offset);
    return true;
}

static bool reconfigure_audio(uint32_t sample_rate, uint32_t window_size) {
    if (!is_supported_audio_config(sample_rate, window_size)) {
        Serial.printf("Unsupported audio configuration: rate=%lu, window=%lu\n",
                      static_cast<unsigned long>(sample_rate),
                      static_cast<unsigned long>(window_size));
        return false;
    }

    uint32_t previous_rate;
    uint32_t previous_window;
    if (xSemaphoreTake(config_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        Serial.println("Could not lock configuration for audio reconfiguration");
        return false;
    }
    previous_rate = device_config.sample_rate;
    previous_window = device_config.window_size;
    xSemaphoreGive(config_mutex);

    if (xSemaphoreTake(doa_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        Serial.println("Could not lock DSP for audio reconfiguration");
        return false;
    }
    const bool dsp_ready = doa.configure(
        static_cast<float>(sample_rate), window_size);
    xSemaphoreGive(doa_mutex);
    if (!dsp_ready) {
        Serial.println("GCC-PHAT rejected audio reconfiguration");
        return false;
    }

    if (xSemaphoreTake(i2s_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        Serial.println("Could not lock I2S for audio reconfiguration");
        xSemaphoreTake(doa_mutex, portMAX_DELAY);
        doa.configure(static_cast<float>(previous_rate), previous_window);
        xSemaphoreGive(doa_mutex);
        return false;
    }
    deinit_i2s_microphones();
    const bool i2s_ready = init_i2s_microphones(sample_rate);
    xSemaphoreGive(i2s_mutex);
    if (i2s_ready) {
        return true;
    }

    Serial.println("I2S rejected audio reconfiguration; rolling back");
    xSemaphoreTake(doa_mutex, portMAX_DELAY);
    const bool dsp_rollback = doa.configure(
        static_cast<float>(previous_rate), previous_window);
    xSemaphoreGive(doa_mutex);
    if (xSemaphoreTake(i2s_mutex, portMAX_DELAY) == pdTRUE) {
        init_i2s_microphones(previous_rate);
        xSemaphoreGive(i2s_mutex);
    }
    return dsp_rollback;
}

static bool fetch_remote_calibration() {
    if (device_config.device_id == 0 || WiFi.status() != WL_CONNECTED) {
        return false;
    }

    HTTPClient http;
    String url = device_config.api_url + "/naprave/" +
                 String(device_config.device_id) + "/kalibracija";
    if (!http.begin(url)) {
        Serial.println("Calibration HTTP client initialization failed");
        return false;
    }

    http.setTimeout(5000);
    http.addHeader("Accept", "application/json");
    http.addHeader("x-api-key", device_config.api_key);

    const int64_t request_start_ms =
        static_cast<int64_t>(esp_timer_get_time() / 1000ULL);
    int status = http.GET();
    String response = http.getString();
    const int64_t request_end_ms =
        static_cast<int64_t>(esp_timer_get_time() / 1000ULL);
    http.end();

    if (status < 200 || status >= 300) {
        Serial.printf("Calibration GET failed: %d, response: %s\n",
                      status, response.c_str());
        return false;
    }

    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, response);
    if (error) {
        Serial.printf("Calibration response JSON error: %s\n", error.c_str());
        return false;
    }

    JsonVariant server_time_value = doc["server_time_unix_ms"];
    if (!server_time_value.isNull()) {
        const uint64_t server_time_ms =
            server_time_value.as<uint64_t>();
        const int64_t midpoint_ms =
            request_start_ms + (request_end_ms - request_start_ms) / 2;
        if (server_time_ms >=
                static_cast<uint64_t>(MIN_VALID_EPOCH) * 1000ULL &&
            midpoint_ms >= 0) {
            server_utc_offset_ms.store(
                static_cast<int64_t>(server_time_ms) - midpoint_ms);
            server_time_valid.store(true);
        } else {
            Serial.println("Calibration server time is invalid");
        }
    }

    float distance = doc["razdalja_mikrofonov_m"] | 0.0f;
    float correlation = doc["prag_korelacije"] | 0.0f;
    float spl_threshold = doc["prag_glasnosti_db"] | 0.0f;
    uint32_t sample_rate = doc["vzorcevalna_frekvenca"] | 0U;
    uint32_t window_size = doc["okno_velikost"] | 0U;
    float offset = doc["db_spl_offset"] | db_spl_offset.load();

    if (distance <= 0.0f || correlation <= 0.0f ||
        spl_threshold <= -200.0f ||
        !is_supported_audio_config(sample_rate, window_size) ||
        !isfinite(distance) || !isfinite(correlation) ||
        !isfinite(spl_threshold) || !isfinite(offset)) {
        Serial.println("Calibration response contains invalid or unsupported values");
        return false;
    }

    if (xSemaphoreTake(config_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        Serial.println("Could not lock device configuration");
        return false;
    }
    DeviceConfig previous = device_config;
    xSemaphoreGive(config_mutex);

    const bool audio_config_changed =
        sample_rate != previous.sample_rate || window_size != previous.window_size;
    if (audio_config_changed &&
        !reconfigure_audio(sample_rate, window_size)) {
        return false;
    }

    if (xSemaphoreTake(config_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        Serial.println("Could not lock device configuration for update");
        reconfigure_audio(previous.sample_rate, previous.window_size);
        return false;
    }
    device_config.microphone_distance_m = distance;
    device_config.correlation_threshold = correlation;
    device_config.sound_level_threshold_db_spl = spl_threshold;
    device_config.db_spl_offset = offset;
    device_config.sample_rate = sample_rate;
    device_config.window_size = window_size;
    xSemaphoreGive(config_mutex);

    if (!apply_dsp_calibration(current_temperature.load(),
                               current_humidity.load(),
                               current_pressure.load())) {
        xSemaphoreTake(config_mutex, portMAX_DELAY);
        device_config = previous;
        xSemaphoreGive(config_mutex);
        reconfigure_audio(previous.sample_rate, previous.window_size);
        return false;
    }

    save_device_config();
    Serial.println("Remote calibration applied");
    return true;
}

// --- OMREŽNI DEL ---
static bool send_batch_to_backend(HTTPClient &http, bool &http_initialized,
                                  const NetworkMeasurement *measurements,
                                  size_t count) {
    if (WiFi.status() != WL_CONNECTED || device_config.api_url.isEmpty() ||
        device_config.api_key.isEmpty()) {
        server_available.store(false);
        return false;
    }

    if (!http_initialized) {
        String url = device_config.api_url + "/meritve";
        if (!http.begin(url)) {
            Serial.println("HTTP client initialization failed");
            server_available.store(false);
            return false;
        }
        http.setReuse(true);
        http.useHTTP10(false);
        http.setTimeout(5000);
        http.addHeader("Content-Type", "application/octet-stream");
        http.addHeader("Connection", "keep-alive");
        http.addHeader("x-api-key", device_config.api_key);
        http_initialized = true;
    }

    constexpr size_t max_payload_size =
        MEASUREMENT_BATCH_HEADER_SIZE +
        NETWORK_BATCH_MAX_MEASUREMENTS * MEASUREMENT_RECORD_SIZE;
    uint8_t payload[max_payload_size];
    const size_t payload_size = serialize_measurement_batch(
        measurements, count, payload, sizeof(payload));
    if (payload_size == 0) {
        Serial.println("Measurement batch serialization failed");
        server_available.store(false);
        return false;
    }

    const int httpResponseCode = http.POST(payload, payload_size);

    if (httpResponseCode < 200 || httpResponseCode >= 300) {
        const String response = http.getString();
        Serial.printf("HTTP POST /meritve failed: %d, response: %s\n",
                      httpResponseCode, response.c_str());
        http.end();
        http_initialized = false;
        server_available.store(false);
        return false;
    }

    server_available.store(true);
    Serial.printf("Measurement batch accepted: HTTP %d, count: %u\n",
                  httpResponseCode, static_cast<unsigned>(count));
    return true;
}

void network_task(void *param) {
    NetworkMeasurement batch[NETWORK_BATCH_MAX_MEASUREMENTS];
    unsigned long last_calibration_refresh = 0;
    unsigned long last_request_ms = 0;
    HTTPClient http;
    bool http_initialized = false;
    bool ntp_started = false;
    esp_task_wdt_add(nullptr);

    while (true) {
        if (WiFi.status() != WL_CONNECTED) {
            server_available.store(false);
            ntp_started = false;
            ntp_time_valid.store(false);
            if (http_initialized) {
                http.end();
                http_initialized = false;
            }
            WiFi.reconnect();
            vTaskDelay(pdMS_TO_TICKS(1000));
            esp_task_wdt_reset();
            continue;
        }

        if (!ntp_started && time(nullptr) < MIN_VALID_EPOCH) {
            configTime(0, 0, "pool.ntp.org", "time.nist.gov");
            ntp_started = true;
        }
        if (!ntp_time_valid.load() && time(nullptr) >= MIN_VALID_EPOCH) {
            const int64_t monotonic_ms =
                static_cast<int64_t>(esp_timer_get_time() / 1000ULL);
            struct timeval time_value;
            gettimeofday(&time_value, nullptr);
            const int64_t utc_ms =
                static_cast<int64_t>(time_value.tv_sec) * 1000LL +
                static_cast<int64_t>(time_value.tv_usec) / 1000LL;
            if (utc_ms >=
                static_cast<int64_t>(MIN_VALID_EPOCH) * 1000LL) {
                ntp_utc_offset_ms.store(utc_ms - monotonic_ms);
                ntp_time_valid.store(true);
            }
        }
        if (last_calibration_refresh == 0 ||
            millis() - last_calibration_refresh >= CONFIG_REFRESH_INTERVAL_MS) {
            fetch_remote_calibration();
            last_calibration_refresh = millis();
        }

        size_t batch_count = 0;
        if (xQueueReceive(network_queue, &batch[batch_count],
                          pdMS_TO_TICKS(100)) == pdTRUE) {
            ++batch_count;
            const unsigned long flush_deadline =
                millis() + NETWORK_BATCH_FLUSH_INTERVAL_MS;
            while (batch_count < NETWORK_BATCH_MAX_MEASUREMENTS) {
                const unsigned long now = millis();
                if (now >= flush_deadline) {
                    break;
                }
                NetworkMeasurement next_measurement;
                if (xQueuePeek(network_queue, &next_measurement, 0) != pdTRUE ||
                    next_measurement.clock_flags != batch[0].clock_flags) {
                    break;
                }
                const TickType_t wait_ticks = pdMS_TO_TICKS(flush_deadline - now);
                if (xQueueReceive(network_queue, &batch[batch_count],
                                  wait_ticks) != pdTRUE) {
                    break;
                }
                ++batch_count;
            }

            const unsigned long now = millis();
            if (last_request_ms != 0 &&
                now - last_request_ms < NETWORK_MAX_REQUEST_INTERVAL_MS) {
                vTaskDelay(pdMS_TO_TICKS(
                    NETWORK_MAX_REQUEST_INTERVAL_MS - (now - last_request_ms)));
            }

            bool sent = false;
            for (int attempt = 1; attempt <= HTTP_RETRY_COUNT && !sent; ++attempt) {
                sent = send_batch_to_backend(http, http_initialized, batch,
                                             batch_count);
                if (!sent && attempt < HTTP_RETRY_COUNT) {
                    vTaskDelay(pdMS_TO_TICKS(1000 * attempt));
                }
            }
            if (!sent) {
                Serial.println("Measurement batch dropped after RAM retry limit");
            } else {
                last_request_ms = millis();
            }
        }
        esp_task_wdt_reset();
    }
}

void draw_radar_ui(float angle_deg, float temp_c, float hum, float pres, float c_sound,
                   float db_spl, bool api_available) {
    canvas.fillScreen(ST77XX_BLACK);

    // 1. Izris polkrožnega radarja (središče na x=64, y=75, polmer r=52)
    const int cx = 64;
    const int cy = 68;
    const int r = 52;

    // Barvna mreža radarja (temno zelena: 0x03E0)
    canvas.drawCircle(cx, cy, r, 0x03E0);
    canvas.drawCircle(cx, cy, r / 2, 0x03E0);
    // Prekrijemo spodnjo polovico kroga
    canvas.fillRect(0, cy + 1, 128, r + 2, ST77XX_BLACK);
    canvas.drawLine(cx - r, cy, cx + r, cy, 0x03E0);

    // Oznake kotov
    canvas.setTextSize(1);
    canvas.setTextColor(ST77XX_WHITE);
    canvas.setCursor(4, cy - 10);
    canvas.print("-90");
    canvas.setCursor(cx - 6, cy - r - 10);
    canvas.print("0");
    canvas.setCursor(104, cy - 10);
    canvas.print("+90");

    // 2. Izris igle (barva glede na glasnost: rdeča > 75 dB SPL, rumena > 60 dB SPL, sicer zelena)
    uint16_t needle_color = (db_spl > 75.0f) ? ST77XX_RED : ((db_spl > 60.0f) ? ST77XX_YELLOW : ST77XX_GREEN);
    float rad = (angle_deg - 90.0f) * (M_PI / 180.0f);
    int target_x = cx + static_cast<int>(r * cosf(rad));
    int target_y = cy + static_cast<int>(r * sinf(rad));

    canvas.drawLine(cx, cy, target_x, target_y, needle_color);
    canvas.fillCircle(target_x, target_y, 3, needle_color);

    // 3. Spodnja nadzorna plošča s telemetrijo (y = 75 do 159)
    canvas.drawFastHLine(0, 75, 128, 0x7BEF); // Siva ločilna črta

    canvas.setTextSize(2);
    canvas.setTextColor(ST77XX_CYAN);
    canvas.setCursor(14, 83);
    canvas.printf("%+5.1f deg", angle_deg);

    canvas.setTextSize(1);
    canvas.setTextColor(ST77XX_YELLOW);
    canvas.setCursor(13, 108);
    canvas.printf("%.1fdB   %.2fm/s", db_spl, c_sound);

    canvas.setTextColor(ST77XX_WHITE);
    canvas.setCursor(6, 123);
    canvas.printf("T: %.1fC   RH: %.0f%%", temp_c, hum);

    canvas.setCursor(6, 137);
    canvas.printf("P: %.1f hPa", pres / 100.0f);

    int status_y = 151; // Središče krožca (višina vrstice)

    if (api_available) {
        // Poln zelen krožec (polmer 3 px)
        canvas.fillCircle(8, status_y, 3, ST77XX_GREEN);
        
        canvas.setTextColor(ST77XX_GREEN);
        canvas.setCursor(16, 148);
        canvas.print("API ONLINE");
    } else {
        // Prazen rdeč krožec (obroba)
        canvas.drawCircle(8, status_y, 3, ST77XX_RED);
        
        canvas.setTextColor(ST77XX_RED);
        canvas.setCursor(16, 148);
        canvas.print("API OFFLINE");
    }

    // Pošlji celotno sliko na zaslon v enem kosu prek SPI
    display.drawRGBBitmap(0, 0, canvas.getBuffer(), 128, 160);
}

void init_sensors_and_display() {
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 400000); // 400 kHz Fast I2C za BME280

    // 1. Zagon SPI TFT zaslona KMR-1.8 (ST7735)
    tft_spi.begin(TFT_SCLK_PIN, -1, TFT_MOSI_PIN, -1);
    display.initR(INITR_BLACKTAB); // Če so barve obrnjene ali odrezane, INITR_GREENTAB / INITR_18BLACKTAB
    display.setSPISpeed(40000000); // 40 MHz SPI
    display.setRotation(0);        // Pokončni način 128x160
    if (TFT_INVERT_COLORS) {
        display.invertDisplay(true);
    }
    display_initialized = true;

    canvas.fillScreen(ST77XX_BLACK);
    canvas.setTextSize(1);
    canvas.setTextColor(ST77XX_WHITE);
    canvas.setCursor(10, 30);
    canvas.println("Inicializacija...");
    display.drawRGBBitmap(0, 0, canvas.getBuffer(), 128, 160);

    // 2. Zagon BME280
    bme_initialized = bme.begin(0x76, &Wire);
    if (!bme_initialized) {
        // Če ni na 0x76, poskusi še privzeti 0x77
        bme_initialized = bme.begin(0x77, &Wire);
        if (!bme_initialized) {
            Serial.println("BME280 senzor ni bil najden! Uporabljam privzeto c=343.9 m/s");
            log_i2c_devices();
        }
    }

    if (bme_initialized) {
        float t = bme.readTemperature();
        float rh = bme.readHumidity();
        float p = bme.readPressure();
        if (std::isfinite(t) && std::isfinite(rh) && std::isfinite(p)) {
            current_temperature.store(t);
            current_humidity.store(rh);
            current_pressure.store(p);
            current_sound_speed.store(GccPhat::calculateSoundSpeed(t, rh, p));
        }
    }
}

// Periodično branje BME280
void update_environmental_compensation() {
    static unsigned long last_read = 0;
    if (millis() - last_read > 2000) { // Vsaki 2 sekundi
        last_read = millis();

        if (!bme_initialized) {
            return;
        }

        float t = bme.readTemperature();      // °C
        float rh = bme.readHumidity();        // %
        float p = bme.readPressure();         // Pa

        if (std::isfinite(t) && std::isfinite(rh) && std::isfinite(p)) {
            current_temperature.store(t);
            current_humidity.store(rh);
            current_pressure.store(p);

            float c = GccPhat::calculateSoundSpeed(t, rh, p);
            current_sound_speed.store(c);

            // Dinamično posodobimo hitrost zvoka v DSP razredu
            apply_dsp_calibration(t, rh, p);
        }
    }
}

void sensor_display_task(void *param) {
    DoaResult event;
    DoaResult last_event = {0.0f, 0.0f, 0.0f, -100.0f, false};
    bool last_server_available = server_available.load();

    while (true) {
        update_environmental_compensation();

        if (xQueueReceive(display_queue, &event, pdMS_TO_TICKS(100))) {
            last_event = event;
            float temp = current_temperature.load();
            float speed = current_sound_speed.load();
            float humidity = current_humidity.load();
            float pressure = current_pressure.load();
            const float calibrated_db = event.rms_db + db_spl_offset.load();

            Serial.printf("Zaznan kot: %.1f deg, R: %.2f, dB: %.1f\n",
                          event.angle_deg, event.confidence,
                          calibrated_db);

            if (display_initialized) {
                last_server_available = server_available.load();
                draw_radar_ui(event.angle_deg, temp, humidity, pressure, speed,
                              calibrated_db, last_server_available);
            }
        }

        const bool current_server_available = server_available.load();
        if (display_initialized &&
            current_server_available != last_server_available) {
            last_server_available = current_server_available;
            const float calibrated_db = last_event.rms_db + db_spl_offset.load();
            draw_radar_ui(last_event.angle_deg,
                          current_temperature.load(), current_humidity.load(),
                          current_pressure.load(), current_sound_speed.load(),
                          calibrated_db, last_server_available);
        }
    }
}



void setup() {
    Serial.begin(115200);
    Serial.printf("PSRAM: %s, size=%u bytes, free=%u bytes\n",
                  psramFound() ? "found" : "not found",
                  ESP.getPsramSize(),
                  ESP.getFreePsram());
    load_device_config();

    // 1. Zagon zaslona in BME280 senzorja
    init_sensors_and_display();

    // 2. Preveri gumb za nastavitve (prikaze poziv in odstevalnik na zaslonu)
    check_setup_button();
    if (!device_configured) {
        Serial.println(
            "Device is not configured. Boot normally, then press and hold GPIO 0 during startup.");
    }

    // 3. Po izhodu iz boot okna narisi zacetni radar UI pred zagonom nalog
    if (display_initialized) {
        draw_radar_ui(0.0f, current_temperature.load(), 
                      current_humidity.load(),
                      current_pressure.load(),
                      current_sound_speed.load(), 0.0f,
                      server_available.load());
    }

    // Ustvari FreeRTOS objekte za prenos rezultatov in zaščito DSP konfiguracije
    display_queue = xQueueCreate(10, sizeof(DoaResult));
    network_queue = xQueueCreate(NETWORK_QUEUE_LENGTH, sizeof(NetworkMeasurement));
    doa_mutex = xSemaphoreCreateMutex();
    config_mutex = xSemaphoreCreateMutex();
    i2s_mutex = xSemaphoreCreateMutex();
    if (display_queue == nullptr || network_queue == nullptr ||
        doa_mutex == nullptr || config_mutex == nullptr || i2s_mutex == nullptr) {
        Serial.println("FreeRTOS objektov ni bilo mogoce ustvariti!");
        while (true) {
            delay(1000);
        }
    }

    if (!is_supported_audio_config(device_config.sample_rate,
                                   device_config.window_size)) {
        Serial.println("Unsupported saved audio configuration; using 44100 Hz / 1024 samples");
        device_config.sample_rate = 44100;
        device_config.window_size = 1024;
    }
    if (!doa.configure(static_cast<float>(device_config.sample_rate),
                       device_config.window_size)) {
        Serial.println("GCC-PHAT inicializacija ni uspela!");
        while (true) {
            delay(1000);
        }
    }
    if (!init_i2s_microphones(device_config.sample_rate)) {
        Serial.println("I2S inicializacija ni uspela!");
        while (true) {
            delay(1000);
        }
    }

    // 3. Povezava na Wi-Fi (Arduino knjižnica)
    if (device_configured) {
        WiFi.setAutoReconnect(true);
        WiFi.begin(
            device_config.wifi_ssid.c_str(), device_config.wifi_password.c_str());
    }

    if (!apply_dsp_calibration(current_temperature.load(),
                               current_humidity.load(),
                               current_pressure.load())) {
        Serial.println("Initial DSP calibration is invalid");
        while (true) {
            delay(1000);
        }
    }
    initialize_task_watchdog();

    // Zaženi DSP nalogo na Jedru 1 (prioriteta 5 - visoka)
    if (xTaskCreatePinnedToCore(dsp_task, "DSP_Task", 8192, NULL, 5, NULL, 1) != pdPASS) {
        Serial.println("DSP naloge ni bilo mogoce zagnati!");
        while (true) {
            delay(1000);
        }
    }

    if (xTaskCreatePinnedToCore(sensor_display_task, "Sensor_Display", 4096,
                                NULL, 2, NULL, 0) != pdPASS) {
        Serial.println("Sensor/display naloge ni bilo mogoce zagnati!");
        while (true) {
            delay(1000);
        }
    }

    if (xTaskCreatePinnedToCore(network_task, "Network_Task", 8192,
                                NULL, 2, NULL, 0) != pdPASS) {
        Serial.println("Omrezne naloge ni bilo mogoce zagnati!");
        while (true) {
            delay(1000);
        }
    }
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}