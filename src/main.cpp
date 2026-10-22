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

#include "app_config.h"
#include "gcc_phat.h"
#include "display_filter.h"
#include "post_checker.h"
#include "i2s_sampler.h"

// Globalne periferne enote
Adafruit_BME280 bme; // I2C

// Deljene spremenljivke med jedroma (Thread-safe)
std::atomic<float> current_sound_speed(DEFAULT_SOUND_SPEED_MPS);
std::atomic<float> current_temperature(DEFAULT_TEMPERATURE_C);
std::atomic<float> current_humidity(DEFAULT_HUMIDITY_PCT);
std::atomic<float> current_pressure(DEFAULT_PRESSURE_PA);

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
    String wifi_ssid                    = DEFAULT_WIFI_SSID;
    String wifi_password                = DEFAULT_WIFI_PASSWORD;
    String api_url                      = DEFAULT_API_URL;
    String api_key                      = DEFAULT_API_KEY;
    uint32_t device_id                  = 0;
    float microphone_distance_m         = DEFAULT_MIC_DISTANCE_M;
    float correlation_threshold         = DEFAULT_CORRELATION_THRESHOLD;
    float sound_level_threshold_db_spl  = DEFAULT_SOUND_LEVEL_THRESHOLD_DB_SPL;
    float db_spl_offset                 = DEFAULT_DB_SPL_OFFSET;
    float angle_offset_deg              = DEFAULT_ANGLE_OFFSET_DEG;
    uint32_t sample_rate                = DEFAULT_SAMPLE_RATE;
    uint32_t window_size                = DEFAULT_WINDOW_SIZE;
    bool adaptive_threshold_enabled     = DEFAULT_ADAPTIVE_THRESHOLD_ENABLED;
    float snr_margin_db                 = DEFAULT_SNR_MARGIN_DB;
};

struct NetworkMeasurement {
    DoaResult result;
    uint64_t timestamp_ms;
    uint16_t clock_flags;
};

static GccPhat doa(DEFAULT_SAMPLE_RATE, DEFAULT_MIC_DISTANCE_M, DEFAULT_TEMPERATURE_C);
static QueueHandle_t display_queue;
static QueueHandle_t network_queue;
static SPIClass tft_spi(FSPI);
static Adafruit_ST7735 display(&tft_spi, TFT_CS_PIN, TFT_DC_PIN, TFT_RST_PIN);
static GFXcanvas16 canvas(TFT_WIDTH, TFT_HEIGHT);
static SemaphoreHandle_t doa_mutex;
static SemaphoreHandle_t config_mutex;
static SemaphoreHandle_t i2s_mutex;
static Preferences preferences;
static DeviceConfig device_config;
static std::atomic<float> db_spl_offset(DEFAULT_DB_SPL_OFFSET);
static std::atomic<bool> server_available(false);
static std::atomic<int64_t> ntp_utc_offset_ms(0);
static std::atomic<bool> ntp_time_valid(false);
static std::atomic<int64_t> server_utc_offset_ms(0);
static std::atomic<bool> server_time_valid(false);
static std::atomic<float> current_noise_floor_db(-100.0f);
static std::atomic<bool> adaptive_threshold_active(DEFAULT_ADAPTIVE_THRESHOLD_ENABLED);
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

    device_config.wifi_ssid = get_preference_string("wifi_ssid", device_config.wifi_ssid.c_str());
    device_config.wifi_password =
        get_preference_string("wifi_pass", device_config.wifi_password.c_str());
    device_config.api_url = get_preference_string("api_url", device_config.api_url.c_str());
    device_config.api_key = get_preference_string("api_key", device_config.api_key.c_str());
    device_config.device_id = preferences.getUInt("device_id", device_config.device_id);
    device_config.microphone_distance_m =
        get_preference_float("mic_distance", device_config.microphone_distance_m);
    device_config.correlation_threshold =
        get_preference_float("corr_threshold", device_config.correlation_threshold);
    device_config.sound_level_threshold_db_spl =
        get_preference_float("spl_threshold", device_config.sound_level_threshold_db_spl);
    device_config.db_spl_offset =
        get_preference_float("spl_offset", device_config.db_spl_offset);
    device_config.angle_offset_deg =
        get_preference_float("ang_offset", device_config.angle_offset_deg);
    if (!std::isfinite(device_config.angle_offset_deg) ||
        device_config.angle_offset_deg < ANGLE_OFFSET_MIN_DEG ||
        device_config.angle_offset_deg > ANGLE_OFFSET_MAX_DEG) {
        device_config.angle_offset_deg = DEFAULT_ANGLE_OFFSET_DEG;
    }
    device_config.sample_rate = preferences.getUInt("sample_rate", device_config.sample_rate);
    device_config.window_size = preferences.getUInt("window_size", device_config.window_size);
    device_config.adaptive_threshold_enabled =
        preferences.isKey("adapt_thresh") ? preferences.getBool("adapt_thresh") : device_config.adaptive_threshold_enabled;
    device_config.snr_margin_db =
        get_preference_float("snr_margin", device_config.snr_margin_db);
    if (!std::isfinite(device_config.snr_margin_db) ||
        device_config.snr_margin_db < SNR_MARGIN_MIN_DB ||
        device_config.snr_margin_db > SNR_MARGIN_MAX_DB) {
        device_config.snr_margin_db = DEFAULT_SNR_MARGIN_DB;
    }
    db_spl_offset.store(device_config.db_spl_offset);
    adaptive_threshold_active.store(device_config.adaptive_threshold_enabled);
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
    preferences.putFloat("ang_offset", device_config.angle_offset_deg);
    preferences.putUInt("sample_rate", device_config.sample_rate);
    preferences.putUInt("window_size", device_config.window_size);
    preferences.putBool("adapt_thresh", device_config.adaptive_threshold_enabled);
    preferences.putFloat("snr_margin", device_config.snr_margin_db);
}

static String setup_page() {
    String page;
    page.reserve(2400);
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
            String(device_config.db_spl_offset, 2) + "'><br>";
    page += "Fixed threshold (dB SPL)<br><input name='spl_threshold' value='" +
            String(device_config.sound_level_threshold_db_spl, 1) + "'><br>";
    page += "Zero angle offset (deg)<br><input name='ang_offset' value='" +
            String(device_config.angle_offset_deg, 1) + "'><br>";
    page += "<label><input type='checkbox' name='adapt_thresh' value='1'" +
            String(device_config.adaptive_threshold_enabled ? " checked" : "") +
            "> Enable adaptive noise floor threshold</label><br>";
    page += "SNR margin (dB)<br><input name='snr_margin' value='" +
            String(device_config.snr_margin_db, 1) + "'><br><br>";
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

    device_config.adaptive_threshold_enabled = setup_server.hasArg("adapt_thresh");
    if (setup_server.hasArg("snr_margin")) {
        float snr = setup_server.arg("snr_margin").toFloat();
        if (std::isfinite(snr) && snr >= SNR_MARGIN_MIN_DB && snr <= SNR_MARGIN_MAX_DB) {
            device_config.snr_margin_db = snr;
        } else {
            setup_server.send(400, "text/plain", "Invalid SNR margin (must be 0-60 dB)");
            return;
        }
    }
    if (setup_server.hasArg("spl_threshold")) {
        float spl = setup_server.arg("spl_threshold").toFloat();
        if (std::isfinite(spl) && spl >= SPL_THRESHOLD_MIN_DB && spl <= SPL_THRESHOLD_MAX_DB) {
            device_config.sound_level_threshold_db_spl = spl;
        }
    }
    if (setup_server.hasArg("ang_offset")) {
        float ang = setup_server.arg("ang_offset").toFloat();
        if (std::isfinite(ang) && ang >= ANGLE_OFFSET_MIN_DEG && ang <= ANGLE_OFFSET_MAX_DEG) {
            device_config.angle_offset_deg = ang;
        } else {
            setup_server.send(400, "text/plain", "Invalid angle offset (must be -180 to +180 deg)");
            return;
        }
    }

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

struct BootSelfTestResult {
    bool psram_ok = false;
    size_t psram_bytes = 0;
    bool bme_ok = false;
    float bme_temp_c = 0.0f;
    float bme_pres_hpa = 0.0f;
    bool i2s_driver_ok = false;
    MicrophoneHealthResult mic_health{};
    bool all_nominal = false;
};

static BootSelfTestResult run_boot_self_test(uint32_t sample_rate) {
    BootSelfTestResult res;
    res.psram_ok = psramFound();
    res.psram_bytes = ESP.getPsramSize();

    res.bme_ok = bme_initialized;
    res.bme_temp_c = current_temperature.load();
    res.bme_pres_hpa = current_pressure.load() / 100.0f;

    res.i2s_driver_ok = init_i2s_microphones(sample_rate);
    if (res.i2s_driver_ok) {
        static int32_t ch0_test[POST_SAMPLE_COUNT];
        static int32_t ch1_test[POST_SAMPLE_COUNT];
        static int32_t ch2_test[POST_SAMPLE_COUNT];
        static int32_t ch3_test[POST_SAMPLE_COUNT];
        if (read_i2s_quad(ch0_test, ch1_test, ch2_test, ch3_test, POST_SAMPLE_COUNT)) {
            res.mic_health = evaluate_quad_microphone_health(
                ch0_test, ch1_test, ch2_test, ch3_test, POST_SAMPLE_COUNT);
        }
        deinit_i2s_microphones();
    }

    res.all_nominal = res.psram_ok && res.i2s_driver_ok && res.mic_health.all_ok;

    Serial.println("\n==================================================");
    Serial.println("           ACOUSTIC LOCATOR BOOT SELF-TEST        ");
    Serial.println("==================================================");
    Serial.printf("[%s] Octal PSRAM : %s (%u KB)\n",
                  res.psram_ok ? "PASS" : "FAIL",
                  res.psram_ok ? "Found" : "Missing",
                  static_cast<unsigned>(res.psram_bytes / 1024));

    if (res.bme_ok) {
        Serial.printf("[PASS] BME280 I2C  : OK (T: %.1fC, P: %.1f hPa)\n",
                      res.bme_temp_c, res.bme_pres_hpa);
    } else {
        Serial.printf("[WARN] BME280 I2C  : NOT FOUND (Using default c=%.1f m/s)\n",
                      DEFAULT_SOUND_SPEED_MPS);
    }

    Serial.printf("[%s] I2S Driver  : %s (%u Hz Dual-Bus Quad)\n",
                  res.i2s_driver_ok ? "PASS" : "FAIL",
                  res.i2s_driver_ok ? "Initialized (I2S0 Master + I2S1 Slave)" : "Failed",
                  static_cast<unsigned>(sample_rate));

    static const char* const mic_labels[AUDIO_NUM_CHANNELS] = {
        "M0 (RB/I2S0L)",
        "M1 (RF/I2S0R)",
        "M2 (LF/I2S1L)",
        "M3 (LB/I2S1R)"
    };
    for (size_t c = 0; c < AUDIO_NUM_CHANNELS; ++c) {
        const bool ok = res.mic_health.ch_ok[c];
        const char* state = ok
            ? "ACTIVE"
            : (res.mic_health.ch_min[c] == res.mic_health.ch_max[c] ? "FLATLINE" : "OUT OF BOUNDS");
        Serial.printf("[%s] %-13s: %s (RMS: %.1f dBFS, min=%ld, max=%ld)\n",
                      ok ? "PASS" : "FAIL",
                      mic_labels[c],
                      state,
                      res.mic_health.ch_rms_db[c],
                      static_cast<long>(res.mic_health.ch_min[c]),
                      static_cast<long>(res.mic_health.ch_max[c]));
    }

    const bool sync_ok = res.i2s_driver_ok &&
                         res.mic_health.bus0_distinct &&
                         res.mic_health.bus1_distinct &&
                         res.mic_health.cross_bus_distinct;
    Serial.printf("[%s] Bus Sync    : Bus0=%s, Bus1=%s, CrossBus=%s (max identical=%u/%u)\n",
                  sync_ok ? "PASS" : "FAIL",
                  res.mic_health.bus0_distinct ? "OK" : "COLLISION",
                  res.mic_health.bus1_distinct ? "OK" : "COLLISION",
                  res.mic_health.cross_bus_distinct ? "OK" : "SHORTED",
                  static_cast<unsigned>(res.mic_health.identical_samples),
                  static_cast<unsigned>(POST_SAMPLE_COUNT));

    Serial.println("==================================================");
    Serial.printf("STATUS: %s\n", res.all_nominal ? "ALL SYSTEMS NOMINAL" : "HARDWARE DEGRADED - CHECK WIRING");
    Serial.println("==================================================\n");

    return res;
}

static void show_boot_prompt(uint32_t remaining_ms, uint32_t total_ms, const BootSelfTestResult &post) {
    if (!display_initialized) return;
    canvas.fillScreen(ST77XX_BLACK);

    // Naslov
    canvas.setTextSize(1);
    canvas.setTextColor(ST77XX_CYAN);
    canvas.setCursor(14, 6);
    canvas.println("ACOUSTIC LOCATOR");
    canvas.drawFastHLine(8, 16, 112, COLOR_MUTED_GRAY);

    const uint32_t elapsed_ms = (total_ms >= remaining_ms) ? (total_ms - remaining_ms) : total_ms;

    // POST Checklist (staggered display for user experience)
    // PSRAM (revealed at >= 200 ms)
    canvas.setTextColor(ST77XX_WHITE);
    canvas.setCursor(8, 21);
    canvas.print("PSRAM :");
    canvas.setCursor(54, 21);
    if (elapsed_ms >= 200) {
        if (post.psram_ok) {
            canvas.setTextColor(ST77XX_GREEN);
            canvas.printf("[OK] %uMB", static_cast<unsigned>(post.psram_bytes / (1024 * 1024)));
        } else {
            canvas.setTextColor(ST77XX_RED);
            canvas.print("[FAIL]");
        }
    } else {
        canvas.setTextColor(COLOR_MUTED_GRAY);
        canvas.print("[..]");
    }

    // BME280 (revealed at >= 400 ms)
    canvas.setTextColor(ST77XX_WHITE);
    canvas.setCursor(8, 32);
    canvas.print("BME280:");
    canvas.setCursor(54, 32);
    if (elapsed_ms >= 400) {
        if (post.bme_ok) {
            canvas.setTextColor(ST77XX_GREEN);
            canvas.printf("[OK] %.0fC", post.bme_temp_c);
        } else {
            canvas.setTextColor(COLOR_MUTED_GRAY);
            canvas.print("[--] Default");
        }
    } else {
        canvas.setTextColor(COLOR_MUTED_GRAY);
        canvas.print("[..]");
    }

    // M0 & M1 (Bus 0: GPIO 6, revealed at >= 600 ms and >= 800 ms)
    canvas.setTextColor(ST77XX_WHITE);
    canvas.setCursor(8, 43);
    canvas.print("M0:");
    if (elapsed_ms >= 600) {
        canvas.setTextColor(post.mic_health.ch_ok[0] ? ST77XX_GREEN : ST77XX_RED);
        canvas.print(post.mic_health.ch_ok[0] ? "[OK]" : "[ER]");
    } else {
        canvas.setTextColor(COLOR_MUTED_GRAY);
        canvas.print("[..]");
    }

    canvas.setTextColor(ST77XX_WHITE);
    canvas.setCursor(68, 43);
    canvas.print("M1:");
    if (elapsed_ms >= 800) {
        canvas.setTextColor(post.mic_health.ch_ok[1] ? ST77XX_GREEN : ST77XX_RED);
        canvas.print(post.mic_health.ch_ok[1] ? "[OK]" : "[ER]");
    } else {
        canvas.setTextColor(COLOR_MUTED_GRAY);
        canvas.print("[..]");
    }

    // M2 & M3 (Bus 1: GPIO 7, revealed at >= 1000 ms and >= 1200 ms)
    canvas.setTextColor(ST77XX_WHITE);
    canvas.setCursor(8, 54);
    canvas.print("M2:");
    if (elapsed_ms >= 1000) {
        canvas.setTextColor(post.mic_health.ch_ok[2] ? ST77XX_GREEN : ST77XX_RED);
        canvas.print(post.mic_health.ch_ok[2] ? "[OK]" : "[ER]");
    } else {
        canvas.setTextColor(COLOR_MUTED_GRAY);
        canvas.print("[..]");
    }

    canvas.setTextColor(ST77XX_WHITE);
    canvas.setCursor(68, 54);
    canvas.print("M3:");
    if (elapsed_ms >= 1200) {
        canvas.setTextColor(post.mic_health.ch_ok[3] ? ST77XX_GREEN : ST77XX_RED);
        canvas.print(post.mic_health.ch_ok[3] ? "[OK]" : "[ER]");
    } else {
        canvas.setTextColor(COLOR_MUTED_GRAY);
        canvas.print("[..]");
    }

    // Dual-I2S Bus Synchronization & Channel Independence (revealed at >= 1400 ms)
    canvas.setTextColor(ST77XX_WHITE);
    canvas.setCursor(8, 65);
    canvas.print("SYNC  :");
    canvas.setCursor(54, 65);
    if (elapsed_ms >= 1400) {
        if (post.i2s_driver_ok &&
            post.mic_health.bus0_distinct &&
            post.mic_health.bus1_distinct &&
            post.mic_health.cross_bus_distinct) {
            canvas.setTextColor(ST77XX_GREEN);
            canvas.print("[OK] Quad");
        } else if (!post.i2s_driver_ok) {
            canvas.setTextColor(ST77XX_RED);
            canvas.print("[FAIL] I2S");
        } else if (!post.mic_health.bus0_distinct || !post.mic_health.bus1_distinct) {
            canvas.setTextColor(ST77XX_RED);
            canvas.print("[FAIL] BusLR");
        } else {
            canvas.setTextColor(ST77XX_RED);
            canvas.print("[FAIL] XBus");
        }
    } else {
        canvas.setTextColor(COLOR_MUTED_GRAY);
        canvas.print("[..]");
    }

    canvas.drawFastHLine(8, 78, 112, COLOR_MUTED_GRAY);

    // Poziv za setup
    canvas.setTextColor(ST77XX_YELLOW);
    canvas.setCursor(8, 86);
    canvas.println("Hold BOOT for Setup");

    // Vrstica napredka
    const int bar_x = 14;
    const int bar_y = 100;
    const int bar_w = 100;
    const int bar_h = 7;
    canvas.drawRect(bar_x, bar_y, bar_w, bar_h, ST77XX_WHITE);
    int fill_w = (remaining_ms * (bar_w - 2)) / total_ms;
    if (fill_w > 0) {
        canvas.fillRect(bar_x + 1, bar_y + 1, fill_w, bar_h - 2, ST77XX_CYAN);
    }

    // Besedilo odštevanja
    canvas.setTextColor(COLOR_MUTED_GRAY);
    canvas.setCursor(14, 114);
    int sec = (remaining_ms + 999) / 1000;
    canvas.printf("Starting in %d s...", sec);

    // Status celote (revealed at >= 1600 ms, otherwise shows TESTING...)
    canvas.setCursor(8, 136);
    if (elapsed_ms < 1600) {
        canvas.setTextColor(ST77XX_CYAN);
        canvas.print("STATUS: TESTING...");
    } else if (post.all_nominal) {
        canvas.setTextColor(ST77XX_GREEN);
        canvas.print("STATUS: ALL NOMINAL");
    } else {
        canvas.setTextColor(ST77XX_RED);
        canvas.print("STATUS: DEGRADED");
    }

    display.drawRGBBitmap(0, 0, canvas.getBuffer(), TFT_WIDTH, TFT_HEIGHT);
}

static void show_setup_portal_screen() {
    if (!display_initialized) return;
    canvas.fillScreen(ST77XX_BLACK);

    canvas.setTextSize(1);
    canvas.setTextColor(ST77XX_YELLOW);
    canvas.setCursor(16, 12);
    canvas.println("=== SETUP MODE ===");
    canvas.drawFastHLine(10, 24, 108, COLOR_MUTED_GRAY);

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
    canvas.println("http://" SETUP_AP_DEFAULT_IP_STR);

    canvas.setTextColor(COLOR_MUTED_GRAY);
    canvas.setCursor(8, 130);
    canvas.println("Save & reboot");
    canvas.setCursor(8, 142);
    canvas.println("when finished.");

    display.drawRGBBitmap(0, 0, canvas.getBuffer(), TFT_WIDTH, TFT_HEIGHT);
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

static void show_hardware_degraded_screen(const BootSelfTestResult &post) {
    if (!display_initialized) return;
    canvas.fillScreen(ST77XX_BLACK);

    // Red Header Banner
    canvas.fillRect(0, 0, TFT_WIDTH, 18, ST77XX_RED);
    canvas.setTextSize(1);
    canvas.setTextColor(ST77XX_WHITE);
    canvas.setCursor(13, 5);
    canvas.println("HARDWARE DEGRADED");

    // Failure checklist
    canvas.setCursor(8, 24);
    if (!post.psram_ok) {
        canvas.setTextColor(ST77XX_RED);
        canvas.println("PSRAM : [FAIL]");
    } else {
        canvas.setTextColor(COLOR_MUTED_GRAY);
        canvas.printf("PSRAM : [OK] %uMB\n", static_cast<unsigned>(post.psram_bytes / (1024 * 1024)));
    }

    // M0..M3
    static const char* const mic_names[AUDIO_NUM_CHANNELS] = {"M0", "M1", "M2", "M3"};
    for (size_t c = 0; c < AUDIO_NUM_CHANNELS; ++c) {
        const int y = 36 + static_cast<int>(c) * 11;
        canvas.setCursor(8, y);
        if (!post.mic_health.ch_ok[c]) {
            const char* reason = (post.mic_health.ch_min[c] == post.mic_health.ch_max[c]) ? "FLATLINE" : "RAIL SHORT";
            canvas.setTextColor(ST77XX_RED);
            canvas.printf("%s    : [FAIL] %s", mic_names[c], reason);
        } else {
            canvas.setTextColor(ST77XX_GREEN);
            canvas.printf("%s    : [OK]", mic_names[c]);
        }
    }

    // Bus Sync
    canvas.setCursor(8, 80);
    if (!post.i2s_driver_ok) {
        canvas.setTextColor(ST77XX_RED);
        canvas.println("SYNC  : [FAIL] Driver");
    } else if (!post.mic_health.bus0_distinct) {
        canvas.setTextColor(ST77XX_RED);
        canvas.println("SYNC  : [FAIL] Bus0 LR");
    } else if (!post.mic_health.bus1_distinct) {
        canvas.setTextColor(ST77XX_RED);
        canvas.println("SYNC  : [FAIL] Bus1 LR");
    } else if (!post.mic_health.cross_bus_distinct) {
        canvas.setTextColor(ST77XX_RED);
        canvas.println("SYNC  : [FAIL] X-Bus");
    } else {
        canvas.setTextColor(ST77XX_GREEN);
        canvas.println("SYNC  : [OK] Quad Synced");
    }

    // Divider
    canvas.drawFastHLine(8, 96, 112, COLOR_MUTED_GRAY);

    // User Guidance
    canvas.setTextColor(ST77XX_YELLOW);
    canvas.setCursor(8, 104);
    canvas.println("Fix wiring & reboot.");
    canvas.setCursor(8, 116);
    canvas.println("Operation halted.");

    canvas.setTextColor(ST77XX_CYAN);
    canvas.setCursor(8, 134);
    canvas.println("Hold BOOT (GPIO 0)");
    canvas.setCursor(8, 146);
    canvas.println("for Setup Portal.");

    display.drawRGBBitmap(0, 0, canvas.getBuffer(), TFT_WIDTH, TFT_HEIGHT);
}

static void handle_hardware_degraded(const BootSelfTestResult &post) {
    Serial.println("\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    Serial.println("  CRITICAL: BOOT SELF-TEST FAILED - HALTING!");
    Serial.println("  Real-time DOA processing and network streaming");
    Serial.println("  are suspended to prevent corrupt measurements.");
    Serial.println("  Fix hardware wiring and press RST to reboot,");
    Serial.println("  or hold BOOT (GPIO 0) to enter setup mode.");
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");

    show_hardware_degraded_screen(post);

    uint32_t press_start_ms = 0;
    while (true) {
        if (digitalRead(SETUP_BUTTON_PIN) == LOW) {
            if (press_start_ms == 0) {
                press_start_ms = millis();
            } else if (millis() - press_start_ms >= 100) {
                enter_setup_mode();
                show_hardware_degraded_screen(post);
                press_start_ms = 0;
            }
        } else {
            press_start_ms = 0;
        }
        delay(20);
    }
}

static void check_setup_button(const BootSelfTestResult &post_result) {
    pinMode(SETUP_BUTTON_PIN, INPUT_PULLUP);
    const uint32_t total_ms = SETUP_BOOT_DELAY_MS;
    const uint32_t start_ms = millis();
    uint32_t last_draw_ms = 0;
    bool button_pressed = false;

    while (millis() - start_ms < total_ms) {
        if (digitalRead(SETUP_BUTTON_PIN) == LOW) {
            button_pressed = true;
            break;
        }

        const uint32_t now = millis();
        if (last_draw_ms == 0 || now - last_draw_ms >= 100) {
            last_draw_ms = now;
            uint32_t elapsed = now - start_ms;
            uint32_t remaining = (elapsed < total_ms) ? (total_ms - elapsed) : 0;
            show_boot_prompt(remaining, total_ms, post_result);
        }
        delay(10);
    }

    if (button_pressed) {
        enter_setup_mode();
    }

    // If hardware is degraded, halt operation and hold the diagnostic screen
    if (!post_result.all_nominal) {
        handle_hardware_degraded(post_result);
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
    static int32_t ch0_buf[AUDIO_MAX_WINDOW_SIZE];
    static int32_t ch1_buf[AUDIO_MAX_WINDOW_SIZE];
    static int32_t ch2_buf[AUDIO_MAX_WINDOW_SIZE];
    static int32_t ch3_buf[AUDIO_MAX_WINDOW_SIZE];
    esp_task_wdt_add(nullptr);

    while (true) {
        uint32_t window_size;
        uint32_t sample_rate;
        if (xSemaphoreTake(config_mutex, pdMS_TO_TICKS(CONFIG_MUTEX_FAST_TIMEOUT_MS)) != pdTRUE) {
            esp_task_wdt_reset();
            continue;
        }
        sample_rate = device_config.sample_rate;
        window_size = device_config.window_size;
        xSemaphoreGive(config_mutex);

        bool read_ok = false;
        if (xSemaphoreTake(i2s_mutex, portMAX_DELAY) == pdTRUE) {
            read_ok = read_i2s_quad(ch0_buf, ch1_buf, ch2_buf, ch3_buf, window_size);
            xSemaphoreGive(i2s_mutex);
        }
        if (read_ok) {
            const uint32_t process_start = micros();
            if (xSemaphoreTake(doa_mutex, portMAX_DELAY) != pdTRUE) {
                esp_task_wdt_reset();
                continue;
            }
            DoaResult res = doa.process(ch0_buf, ch1_buf, ch2_buf, ch3_buf);
            const float noise_floor = doa.getNoiseFloorDb();
            xSemaphoreGive(doa_mutex);
            current_noise_floor_db.store(noise_floor);

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
                                  float humidity_pct = DEFAULT_HUMIDITY_PCT,
                                  float pressure_pa = DEFAULT_PRESSURE_PA) {
    if (xSemaphoreTake(config_mutex, pdMS_TO_TICKS(CONFIG_MUTEX_BLOCK_TIMEOUT_MS)) != pdTRUE) {
        Serial.println("Could not lock device configuration");
        return false;
    }

    const float microphone_distance_m = device_config.microphone_distance_m;
    const float correlation_threshold = device_config.correlation_threshold;
    const float sound_level_threshold_db_spl =
        device_config.sound_level_threshold_db_spl;
    const float configured_offset = device_config.db_spl_offset;
    const float angle_offset = device_config.angle_offset_deg;
    const uint32_t sample_rate = device_config.sample_rate;
    const uint32_t window_size = device_config.window_size;
    const bool adaptive_enabled = device_config.adaptive_threshold_enabled;
    const float snr_margin = device_config.snr_margin_db;
    xSemaphoreGive(config_mutex);

    if (microphone_distance_m <= 0.0f ||
        correlation_threshold <= 0.0f ||
        sound_level_threshold_db_spl < SPL_THRESHOLD_MIN_DB ||
        !is_supported_audio_config(sample_rate, window_size) ||
        !std::isfinite(snr_margin) || snr_margin < SNR_MARGIN_MIN_DB || snr_margin > SNR_MARGIN_MAX_DB ||
        !std::isfinite(angle_offset) || angle_offset < ANGLE_OFFSET_MIN_DEG || angle_offset > ANGLE_OFFSET_MAX_DEG) {
        Serial.println("Invalid or unsupported remote calibration");
        return false;
    }

    if (xSemaphoreTake(doa_mutex, pdMS_TO_TICKS(DOA_MUTEX_BLOCK_TIMEOUT_MS)) != pdTRUE) {
        Serial.println("Could not lock DSP calibration");
        return false;
    }

    doa.updateCalibration(
        microphone_distance_m,
        temperature_c,
        humidity_pct,
        pressure_pa,
        correlation_threshold,
        sound_level_threshold_db_spl - configured_offset,
        adaptive_enabled,
        snr_margin,
        angle_offset);
    xSemaphoreGive(doa_mutex);
    db_spl_offset.store(configured_offset);
    adaptive_threshold_active.store(adaptive_enabled);
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

    if (xSemaphoreTake(config_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        Serial.println("Could not lock device configuration");
        return false;
    }
    DeviceConfig previous = device_config;
    xSemaphoreGive(config_mutex);

    bool adaptive_enabled = previous.adaptive_threshold_enabled;
    if (!doc["prilagodljivi_prag"].isNull()) {
        adaptive_enabled = doc["prilagodljivi_prag"].as<bool>();
    } else if (!doc["adaptive_threshold"].isNull()) {
        adaptive_enabled = doc["adaptive_threshold"].as<bool>();
    } else if (!doc["adaptive_threshold_enabled"].isNull()) {
        adaptive_enabled = doc["adaptive_threshold_enabled"].as<bool>();
    }

    float snr_margin = previous.snr_margin_db;
    if (!doc["snr_rob_db"].isNull()) {
        snr_margin = doc["snr_rob_db"].as<float>();
    } else if (!doc["snr_margin_db"].isNull()) {
        snr_margin = doc["snr_margin_db"].as<float>();
    } else if (!doc["snr_margin"].isNull()) {
        snr_margin = doc["snr_margin"].as<float>();
    }

    float angle_offset = previous.angle_offset_deg;
    if (!doc["kot_odmik"].isNull()) {
        angle_offset = doc["kot_odmik"].as<float>();
    } else if (!doc["angle_offset_deg"].isNull()) {
        angle_offset = doc["angle_offset_deg"].as<float>();
    } else if (!doc["angle_offset"].isNull()) {
        angle_offset = doc["angle_offset"].as<float>();
    }

    if (distance <= 0.0f || correlation <= 0.0f ||
        spl_threshold < SPL_THRESHOLD_MIN_DB ||
        !is_supported_audio_config(sample_rate, window_size) ||
        !isfinite(distance) || !isfinite(correlation) ||
        !isfinite(spl_threshold) || !isfinite(offset) ||
        !isfinite(snr_margin) || snr_margin < SNR_MARGIN_MIN_DB || snr_margin > SNR_MARGIN_MAX_DB ||
        !isfinite(angle_offset) || angle_offset < ANGLE_OFFSET_MIN_DEG || angle_offset > ANGLE_OFFSET_MAX_DEG) {
        Serial.println("Calibration response contains invalid or unsupported values");
        return false;
    }

    const bool audio_config_changed =
        sample_rate != previous.sample_rate || window_size != previous.window_size;
    if (audio_config_changed &&
        !reconfigure_audio(sample_rate, window_size)) {
        return false;
    }

    if (xSemaphoreTake(config_mutex, pdMS_TO_TICKS(CONFIG_MUTEX_BLOCK_TIMEOUT_MS)) != pdTRUE) {
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
    device_config.adaptive_threshold_enabled = adaptive_enabled;
    device_config.snr_margin_db = snr_margin;
    device_config.angle_offset_deg = angle_offset;
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
                   float db_spl, bool api_available, bool adaptive_active = false) {
    canvas.fillScreen(ST77XX_BLACK);

    // 1. Izris 360-stopinjskega krožnega radarja (cx = 64, cy = 44, r = 34)
    const int cx = RADAR_CENTER_X;
    const int cy = RADAR_CENTER_Y;
    const int r = RADAR_RADIUS;

    // Značka načina delovanja (AUTO ali FIX) v zgornjem desnem kotu
    if (adaptive_active) {
        canvas.drawRoundRect(91, 2, 34, 11, 2, ST77XX_CYAN);
        canvas.setTextSize(1);
        canvas.setTextColor(ST77XX_CYAN);
        canvas.setCursor(96, 4);
        canvas.print("AUTO");
    } else {
        canvas.drawRoundRect(96, 2, 29, 11, 2, COLOR_BADGE_BORDER);
        canvas.setTextSize(1);
        canvas.setTextColor(COLOR_MUTED_GRAY);
        canvas.setCursor(102, 4);
        canvas.print("FIX");
    }

    // Koncentrična obroča in križne osi N/E/S/W (temno zelena mreža)
    canvas.drawCircle(cx, cy, r, COLOR_RADAR_GRID);
    canvas.drawCircle(cx, cy, r / 2, COLOR_RADAR_GRID);
    canvas.drawLine(cx - r, cy, cx + r, cy, COLOR_RADAR_GRID);
    canvas.drawLine(cx, cy - r, cx, cy + r, COLOR_RADAR_GRID);

    // Kardinalne oznake azimuta (0, +90, 180, -90)
    canvas.setTextSize(1);
    canvas.setTextColor(ST77XX_WHITE);
    canvas.setCursor(cx - 2, 1);
    canvas.print("0");
    canvas.setCursor(cx + r + 4, cy - 3);
    canvas.print("+90");
    canvas.setCursor(cx - r - 21, cy - 3);
    canvas.print("-90");
    canvas.setCursor(cx + 3, cy + r - 9);
    canvas.print("180");

    // 2. Izris 360-stopinjske polarne igle (barva glede na glasnost)
    uint16_t needle_color = (db_spl > NEEDLE_HIGH_SPL_THRESHOLD) ? ST77XX_RED :
                            ((db_spl > NEEDLE_MED_SPL_THRESHOLD) ? ST77XX_YELLOW : ST77XX_GREEN);
    const float rad = (angle_deg - 90.0f) * (static_cast<float>(M_PI) / 180.0f);
    const int target_x = cx + static_cast<int>(roundf(r * cosf(rad)));
    const int target_y = cy + static_cast<int>(roundf(r * sinf(rad)));

    canvas.fillCircle(cx, cy, 2, COLOR_MUTED_GRAY);
    canvas.drawLine(cx, cy, target_x, target_y, needle_color);
    canvas.fillCircle(target_x, target_y, 3, needle_color);

    // 3. Spodnja nadzorna plošča s telemetrijo (y = 84..159)
    canvas.drawFastHLine(0, 82, TFT_WIDTH, COLOR_MUTED_GRAY); // Siva ločilna črta

    canvas.setTextSize(2);
    canvas.setTextColor(ST77XX_CYAN);
    canvas.setCursor(4, 84);
    canvas.printf("%+6.1f deg", angle_deg);

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
    display.drawRGBBitmap(0, 0, canvas.getBuffer(), TFT_WIDTH, TFT_HEIGHT);
}

void init_sensors_and_display() {
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_BUS_SPEED_HZ);

    // 1. Zagon SPI TFT zaslona KMR-1.8 (ST7735)
    tft_spi.begin(TFT_SCLK_PIN, -1, TFT_MOSI_PIN, -1);
    display.initR(INITR_BLACKTAB); // Če so barve obrnjene ali odrezane, INITR_GREENTAB / INITR_18BLACKTAB
    display.setSPISpeed(27000000); // 27 MHz SPI (optimalno za ST7735)
    display.setRotation(0);        // Pokončni način 128x160
    if (TFT_INVERT_COLORS) {
        display.invertDisplay(true);
    }
    display_initialized = true;

    // 2. Zagon BME280
    bme_initialized = bme.begin(BME280_ADDR_PRIMARY, &Wire);
    if (!bme_initialized) {
        // Če ni na primarnem naslovu, poskusi še sekundarnega
        bme_initialized = bme.begin(BME280_ADDR_SECONDARY, &Wire);
        if (!bme_initialized) {
            Serial.printf("BME280 senzor ni bil najden! Uporabljam privzeto c=%.1f m/s\n",
                          DEFAULT_SOUND_SPEED_MPS);
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
    if (millis() - last_read > SENSOR_READ_INTERVAL_MS) {
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
    DisplaySmoothingFilter display_filter;

    while (true) {
        update_environmental_compensation();

        if (xQueueReceive(display_queue, &event, pdMS_TO_TICKS(DISPLAY_QUEUE_TIMEOUT_MS))) {
            last_event = event;
            const uint32_t now_ms = millis();
            const float display_angle = display_filter.update(event.angle_deg, now_ms);

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
                draw_radar_ui(display_angle, temp, humidity, pressure, speed,
                              calibrated_db, last_server_available,
                              adaptive_threshold_active.load());
            }
        }

        const bool current_server_available = server_available.load();
        if (display_initialized &&
            current_server_available != last_server_available) {
            last_server_available = current_server_available;
            const float calibrated_db = last_event.rms_db + db_spl_offset.load();
            draw_radar_ui(display_filter.isInitialized() ? display_filter.getSmoothedAngle() : last_event.angle_deg,
                          current_temperature.load(), current_humidity.load(),
                          current_pressure.load(), current_sound_speed.load(),
                          calibrated_db, last_server_available,
                          adaptive_threshold_active.load());
        }
        esp_task_wdt_reset();
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

    // 2. Zagonski preizkus strojne opreme (POST)
    BootSelfTestResult post_res = run_boot_self_test(device_config.sample_rate);

    // 3. Preveri gumb za nastavitve (prikaze POST rezultate in odstevalnik na zaslonu)
    check_setup_button(post_res);
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
                      server_available.load(),
                      adaptive_threshold_active.load());
    }

    // Ustvari FreeRTOS objekte za prenos rezultatov in zaščito DSP konfiguracije
    display_queue = xQueueCreate(DISPLAY_QUEUE_LENGTH, sizeof(DoaResult));
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
        Serial.printf("Unsupported saved audio configuration; using %lu Hz / %zu samples\n",
                      static_cast<unsigned long>(DEFAULT_SAMPLE_RATE),
                      DEFAULT_WINDOW_SIZE);
        device_config.sample_rate = DEFAULT_SAMPLE_RATE;
        device_config.window_size = DEFAULT_WINDOW_SIZE;
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
    if (xTaskCreatePinnedToCore(dsp_task, "DSP_Task", TASK_DSP_STACK_SIZE,
                                NULL, TASK_DSP_PRIORITY, NULL, TASK_DSP_CORE) != pdPASS) {
        Serial.println("DSP naloge ni bilo mogoce zagnati!");
        while (true) {
            delay(1000);
        }
    }

    if (xTaskCreatePinnedToCore(sensor_display_task, "Sensor_Display", TASK_SENSOR_DISPLAY_STACK_SIZE,
                                NULL, TASK_SENSOR_DISPLAY_PRIORITY, NULL, TASK_SENSOR_DISPLAY_CORE) != pdPASS) {
        Serial.println("Sensor/display naloge ni bilo mogoce zagnati!");
        while (true) {
            delay(1000);
        }
    }

    if (xTaskCreatePinnedToCore(network_task, "Network_Task", TASK_NETWORK_STACK_SIZE,
                                NULL, TASK_NETWORK_PRIORITY, NULL, TASK_NETWORK_CORE) != pdPASS) {
        Serial.println("Omrezne naloge ni bilo mogoce zagnati!");
        while (true) {
            delay(1000);
        }
    }
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}