#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <DNSServer.h>
#include <WebServer.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BME280.h>
#include <atomic>

#include "gcc_phat.h"

// I2C Pini za ESP32-S3
#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9
#define SETUP_BUTTON_PIN 0
#define SETUP_BOOT_DELAY_MS 1500
#define SETUP_AP_NAME "Acoustic-Locator-Setup"
#define SETUP_AP_PASSWORD "configure123"
#define CONFIG_REFRESH_INTERVAL_MS 60000UL
#define HTTP_RETRY_COUNT 3

// Globalne periferne enote
Adafruit_BME280 bme; // I2C

// Deljene spremenljivke med jedroma (Thread-safe)
std::atomic<float> current_sound_speed(343.2f);
std::atomic<float> current_temperature(20.0f);

// Funkcije iz i2s_sampler.cpp
bool init_i2s_microphones(uint32_t sample_rate);
bool read_i2s_stereo(int32_t* left, int32_t* right, size_t count);

// Nastavitve sistema
static constexpr char DEFAULT_WIFI_SSID[] = "";
static constexpr char DEFAULT_WIFI_PASSWORD[] = "";
static constexpr char DEFAULT_API_URL[] = "";
static constexpr char DEFAULT_API_KEY[] = "";

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

static GccPhat doa(44100, 0.08f, 20.0f); // 44.1 kHz, 8 cm, 20 °C
static QueueHandle_t display_queue;
static QueueHandle_t network_queue;
static Adafruit_SSD1306 display(128, 64, &Wire, -1);
static SemaphoreHandle_t doa_mutex;
static SemaphoreHandle_t config_mutex;
static Preferences preferences;
static DeviceConfig device_config;
static std::atomic<float> db_spl_offset(0.0f);
static bool display_initialized = false;
static bool bme_initialized = false;
static bool setup_mode = false;
static bool device_configured = false;
static DNSServer setup_dns;
static WebServer setup_server(80);

static void load_device_config() {
    preferences.begin("acoustic", false);

    device_config.wifi_ssid =
        preferences.getString("wifi_ssid", DEFAULT_WIFI_SSID);
    device_config.wifi_password =
        preferences.getString("wifi_pass", DEFAULT_WIFI_PASSWORD);
    device_config.api_url =
        preferences.getString("api_url", DEFAULT_API_URL);
    device_config.api_key =
        preferences.getString("api_key", DEFAULT_API_KEY);
    device_config.device_id = preferences.getUInt("device_id", 0);
    device_config.microphone_distance_m =
        preferences.getFloat("mic_distance", 0.08f);
    device_config.correlation_threshold =
        preferences.getFloat("corr_threshold", 0.35f);
    device_config.sound_level_threshold_db_spl =
        preferences.getFloat("spl_threshold", 55.0f);
    device_config.db_spl_offset = preferences.getFloat("spl_offset", 0.0f);
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
    page += "Wi-Fi password<br><input type='password' name='wifi_password'><br>";
    page += "API URL<br><input name='api_url' value='" + device_config.api_url + "'><br>";
    page += "API key<br><input name='api_key' value='" + device_config.api_key + "'><br>";
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
        !setup_server.hasArg("api_key") ||
        !setup_server.hasArg("device_id") ||
        !setup_server.hasArg("mic_distance") ||
        !setup_server.hasArg("spl_offset")) {
        setup_server.send(400, "text/plain", "Missing required fields");
        return;
    }

    String wifi_password = setup_server.arg("wifi_password");
    device_config.wifi_ssid = setup_server.arg("wifi_ssid");
    if (wifi_password.length() > 0) {
        device_config.wifi_password = wifi_password;
    }
    device_config.api_url = setup_server.arg("api_url");
    device_config.api_key = setup_server.arg("api_key");
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

static void enter_setup_mode() {
    setup_mode = true;
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
    delay(SETUP_BOOT_DELAY_MS);
    if (digitalRead(SETUP_BUTTON_PIN) == LOW) {
        enter_setup_mode();
    }
}

// --- NALOGA NA JEDRU 1: DSP OBDELAVA ---
void dsp_task(void *param) {
    static int32_t left_buf[N_SAMPLES];
    static int32_t right_buf[N_SAMPLES];

    while (true) {
        if (read_i2s_stereo(left_buf, right_buf, N_SAMPLES)) {
            if (xSemaphoreTake(doa_mutex, portMAX_DELAY) != pdTRUE) {
                continue;
            }
            DoaResult res = doa.process(left_buf, right_buf);
            xSemaphoreGive(doa_mutex);
            
            // Vsak potrošnik prejme svojo kopijo dogodka.
            if (res.valid) {
                xQueueSend(display_queue, &res, 0);
                xQueueSend(network_queue, &res, 0);
            }
        }
    }
}

static bool apply_dsp_calibration(float temperature_c) {
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
        sample_rate != 44100 ||
        window_size != N_SAMPLES) {
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
        correlation_threshold,
        sound_level_threshold_db_spl - configured_offset);
    xSemaphoreGive(doa_mutex);
    db_spl_offset.store(configured_offset);
    return true;
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

    int status = http.GET();
    String response = http.getString();
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

    float distance = doc["razdalja_mikrofonov_m"] | 0.0f;
    float correlation = doc["prag_korelacije"] | 0.0f;
    float spl_threshold = doc["prag_glasnosti_db"] | 0.0f;
    uint32_t sample_rate = doc["vzorcevalna_frekvenca"] | 0U;
    uint32_t window_size = doc["okno_velikost"] | 0U;
    float offset = doc["db_spl_offset"] | db_spl_offset.load();

    if (distance <= 0.0f || correlation <= 0.0f ||
        spl_threshold <= -200.0f || sample_rate == 0 || window_size == 0 ||
        !isfinite(distance) || !isfinite(correlation) ||
        !isfinite(spl_threshold) || !isfinite(offset)) {
        Serial.println("Calibration response contains invalid values");
        return false;
    }

    if (xSemaphoreTake(config_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        Serial.println("Could not lock device configuration");
        return false;
    }
    DeviceConfig previous = device_config;
    device_config.microphone_distance_m = distance;
    device_config.correlation_threshold = correlation;
    device_config.sound_level_threshold_db_spl = spl_threshold;
    device_config.db_spl_offset = offset;
    device_config.sample_rate = sample_rate;
    device_config.window_size = window_size;
    xSemaphoreGive(config_mutex);

    if (!apply_dsp_calibration(current_temperature.load())) {
        xSemaphoreTake(config_mutex, portMAX_DELAY);
        device_config = previous;
        xSemaphoreGive(config_mutex);
        return false;
    }

    save_device_config();
    Serial.println("Remote calibration applied");
    return true;
}

// --- OMREŽNI DEL ---
static bool send_event_to_backend(const DoaResult& res) {
    if (WiFi.status() != WL_CONNECTED || device_config.device_id == 0) {
        if (device_config.device_id == 0) {
            Serial.println("Measurement not sent: device ID is not configured");
        }
        return false;
    }

    HTTPClient http;
    String url = device_config.api_url + "/meritve";
    if (!http.begin(url)) {
        Serial.println("HTTP client initialization failed");
        return false;
    }
    http.setTimeout(5000);
    http.addHeader("Content-Type", "application/json");
    http.addHeader("x-api-key", device_config.api_key);

    JsonDocument doc;
    doc["naprava_id"] = device_config.device_id;
    doc["kot_stopinje"] = res.angle_deg;
    doc["casovna_zakasnitev_us"] = res.delay_us;
    doc["raven_zvoka_db"] = res.rms_db + db_spl_offset.load();
    doc["korelacijski_koeficient"] = res.confidence;

    String json_str;
    serializeJson(doc, json_str);

    int httpResponseCode = http.POST(json_str);
    String response = http.getString();
    http.end();

    if (httpResponseCode < 200 || httpResponseCode >= 300) {
        Serial.printf("HTTP POST /meritve failed: %d, response: %s\n",
                      httpResponseCode, response.c_str());
        return false;
    }

    Serial.printf("Measurement accepted: HTTP %d, dB SPL: %.1f\n",
                  httpResponseCode, res.rms_db + db_spl_offset.load());
    return true;
}

void network_task(void *param) {
    DoaResult event;
    unsigned long last_calibration_refresh = 0;

    while (true) {
        if (WiFi.status() != WL_CONNECTED) {
            WiFi.reconnect();
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        if (last_calibration_refresh == 0 ||
            millis() - last_calibration_refresh >= CONFIG_REFRESH_INTERVAL_MS) {
            fetch_remote_calibration();
            last_calibration_refresh = millis();
        }

        if (xQueueReceive(network_queue, &event, pdMS_TO_TICKS(1000)) == pdTRUE) {
            bool sent = false;
            for (int attempt = 1; attempt <= HTTP_RETRY_COUNT && !sent; ++attempt) {
                sent = send_event_to_backend(event);
                if (!sent && attempt < HTTP_RETRY_COUNT) {
                    vTaskDelay(pdMS_TO_TICKS(1000 * attempt));
                }
            }
            if (!sent) {
                Serial.println("Measurement dropped after RAM retry limit");
            }
        }
    }
}

void draw_radar_ui(Adafruit_SSD1306 &display, float angle_deg, float temp_c, float c_sound, float db) {
    display.clearDisplay();

    // 1. Izris polkrožnega radarja (središče spodaj na x=64, y=52, polmer r=42)
    const int cx = 64;
    const int cy = 52;
    const int r = 40;

    // Lok od -90° do +90°
    display.drawCircle(cx, cy, r, SSD1306_WHITE);
    // Prekrijemo spodnjo polovico kroga, da ostane le zgornji polkrog
    display.fillRect(0, cy + 1, 128, 64 - cy, SSD1306_BLACK);
    display.drawLine(cx - r, cy, cx + r, cy, SSD1306_WHITE); // Osnovna linija

    // Oznake -90, 0, +90
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(cx - r - 20, cy - 8); display.print("-90");
    display.setCursor(cx - 6, cy - r - 9);  display.print("0");
    display.setCursor(cx + r + 3, cy - 8);  display.print("+90");

    // 2. Izris igle (smer vira zvoka)
    // Kot pretvorimo v radiane (0° je naravnost navzgor = -90° v standardnem koordinatnem sistemu)
    float rad = (angle_deg - 90.0f) * (M_PI / 180.0f);
    int target_x = cx + (int)(r * cosf(rad));
    int target_y = cy + (int)(r * sinf(rad));

    display.drawLine(cx, cy, target_x, target_y, SSD1306_WHITE);
    display.fillCircle(target_x, target_y, 2, SSD1306_WHITE); // Pika na vrhu igle

    // 3. Spodnja vrstica s telemetrijo (BME280 + Akustika)
    display.setCursor(0, 56);
    display.printf("%+.1f deg | %.1fC | %.0fm/s", angle_deg, temp_c, c_sound);

    display.display();
}

void init_sensors_and_display() {
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 400000); // 400 kHz Fast I2C

    // 1. Zagon OLED
    display_initialized = display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    if (!display_initialized) {
        Serial.println("SSD1306 ni najden!");
    }
    if (display_initialized) {
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 10);
        display.println("Inicializacija...");
        display.display();
    }

    // 2. Zagon BME280
    bme_initialized = bme.begin(0x76, &Wire);
    if (!bme_initialized) {
        // Če ni na 0x76, poskusi še privzeti 0x77
        bme_initialized = bme.begin(0x77, &Wire);
        if (!bme_initialized) {
            Serial.println("BME280 senzor ni bil najden! Uporabljam privzeto c=343 m/s");
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

        float t = bme.readTemperature();
        if (!isnan(t)) {
            // Formula za hitrost zvoka v zraku
            float c = 331.3f * sqrtf(1.0f + (t / 273.15f));

            current_temperature.store(t);
            current_sound_speed.store(c);

            // Dinamično posodobimo hitrost zvoka v DSP razredu
            apply_dsp_calibration(t);
        }
    }
}

void sensor_display_task(void *param) {
    DoaResult event;

    while (true) {
        update_environmental_compensation();

        if (xQueueReceive(display_queue, &event, pdMS_TO_TICKS(100))) {
            float temp = current_temperature.load();
            float speed = current_sound_speed.load();

            Serial.printf("Zaznan kot: %.1f deg, R: %.2f, dB: %.1f\n",
                          event.angle_deg, event.confidence,
                          event.rms_db + db_spl_offset.load());

            if (display_initialized) {
                draw_radar_ui(display, event.angle_deg, temp, speed, event.rms_db);
            }
        }
    }
}



void setup() {
    Serial.begin(115200);
    load_device_config();
    check_setup_button();
    if (!device_configured) {
        Serial.println(
            "Device is not configured. Boot normally, then press and hold GPIO 0 during startup.");
    }

    // 1. OLED in BME280 zagon (Arduino)
    init_sensors_and_display();

    // 2. DSP inicializacija (ESP-IDF / esp-dsp)
    if (!doa.init()) {
        Serial.println("GCC-PHAT inicializacija ni uspela!");
        while (true) {
            delay(1000);
        }
    }
    if (!init_i2s_microphones(44100)) {
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
    
    // Ustvari FreeRTOS objekte za prenos rezultatov in zaščito DSP konfiguracije
    display_queue = xQueueCreate(10, sizeof(DoaResult));
    network_queue = xQueueCreate(10, sizeof(DoaResult));
    doa_mutex = xSemaphoreCreateMutex();
    config_mutex = xSemaphoreCreateMutex();
    if (display_queue == nullptr || network_queue == nullptr ||
        doa_mutex == nullptr || config_mutex == nullptr) {
        Serial.println("FreeRTOS objektov ni bilo mogoce ustvariti!");
        while (true) {
            delay(1000);
        }
    }
    if (!apply_dsp_calibration(current_temperature.load())) {
        Serial.println("Initial DSP calibration is invalid");
        while (true) {
            delay(1000);
        }
    }

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