#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_BME280.h>
#include <esp_task_wdt.h>
#include <atomic>
#include <ctime>
#include <cstring>
#include <sys/time.h>

#include "config/app_config.h"
#include "config/device_config.h"
#include "config/config_store.h"
#include "audio/post_checker.h"
#include "ui/radar_ui.h"
#include "ui/serial_ui.h"
#include "network/setup_portal.h"
#include "network/rest_protocol.h"
#include "network/rest_client.h"
#include "audio/gcc_phat.h"
#include "audio/i2s_sampler.h"
#include "ui/display_filter.h"

// Globalne periferne enote
static Adafruit_BME280 bme;
static RadarDisplay radar_display;
static GccPhat doa(DEFAULT_SAMPLE_RATE, DEFAULT_MIC_DISTANCE_M, DEFAULT_TEMPERATURE_C);

// Deljene spremenljivke med jedroma (Thread-safe)
static std::atomic<float> current_sound_speed(DEFAULT_SOUND_SPEED_MPS);
static std::atomic<float> current_temperature(DEFAULT_TEMPERATURE_C);
static std::atomic<float> current_humidity(DEFAULT_HUMIDITY_PCT);
static std::atomic<float> current_pressure(DEFAULT_PRESSURE_PA);
static std::atomic<float> db_spl_offset(DEFAULT_DB_SPL_OFFSET);
static std::atomic<bool> server_available(false);
static std::atomic<int64_t> ntp_utc_offset_ms(0);
static std::atomic<bool> ntp_time_valid(false);
static std::atomic<int64_t> server_utc_offset_ms(0);
static std::atomic<bool> server_time_valid(false);
static std::atomic<float> current_noise_floor_db(-100.0f);
static std::atomic<bool> adaptive_threshold_active(DEFAULT_ADAPTIVE_THRESHOLD_ENABLED);

// Diagnostični števci
static std::atomic<uint32_t> dsp_deadline_misses(0);
static std::atomic<uint32_t> dsp_i2s_read_failures(0);
static std::atomic<uint32_t> display_queue_drops(0);
static std::atomic<uint32_t> network_queue_drops(0);
static std::atomic<uint32_t> dsp_blocks_processed(0);
static std::atomic<uint32_t> dsp_max_processing_us(0);

// Nastavitve naprave in FreeRTOS sinhronizacija
static DeviceConfig device_config;
static bool device_configured = false;
static bool bme_initialized = false;

static QueueHandle_t display_queue = nullptr;
static QueueHandle_t network_queue = nullptr;
static SemaphoreHandle_t doa_mutex = nullptr;
static SemaphoreHandle_t config_mutex = nullptr;
static SemaphoreHandle_t i2s_mutex = nullptr;

static uint32_t audio_block_duration_us(uint32_t sample_rate, uint32_t window_size) {
    return static_cast<uint32_t>(
        (static_cast<uint64_t>(window_size) * 1000000ULL) / sample_rate);
}

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

    SerialUI::printSelfTestReport(res, sample_rate);

    return res;
}

static void handle_hardware_degraded(const BootSelfTestResult &post) {
    SerialUI::printHardwareDegradedAlert();

    radar_display.drawHardwareDegraded(post);

    uint32_t press_start_ms = 0;
    while (true) {
        if (digitalRead(SETUP_BUTTON_PIN) == LOW) {
            if (press_start_ms == 0) {
                press_start_ms = millis();
            } else if (millis() - press_start_ms >= 100) {
                SetupPortal::enter(device_config, radar_display, [](const DeviceConfig &saved_cfg) {
                    save_device_config(saved_cfg);
                });
                radar_display.drawHardwareDegraded(post);
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
            radar_display.drawBootPrompt(remaining, total_ms, post_result);
        }
        delay(10);
    }

    if (button_pressed) {
        SetupPortal::enter(device_config, radar_display, [](const DeviceConfig &saved_cfg) {
            save_device_config(saved_cfg);
        });
    }

    if (!post_result.all_nominal) {
        handle_hardware_degraded(post_result);
    }
}

static bool enqueue_latest(QueueHandle_t queue, const MultiDoaResult &result,
                           std::atomic<uint32_t> &drop_counter) {
    if (xQueueSend(queue, &result, 0) == pdPASS) {
        return true;
    }

    MultiDoaResult discarded;
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
    const int64_t monotonic_ms = static_cast<int64_t>(esp_timer_get_time() / 1000ULL);
    if (ntp_time_valid.load()) {
        const int64_t estimated_utc_ms = monotonic_ms + ntp_utc_offset_ms.load();
        if (estimated_utc_ms > 0) {
            return {result, static_cast<uint64_t>(estimated_utc_ms), CLOCK_FLAG_UTC_SYNCHRONIZED};
        }
    }

    if (server_time_valid.load()) {
        const int64_t estimated_utc_ms = monotonic_ms + server_utc_offset_ms.load();
        if (estimated_utc_ms > 0) {
            return {result, static_cast<uint64_t>(estimated_utc_ms),
                    static_cast<uint16_t>(CLOCK_FLAG_UTC_SYNCHRONIZED | CLOCK_FLAG_SERVER_SYNCHRONIZED)};
        }
    }

    return {result, static_cast<uint64_t>(monotonic_ms), CLOCK_FLAG_MONOTONIC_UPTIME};
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

    SerialUI::printDspDiagnostics(
        dsp_blocks_processed.load(),
        dsp_deadline_misses.load(),
        dsp_max_processing_us.load(),
        audio_block_duration_us(sample_rate, window_size),
        dsp_i2s_read_failures.load(),
        display_queue_drops.load(),
        network_queue_drops.load(),
        uxTaskGetStackHighWaterMark(nullptr));
}

static void initialize_task_watchdog() {
    esp_err_t result = esp_task_wdt_init(TASK_WATCHDOG_TIMEOUT_SECONDS, true);
    if (result != ESP_OK && result != ESP_ERR_INVALID_STATE) {
        Serial.printf("Task watchdog initialization failed: %s\n", esp_err_to_name(result));
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
        float min_spl_threshold;
        if (xSemaphoreTake(config_mutex, pdMS_TO_TICKS(CONFIG_MUTEX_FAST_TIMEOUT_MS)) != pdTRUE) {
            esp_task_wdt_reset();
            continue;
        }
        sample_rate = device_config.sample_rate;
        window_size = device_config.window_size;
        min_spl_threshold = device_config.sound_level_threshold_db_spl;
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
            MultiDoaResult multi_res = doa.processMulti(ch0_buf, ch1_buf, ch2_buf, ch3_buf);
            const float noise_floor = doa.getNoiseFloorDb();
            xSemaphoreGive(doa_mutex);
            current_noise_floor_db.store(noise_floor);

            const uint32_t processing_us = micros() - process_start;
            const uint32_t block_budget_us = audio_block_duration_us(sample_rate, window_size);
            dsp_blocks_processed.fetch_add(1);
            uint32_t previous_max = dsp_max_processing_us.load();
            while (processing_us > previous_max &&
                   !dsp_max_processing_us.compare_exchange_weak(previous_max, processing_us)) {
            }
            if (processing_us > block_budget_us) {
                dsp_deadline_misses.fetch_add(1);
            }

            const float sound_spl = multi_res.rms_db + db_spl_offset.load();
            if (multi_res.valid && multi_res.count > 0 && sound_spl >= min_spl_threshold) {
                enqueue_latest(display_queue, multi_res, display_queue_drops);

                // Enqueue each detected source from this block with identical timestamp
                NetworkMeasurement base_meas = make_network_measurement(multi_res.sources[0]);
                for (uint8_t i = 0; i < multi_res.count; ++i) {
                    NetworkMeasurement meas = base_meas;
                    meas.result = multi_res.sources[i];
                    enqueue_network_measurement(meas);
                }
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
    const float correlation_threshold = (device_config.correlation_threshold < DEFAULT_CORRELATION_THRESHOLD)
                                            ? DEFAULT_CORRELATION_THRESHOLD
                                            : device_config.correlation_threshold;
    const float sound_level_threshold_db_spl = device_config.sound_level_threshold_db_spl;
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
    const bool dsp_ready = doa.configure(static_cast<float>(sample_rate), window_size);
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
    const bool dsp_rollback = doa.configure(static_cast<float>(previous_rate), previous_window);
    xSemaphoreGive(doa_mutex);
    if (xSemaphoreTake(i2s_mutex, portMAX_DELAY) == pdTRUE) {
        init_i2s_microphones(previous_rate);
        xSemaphoreGive(i2s_mutex);
    }
    return dsp_rollback;
}

void network_task(void *param) {
    NetworkMeasurement batch[NETWORK_BATCH_MAX_MEASUREMENTS];
    unsigned long last_calibration_refresh = 0;
    unsigned long last_request_ms = 0;
    RestClient rest_client;
    bool ntp_started = false;
    esp_task_wdt_add(nullptr);

    while (true) {
        if (WiFi.status() != WL_CONNECTED) {
            server_available.store(false);
            ntp_started = false;
            ntp_time_valid.store(false);
            rest_client.close();
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
            const int64_t monotonic_ms = static_cast<int64_t>(esp_timer_get_time() / 1000ULL);
            struct timeval time_value;
            gettimeofday(&time_value, nullptr);
            const int64_t utc_ms = static_cast<int64_t>(time_value.tv_sec) * 1000LL +
                                   static_cast<int64_t>(time_value.tv_usec) / 1000LL;
            if (utc_ms >= static_cast<int64_t>(MIN_VALID_EPOCH) * 1000LL) {
                ntp_utc_offset_ms.store(utc_ms - monotonic_ms);
                ntp_time_valid.store(true);
            }
        }

        if (last_calibration_refresh == 0 ||
            millis() - last_calibration_refresh >= CONFIG_REFRESH_INTERVAL_MS) {
            DeviceConfig remote_cfg;
            if (xSemaphoreTake(config_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                remote_cfg = device_config;
                xSemaphoreGive(config_mutex);
            }

            if (rest_client.fetchCalibration(remote_cfg.api_url, remote_cfg.api_key, remote_cfg.device_id,
                                             remote_cfg, server_utc_offset_ms, server_time_valid)) {
                DeviceConfig previous;
                xSemaphoreTake(config_mutex, portMAX_DELAY);
                previous = device_config;
                xSemaphoreGive(config_mutex);

                bool reconfig_ok = true;
                if (remote_cfg.sample_rate != previous.sample_rate || remote_cfg.window_size != previous.window_size) {
                    reconfig_ok = reconfigure_audio(remote_cfg.sample_rate, remote_cfg.window_size);
                }

                if (reconfig_ok) {
                    xSemaphoreTake(config_mutex, portMAX_DELAY);
                    device_config = remote_cfg;
                    xSemaphoreGive(config_mutex);

                    if (apply_dsp_calibration(current_temperature.load(),
                                              current_humidity.load(),
                                              current_pressure.load())) {
                        save_device_config(remote_cfg);
                        Serial.println("Remote calibration applied");
                    } else {
                        xSemaphoreTake(config_mutex, portMAX_DELAY);
                        device_config = previous;
                        xSemaphoreGive(config_mutex);
                        reconfigure_audio(previous.sample_rate, previous.window_size);
                    }
                }
            }
            last_calibration_refresh = millis();
        }

        size_t batch_count = 0;
        if (xQueueReceive(network_queue, &batch[batch_count], pdMS_TO_TICKS(100)) == pdTRUE) {
            ++batch_count;
            const unsigned long flush_deadline = millis() + NETWORK_BATCH_FLUSH_INTERVAL_MS;
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
                if (xQueueReceive(network_queue, &batch[batch_count], wait_ticks) != pdTRUE) {
                    break;
                }
                ++batch_count;
            }

            const unsigned long now = millis();
            if (last_request_ms != 0 && now - last_request_ms < NETWORK_MAX_REQUEST_INTERVAL_MS) {
                vTaskDelay(pdMS_TO_TICKS(NETWORK_MAX_REQUEST_INTERVAL_MS - (now - last_request_ms)));
            }

            String current_api_url;
            String current_api_key;
            if (xSemaphoreTake(config_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                current_api_url = device_config.api_url;
                current_api_key = device_config.api_key;
                xSemaphoreGive(config_mutex);
            }

            bool sent = false;
            for (int attempt = 1; attempt <= HTTP_RETRY_COUNT && !sent; ++attempt) {
                sent = rest_client.sendBatch(batch, batch_count, current_api_url, current_api_key, db_spl_offset.load());
                if (!sent && attempt < HTTP_RETRY_COUNT) {
                    vTaskDelay(pdMS_TO_TICKS(1000 * attempt));
                }
            }
            server_available.store(sent);
            if (!sent) {
                Serial.println("Measurement batch dropped after RAM retry limit");
            } else {
                last_request_ms = millis();
            }
        }
        esp_task_wdt_reset();
    }
}

void init_sensors_and_display() {
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_BUS_SPEED_HZ);

    radar_display.init();

    bme_initialized = bme.begin(BME280_ADDR_PRIMARY, &Wire);
    if (!bme_initialized) {
        bme_initialized = bme.begin(BME280_ADDR_SECONDARY, &Wire);
        if (!bme_initialized) {
            Serial.printf("BME280 senzor ni bil najden! Uporabljam privzeto c=%.1f m/s\n",
                          DEFAULT_SOUND_SPEED_MPS);
            SerialUI::printI2cScan();
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

void update_environmental_compensation() {
    static unsigned long last_read = 0;
    if (millis() - last_read > SENSOR_READ_INTERVAL_MS) {
        last_read = millis();

        if (!bme_initialized) {
            return;
        }

        float t = bme.readTemperature();
        float rh = bme.readHumidity();
        float p = bme.readPressure();

        if (std::isfinite(t) && std::isfinite(rh) && std::isfinite(p)) {
            current_temperature.store(t);
            current_humidity.store(rh);
            current_pressure.store(p);

            float c = GccPhat::calculateSoundSpeed(t, rh, p);
            current_sound_speed.store(c);

            apply_dsp_calibration(t, rh, p);
        }
    }
}

void sensor_display_task(void *param) {
    MultiDoaResult event;
    MultiDoaResult last_event;
    last_event.valid = false;
    last_event.count = 0;
    last_event.rms_db = -100.0f;
    for (size_t s = 0; s < DOA_MAX_SOURCES; ++s) {
        last_event.sources[s] = {0.0f, 0.0f, 0.0f, -100.0f, false};
    }
    bool last_server_available = server_available.load();
    MultiTargetDisplayTracker tracker;
    TrackedTarget tracked_targets[DOA_MAX_SOURCES];
    size_t prev_rendered_tracks = 0;
    bool prev_rendered_blips = false;
    uint32_t last_idle_refresh_ms = 0;

    while (true) {
        update_environmental_compensation();

        const uint32_t now_ms = millis();
        bool needs_redraw = false;

        if (xQueueReceive(display_queue, &event, pdMS_TO_TICKS(DISPLAY_QUEUE_TIMEOUT_MS))) {
            last_event = event;
            tracker.update(event, now_ms);
            needs_redraw = true;
            last_idle_refresh_ms = now_ms;

            const float calibrated_db = event.rms_db + db_spl_offset.load();
            for (uint8_t s = 0; s < event.count; ++s) {
                SerialUI::printDoaEvent(event.sources[s].angle_deg,
                                        event.sources[s].confidence,
                                        calibrated_db,
                                        s,
                                        event.count);
            }
        } else {
            // Tišina (brez novih paketov): preveri ali sledi ali fosforne točke še razpadajo
            // oziroma ali je potekel 1-sekundni interval za osvežitev nivoja šuma okolice
            const size_t active_tracks = tracker.getActiveTracks(tracked_targets, DOA_MAX_SOURCES, now_ms);
            const bool active_blips = radar_display.hasActiveDecayBlips(now_ms);
            const bool current_server_available = server_available.load();
            const bool idle_refresh_due = (now_ms - last_idle_refresh_ms >= DISPLAY_IDLE_REFRESH_INTERVAL_MS);

            if (active_tracks > 0 || active_blips ||
                prev_rendered_tracks > 0 || prev_rendered_blips ||
                current_server_available != last_server_available ||
                idle_refresh_due) {
                needs_redraw = true;
                if (idle_refresh_due) {
                    last_idle_refresh_ms = now_ms;
                }
            }
        }

        if (needs_redraw && radar_display.isInitialized()) {
            last_server_available = server_available.load();
            const size_t num_tracks = tracker.getActiveTracks(tracked_targets, DOA_MAX_SOURCES, now_ms);
            float angles[DOA_MAX_SOURCES];
            float confs[DOA_MAX_SOURCES];
            for (size_t i = 0; i < num_tracks; ++i) {
                angles[i] = tracked_targets[i].smoothed_angle_deg;
                confs[i] = tracked_targets[i].confidence;
            }

            const float current_noise = current_noise_floor_db.load();
            const float offset = db_spl_offset.load();
            float current_rms = (num_tracks > 0) ? last_event.rms_db : current_noise;
            if (current_rms < -100.0f) current_rms = current_noise;
            const float calibrated_db = current_rms + offset;

            float snr_db = 0.0f;
            if (num_tracks > 0 && current_noise > -99.0f) {
                snr_db = last_event.rms_db - current_noise;
                if (snr_db < 0.0f) snr_db = 0.0f;
            }

            radar_display.drawRadarMulti(angles, confs, num_tracks,
                                        current_temperature.load(), current_humidity.load(),
                                        current_pressure.load(), current_sound_speed.load(),
                                        calibrated_db, last_server_available,
                                        adaptive_threshold_active.load(),
                                        snr_db, DEFAULT_SNR_MARGIN_DB);

            prev_rendered_tracks = num_tracks;
            prev_rendered_blips = radar_display.hasActiveDecayBlips(now_ms);
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
    device_configured = load_device_config(device_config);

    // 1. Zagon zaslona in BME280 senzorja
    init_sensors_and_display();

    // 2. Zagonski preizkus strojne opreme (POST)
    BootSelfTestResult post_res = run_boot_self_test(device_config.sample_rate);

    // 3. Preveri gumb za nastavitve (prikaze POST rezultate in odstevalnik na zaslonu)
    check_setup_button(post_res);
    if (!device_configured) {
        Serial.println("Device is not configured. Boot normally, then press and hold GPIO 0 during startup.");
    }

    // Po izhodu iz boot okna narisi zacetni radar UI pred zagonom nalog
    if (radar_display.isInitialized()) {
        radar_display.drawRadar(0.0f, current_temperature.load(),
                                current_humidity.load(),
                                current_pressure.load(),
                                current_sound_speed.load(), 0.0f,
                                server_available.load(),
                                adaptive_threshold_active.load());
    }

    // Ustvari FreeRTOS objekte za prenos rezultatov in zaščito DSP konfiguracije
    display_queue = xQueueCreate(DISPLAY_QUEUE_LENGTH, sizeof(MultiDoaResult));
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

    if (device_configured) {
        WiFi.setAutoReconnect(true);
        WiFi.begin(device_config.wifi_ssid.c_str(), device_config.wifi_password.c_str());
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
