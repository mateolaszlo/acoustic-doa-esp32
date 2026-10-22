#include "rest_client.h"
#include <WiFi.h>

static constexpr time_t MIN_VALID_EPOCH = 1704067200; // 2024-01-01 UTC

RestClient::RestClient() : http_initialized(false) {}

RestClient::~RestClient() {
    close();
}

void RestClient::close() {
    if (http_initialized) {
        http.end();
        http_initialized = false;
    }
}

bool RestClient::sendBatch(const NetworkMeasurement *measurements, size_t count,
                           const String &api_url, const String &api_key,
                           float db_spl_offset) {
    if (WiFi.status() != WL_CONNECTED || api_url.isEmpty() || api_key.isEmpty()) {
        return false;
    }

    if (!http_initialized) {
        String url = api_url + "/meritve";
        if (!http.begin(url)) {
            Serial.println("HTTP client initialization failed");
            return false;
        }
        http.setReuse(true);
        http.useHTTP10(false);
        http.setTimeout(5000);
        http.addHeader("Content-Type", "application/octet-stream");
        http.addHeader("Connection", "keep-alive");
        http.addHeader("x-api-key", api_key);
        http_initialized = true;
    }

    constexpr size_t max_payload_size =
        MEASUREMENT_BATCH_HEADER_SIZE +
        NETWORK_BATCH_MAX_MEASUREMENTS * MEASUREMENT_RECORD_SIZE;
    uint8_t payload[max_payload_size];
    const size_t payload_size = serialize_measurement_batch(
        measurements, count, payload, sizeof(payload), db_spl_offset);
    if (payload_size == 0) {
        Serial.println("Measurement batch serialization failed");
        return false;
    }

    const int httpResponseCode = http.POST(payload, payload_size);

    if (httpResponseCode < 200 || httpResponseCode >= 300) {
        const String response = http.getString();
        Serial.printf("HTTP POST /meritve failed: %d, response: %s\n",
                      httpResponseCode, response.c_str());
        http.end();
        http_initialized = false;
        return false;
    }

    Serial.printf("Measurement batch accepted: HTTP %d, count: %u\n",
                  httpResponseCode, static_cast<unsigned>(count));
    return true;
}

bool RestClient::fetchCalibration(const String &api_url, const String &api_key,
                                  uint32_t device_id,
                                  DeviceConfig &config,
                                  std::atomic<int64_t> &server_utc_offset_ms,
                                  std::atomic<bool> &server_time_valid) {
    if (device_id == 0 || WiFi.status() != WL_CONNECTED) {
        return false;
    }

    HTTPClient cal_http;
    String url = api_url + "/naprave/" + String(device_id) + "/kalibracija";
    if (!cal_http.begin(url)) {
        Serial.println("Calibration HTTP client initialization failed");
        return false;
    }

    cal_http.setTimeout(5000);
    cal_http.addHeader("Accept", "application/json");
    cal_http.addHeader("x-api-key", api_key);

    const int64_t request_start_ms = static_cast<int64_t>(esp_timer_get_time() / 1000ULL);
    int status = cal_http.GET();
    String response = cal_http.getString();
    const int64_t request_end_ms = static_cast<int64_t>(esp_timer_get_time() / 1000ULL);
    cal_http.end();

    if (status < 200 || status >= 300) {
        Serial.printf("Calibration GET failed: %d, response: %s\n", status, response.c_str());
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
        const uint64_t server_time_ms = server_time_value.as<uint64_t>();
        const int64_t midpoint_ms = request_start_ms + (request_end_ms - request_start_ms) / 2;
        if (server_time_ms >= static_cast<uint64_t>(MIN_VALID_EPOCH) * 1000ULL && midpoint_ms >= 0) {
            server_utc_offset_ms.store(static_cast<int64_t>(server_time_ms) - midpoint_ms);
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
    float offset = doc["db_spl_offset"] | config.db_spl_offset;

    bool adaptive_enabled = config.adaptive_threshold_enabled;
    if (!doc["prilagodljivi_prag"].isNull()) {
        adaptive_enabled = doc["prilagodljivi_prag"].as<bool>();
    } else if (!doc["adaptive_threshold"].isNull()) {
        adaptive_enabled = doc["adaptive_threshold"].as<bool>();
    } else if (!doc["adaptive_threshold_enabled"].isNull()) {
        adaptive_enabled = doc["adaptive_threshold_enabled"].as<bool>();
    }

    float snr_margin = config.snr_margin_db;
    if (!doc["snr_rob_db"].isNull()) {
        snr_margin = doc["snr_rob_db"].as<float>();
    } else if (!doc["snr_margin_db"].isNull()) {
        snr_margin = doc["snr_margin_db"].as<float>();
    } else if (!doc["snr_margin"].isNull()) {
        snr_margin = doc["snr_margin"].as<float>();
    }

    float angle_offset = config.angle_offset_deg;
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
        !std::isfinite(distance) || !std::isfinite(correlation) ||
        !std::isfinite(spl_threshold) || !std::isfinite(offset) ||
        !std::isfinite(snr_margin) || snr_margin < SNR_MARGIN_MIN_DB || snr_margin > SNR_MARGIN_MAX_DB ||
        !std::isfinite(angle_offset) || angle_offset < ANGLE_OFFSET_MIN_DEG || angle_offset > ANGLE_OFFSET_MAX_DEG) {
        Serial.println("Calibration response contains invalid or unsupported values");
        return false;
    }

    config.microphone_distance_m = distance;
    config.correlation_threshold = correlation;
    config.sound_level_threshold_db_spl = spl_threshold;
    config.db_spl_offset = offset;
    config.sample_rate = sample_rate;
    config.window_size = window_size;
    config.adaptive_threshold_enabled = adaptive_enabled;
    config.snr_margin_db = snr_margin;
    config.angle_offset_deg = angle_offset;

    return true;
}
