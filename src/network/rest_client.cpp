#include "network/rest_client.h"
#include "ui/serial_ui.h"
#include <WiFi.h>
#include <atomic>

RestClient::RestClient() : http_initialized(false) {}

RestClient::~RestClient() {
    close();
}

void RestClient::close() {
    if (http_initialized) {
        http.end();
        http_initialized = false;
        current_url = "";
        current_key = "";
    }
}

bool RestClient::sendBatch(const uint8_t *payload, size_t payload_size,
                           const String &api_url, const String &api_key) {
    if (WiFi.status() != WL_CONNECTED || api_url.isEmpty() || api_key.isEmpty() ||
        payload == nullptr || payload_size == 0) {
        return false;
    }

    if (!http_initialized || api_url != current_url || api_key != current_key) {
        close();
        http.setReuse(true);
        http.useHTTP10(false);
        http.setTimeout(5000);
        current_url = api_url;
        current_key = api_key;
        http_initialized = true;
    }

    String url = current_url + "/measurements";
    if (!http.begin(url)) {
        Serial.println("HTTP client initialization failed");
        http_initialized = false;
        return false;
    }
    http.addHeader("Content-Type", "application/octet-stream");
    http.addHeader("Connection", "keep-alive");
    http.addHeader("x-api-key", current_key);

    const int httpResponseCode = http.POST(const_cast<uint8_t*>(payload), payload_size);

    if (httpResponseCode < 200 || httpResponseCode >= 300) {
        const String response = http.getString();
        SerialUI::printApiBatchError(httpResponseCode, response.c_str());
        close();
        return false;
    }

    http.end(); // With setReuse(true), drains unread 2xx body while keeping TCP socket alive
    SerialUI::printApiBatchSuccess(httpResponseCode, payload_size);
    return true;
}

bool RestClient::sendBatch(const NetworkMeasurement *measurements, size_t count,
                           const String &api_url, const String &api_key,
                           float db_spl_offset) {
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

    return sendBatch(payload, payload_size, api_url, api_key);
}

bool RestClient::fetchCalibration(const String &api_url, const String &api_key,
                                  uint32_t device_id,
                                  DeviceConfig &config,
                                  std::atomic<int64_t> &server_utc_offset_ms,
                                  std::atomic<bool> &server_time_valid) {
    if (device_id == 0 || WiFi.status() != WL_CONNECTED || api_url.isEmpty() || api_key.isEmpty()) {
        return false;
    }

    HTTPClient cal_http;
    String url = api_url + "/devices/" + String(device_id) + "/calibration";
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
        SerialUI::printApiCalibrationError(status, response.c_str(), device_id);
        return false;
    }

    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, response.c_str());
    if (error) {
        Serial.printf("Calibration response JSON error: %s\n", error.c_str());
        return false;
    }

    if (!doc["device_id"].isNull() && doc["device_id"].as<uint32_t>() != device_id) {
        Serial.printf("Calibration response device_id mismatch: expected %u, got %u\n",
                      device_id, doc["device_id"].as<uint32_t>());
        return false;
    }

    if (doc["microphone_distance_m"].isNull() ||
        doc["correlation_threshold"].isNull() ||
        (doc["sound_level_threshold_db"].isNull() && doc["sound_level_threshold_db_spl"].isNull()) ||
        doc["sample_rate"].isNull() ||
        doc["window_size"].isNull()) {
        Serial.println("Calibration response missing required fields");
        return false;
    }

    DeviceConfig candidate = config;
    candidate.device_id = device_id;
    candidate.microphone_distance_m = doc["microphone_distance_m"].as<float>();
    candidate.correlation_threshold = doc["correlation_threshold"].as<float>();
    if (!doc["sound_level_threshold_db"].isNull()) {
        candidate.sound_level_threshold_db_spl = doc["sound_level_threshold_db"].as<float>();
    } else {
        candidate.sound_level_threshold_db_spl = doc["sound_level_threshold_db_spl"].as<float>();
    }
    candidate.sample_rate = doc["sample_rate"].as<uint32_t>();
    candidate.window_size = doc["window_size"].as<uint32_t>();

    if (!doc["db_spl_offset"].isNull()) {
        candidate.db_spl_offset = doc["db_spl_offset"].as<float>();
    }

    if (!doc["adaptive_threshold_enabled"].isNull()) {
        candidate.adaptive_threshold_enabled = doc["adaptive_threshold_enabled"].as<bool>();
    } else if (!doc["adaptive_threshold"].isNull()) {
        candidate.adaptive_threshold_enabled = doc["adaptive_threshold"].as<bool>();
    }

    if (!doc["snr_margin_db"].isNull()) {
        candidate.snr_margin_db = doc["snr_margin_db"].as<float>();
    } else if (!doc["snr_margin"].isNull()) {
        candidate.snr_margin_db = doc["snr_margin"].as<float>();
    }

    if (!doc["angle_offset_deg"].isNull()) {
        candidate.angle_offset_deg = doc["angle_offset_deg"].as<float>();
    } else if (!doc["angle_offset"].isNull()) {
        candidate.angle_offset_deg = doc["angle_offset"].as<float>();
    }

    if (!validate_device_config(candidate)) {
        Serial.println("Calibration response contains invalid or unsupported values");
        return false;
    }

    JsonVariant server_time_value = doc["server_time_unix_ms"];
    if (!server_time_value.isNull()) {
        const uint64_t server_time_ms = server_time_value.as<uint64_t>();
        const int64_t midpoint_ms = request_start_ms + (request_end_ms - request_start_ms) / 2;
        if (server_time_ms >= static_cast<uint64_t>(MIN_VALID_EPOCH) * 1000ULL && midpoint_ms >= 0) {
            const int64_t new_offset_ms = static_cast<int64_t>(server_time_ms) - midpoint_ms;
            if (!server_time_valid.load()) {
                server_utc_offset_ms.store(new_offset_ms);
                server_time_valid.store(true);
            }
        } else {
            Serial.println("Calibration server time is invalid");
        }
    }

    config = candidate;
    SerialUI::printApiCalibrationSuccess(candidate.device_id, candidate.sample_rate,
                                         candidate.microphone_distance_m,
                                         candidate.correlation_threshold);
    return true;
}
