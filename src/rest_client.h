#pragma once
#include <Arduino.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <atomic>
#include "rest_protocol.h"
#include "device_config.h"

class GccPhat;

class RestClient {
public:
    RestClient();
    ~RestClient();

    bool sendBatch(const NetworkMeasurement *measurements, size_t count,
                   const String &api_url, const String &api_key,
                   float db_spl_offset);

    bool fetchCalibration(const String &api_url, const String &api_key,
                          uint32_t device_id,
                          DeviceConfig &config,
                          std::atomic<int64_t> &server_utc_offset_ms,
                          std::atomic<bool> &server_time_valid);

    void close();

private:
    HTTPClient http;
    bool http_initialized;
};
