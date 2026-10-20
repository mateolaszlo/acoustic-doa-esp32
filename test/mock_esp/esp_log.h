#pragma once
#include <cstdio>
#include <cstdint>

#define ESP_LOGE(tag, format, ...) printf("[ERROR][%s] " format "\n", tag, ##__VA_ARGS__)
#define ESP_LOGW(tag, format, ...) printf("[WARN][%s] " format "\n", tag, ##__VA_ARGS__)
#define ESP_LOGI(tag, format, ...) printf("[INFO][%s] " format "\n", tag, ##__VA_ARGS__)
#define ESP_LOGD(tag, format, ...)

typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
