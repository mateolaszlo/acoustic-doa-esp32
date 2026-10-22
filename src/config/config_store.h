#pragma once
#include "config/device_config.h"

bool load_device_config(DeviceConfig &config);
void save_device_config(const DeviceConfig &config);
bool is_device_configured(const DeviceConfig &config);
