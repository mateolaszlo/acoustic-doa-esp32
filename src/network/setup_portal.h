#pragma once
#include <functional>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include "config/device_config.h"
#include "ui/radar_ui.h"

class SetupPortal {
public:
    static void enter(DeviceConfig &config, RadarDisplay &radar,
                      std::function<void(const DeviceConfig&)> on_save);

private:
    static String generateHtmlPage(const DeviceConfig &config);
};
