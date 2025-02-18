#include <esp_log.h>
#include <iostream>

#include <sdkconfig.h>

#include "modules/GPS.h"

static auto TAG = "DreamPilot";

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Starting DreamPilot v0.0.1");
    std::cout << "Hello, World!" << std::endl;

    GPS *gps = new GPS();
}
