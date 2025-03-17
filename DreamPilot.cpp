#include <esp_log.h>
#include <iostream>

#include <sdkconfig.h>

#include "modules/GPS/IGPSModule.h"
#include "modules/GPS/NEO6M.h"

#include "modules/IMU/IIMUModule.h"

static auto TAG = "DreamPilot";

extern "C" _Noreturn void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("GPS", ESP_LOG_INFO);
    ESP_LOGI(TAG, "Starting DreamPilot v0.0.1");
    std::cout << "Hello, World!" << std::endl;
    IGPSModule *gps = new NEO6M();
    gps->start();

    while (true)
    {
        gps->printLastData();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
