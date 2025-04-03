//
// Created by stikper on 24.03.25.
//

#include "MPU6050.h"

#include <cmath>

#include <esp_log.h>
#include <esp_timer.h>
#include <driver/i2c.h>
#include <driver/i2c_master.h>

MPU6050::MPU6050(): cfg{}
{
    TAG = "MPU6050";
    ESP_LOGI(TAG.data(), "Initializing...");

    // TODO: Remove hardcode
    // TODO: Add verbose logging
    // TODO: Add malloc checks
    // TODO: config sequence
    // Setting configuration
    cfg.i2c_freq = 400000;
    cfg.i2c_port_num = I2C_NUM_0;
    cfg.i2c_sda = GPIO_NUM_18;
    cfg.i2c_scl = GPIO_NUM_5;
    cfg.rate = 100;
    cfg.imu_task_priority = 11;
    cfg.imu_task_stack_size = 4096;
    cfg.accel_scale = 3; // ±8g
    cfg.gyro_scale = 3; // ±1000°/s

    imu_task_handle = nullptr;

    bus_handle = nullptr;
    dev_handle = nullptr;

    running = false;

    ESP_LOGI(TAG.data(), "Module is ready to start!");
}

MPU6050::~MPU6050()
{
    stop();
}

esp_err_t MPU6050::initI2C()
{
    // Check if this I2C already initialized
    esp_err_t ret = i2c_master_get_bus_handle(cfg.i2c_port_num, &bus_handle);

    if (ret != ESP_OK)
    {
        // I2C config
        i2c_master_bus_config_t i2c_mst_config = {};
        i2c_mst_config.clk_source = I2C_CLK_SRC_DEFAULT;
        i2c_mst_config.i2c_port = cfg.i2c_port_num;
        i2c_mst_config.sda_io_num = cfg.i2c_sda;
        i2c_mst_config.scl_io_num = cfg.i2c_scl;
        i2c_mst_config.glitch_ignore_cnt = 7;
        i2c_mst_config.flags.enable_internal_pullup = true;

        ret = i2c_new_master_bus(&i2c_mst_config, &bus_handle);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG.data(), "Failed to initialize I2C bus");
            return ESP_FAIL;
        }
    }

    ESP_LOGI(TAG.data(), "I2C bus initialized");

    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = 0x68;
    dev_cfg.scl_speed_hz = 400000;

    ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG.data(), "Failed to add I2C device");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG.data(), "I2C device initialized");
    return ESP_OK;
}

esp_err_t MPU6050::removeI2C()
{
    //TODO: remove mst or not??

    esp_err_t ret = i2c_master_bus_rm_device(dev_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG.data(), "Failed to remove I2C device");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG.data(), "I2C device removed");
    return ESP_OK;
}

esp_err_t MPU6050::configMPU6050() const
{
    // Unset sleep mode
    uint8_t buf[2] = {0x6B, 0x00};
    esp_err_t ret = i2c_master_transmit(dev_handle, buf, 2, -1);
    if (ret != ESP_OK) return ret;

    // Config accelerometer
    buf[0] = 0x1C;
    buf[1] = cfg.accel_scale << 3;
    ret = i2c_master_transmit(dev_handle, buf, 2, -1);
    if (ret != ESP_OK) return ret;

    // Config gyroscope
    buf[0] = 0x1B;
    buf[1] = cfg.gyro_scale << 3;
    ret = i2c_master_transmit(dev_handle, buf, 2, -1);
    if (ret != ESP_OK) return ret;

    return ESP_OK;
}

esp_err_t MPU6050::getData()
{
    float accel[3];
    float gyro[3];
    float temp[1];

    uint8_t data[14];
    uint8_t data_reg = 0x3B;
    //TODO!!! FIX CRASHES
    //TODO!!! Interrupt wdt timeout on CPU0
    esp_err_t ret = i2c_master_transmit_receive(dev_handle, &data_reg, 1, data, 14, 10);
    if (ret != ESP_OK) return ret;

    int64_t timestamp = esp_timer_get_time();

    auto ax = static_cast<int16_t>(data[0] << 8 | data[1]);
    auto ay = static_cast<int16_t>(data[2] << 8 | data[3]);
    auto az = static_cast<int16_t>(data[4] << 8 | data[5]);

    auto t = static_cast<int16_t>(data[6] << 8 | data[7]);

    auto gx = static_cast<int16_t>(data[8] << 8 | data[9]);
    auto gy = static_cast<int16_t>(data[10] << 8 | data[11]);
    auto gz = static_cast<int16_t>(data[12] << 8 | data[13]);

    const float accel_div = 16384.0f / powf(2.0f, cfg.accel_scale);
    accel[0] = static_cast<float>(ax) / accel_div * 9.81f;
    accel[1] = static_cast<float>(ay) / accel_div * 9.81f;
    accel[2] = static_cast<float>(az) / accel_div * 9.81f;

    const float gyro_div = 131.0f / powf(2.0f, cfg.gyro_scale);
    gyro[0] = static_cast<float>(gx) / gyro_div;
    gyro[1] = static_cast<float>(gy) / gyro_div;
    gyro[2] = static_cast<float>(gz) / gyro_div;

    temp[0] = static_cast<float>(t) / 340.0f + 36.53f;

    updateData(timestamp, accel, gyro, temp);

    return ESP_OK;
}

void MPU6050::imuTaskWrapper(void* param)
{
    auto* imu = static_cast<MPU6050*>(param);

    imu->imuTask();
}

_Noreturn void MPU6050::imuTask()
{
    while (true)
    {
        const auto delay = static_cast<int>(1.0f / static_cast<float>(cfg.rate) * 1000);
        if (running)
        {
            getData();
            vTaskDelay(pdMS_TO_TICKS(delay));
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


esp_err_t MPU6050::start()
{
    ESP_LOGI(TAG.data(), "Starting MPU6050");

    ESP_LOGI(TAG.data(), "Initializing I2C...");
    esp_err_t ret = initI2C();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG.data(), "Failed to initialize!");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG.data(), "I2C initialized");

    ESP_LOGI(TAG.data(), "MPU6050 configuring");
    ret = configMPU6050();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG.data(), "Failed to configure MPU6050!");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG.data(), "MPU6050 configured");

    ESP_LOGI(TAG.data(), "Creating update task");
    const BaseType_t xReturned_2 = xTaskCreate(
        imuTaskWrapper,
        "imu_task",
        cfg.imu_task_stack_size,
        this,
        cfg.imu_task_priority,
        &imu_task_handle);

    if (xReturned_2 != pdPASS)
    {
        ESP_LOGE(TAG.data(), "Failed to create update task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG.data(), "MPU6050 update task started");

    running = true;
    ESP_LOGI(TAG.data(), "MPU6050 started");

    return ESP_OK;
}

esp_err_t MPU6050::stop()
{
    if (!running) return ESP_OK;

    running = false;

    if (imu_task_handle != nullptr)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
        vTaskDelete(imu_task_handle);
        imu_task_handle = nullptr;
    }

    esp_err_t ret = removeI2C();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG.data(), "Failed to remove I2C");
        return ESP_FAIL;
    }

    return ESP_OK;
}
