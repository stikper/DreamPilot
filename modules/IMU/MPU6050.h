//
// Created by stikper on 24.03.25.
//

#ifndef MPU6050_H
#define MPU6050_H

#include <driver/i2c_types.h>
#include <soc/gpio_num.h>

#include "IMU/IIMUModule.h"



class MPU6050 final : public IIMUModule
{
public:
    struct mpu6050_config_t
    {
        int i2c_freq;
        i2c_port_t i2c_port_num;
        gpio_num_t i2c_sda;
        gpio_num_t i2c_scl;
        int rate;
        int imu_task_priority;
        int imu_task_stack_size;
        uint8_t accel_scale;
        uint8_t gyro_scale;
    };

private:
    mpu6050_config_t cfg;
    std::string TAG;

    TaskHandle_t imu_task_handle;

    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;

    bool running;

public:
    MPU6050();
    ~MPU6050() override;

private:
    esp_err_t initI2C();
    esp_err_t removeI2C();
    esp_err_t configMPU6050() const;

    static void imuTaskWrapper(void* param);
    _Noreturn void imuTask();

    esp_err_t getData();

    esp_err_t start() override;
    esp_err_t stop() override;
};



#endif //MPU6050_H
