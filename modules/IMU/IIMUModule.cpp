//
// Created by stikper on 17.03.25.
//

#include "IIMUModule.h"

#include <esp_log.h>
#include <stdexcept>

IIMUModule::IIMUModule()
{
    TAG = "IMU";

    lastAngVel.dataMutex = xSemaphoreCreateMutex();
    lastAccel.dataMutex = xSemaphoreCreateMutex();
    lastTemp.dataMutex = xSemaphoreCreateMutex();


    // TODO: Test throw error
    if (lastAngVel.dataMutex == nullptr || lastAccel.dataMutex == nullptr || lastTemp.dataMutex == nullptr)
        throw std::runtime_error("Failed to create IMU data mutex");
}

IIMUModule::~IIMUModule()
{
    if (lastAngVel.dataMutex != nullptr)
        vSemaphoreDelete(lastAngVel.dataMutex);
    if (lastAccel.dataMutex != nullptr)
        vSemaphoreDelete(lastAccel.dataMutex);
    if (lastTemp.dataMutex != nullptr)
        vSemaphoreDelete(lastTemp.dataMutex);
}

void IIMUModule::updateData(int64_t timestamp, const float* accel, const float* gyro, const float* temp)
{
    if (xSemaphoreTake(lastAccel.dataMutex, 100) == pdTRUE)
    {
        lastAccel.ax = accel[0];
        lastAccel.ay = accel[1];
        lastAccel.az = accel[2];
        lastAccel.timestamp = timestamp;
        xSemaphoreGive(lastAccel.dataMutex);
    }

    if (xSemaphoreTake(lastTemp.dataMutex, 100) == pdTRUE)
    {
        lastTemp.t = temp[0];
        lastTemp.timestamp = timestamp;
        xSemaphoreGive(lastTemp.dataMutex);
    }

    if (xSemaphoreTake(lastAngVel.dataMutex, 100) == pdTRUE)
    {
        lastAngVel.wx = gyro[0];
        lastAngVel.wy = gyro[1];
        lastAngVel.wz = gyro[2];
        lastAngVel.timestamp = timestamp;
        xSemaphoreGive(lastAngVel.dataMutex);
    }
}

IIMUModule::AngVel IIMUModule::getAngVel() const
{
    AngVel result = {};
    if (xSemaphoreTake(lastAngVel.dataMutex, 100) == pdTRUE)
    {
        result = lastAngVel;
        xSemaphoreGive(lastAngVel.dataMutex);
        result.dataMutex = nullptr;
        return result;
    }
    return result;
}

IIMUModule::Accel IIMUModule::getAccel() const
{
    Accel result = {};
    if (xSemaphoreTake(lastAccel.dataMutex, 100) == pdTRUE)
    {
        result = lastAccel;
        xSemaphoreGive(lastAccel.dataMutex);
        result.dataMutex = nullptr;
        return result;
    }
    return result;
}

IIMUModule::Temperature IIMUModule::getTemp() const
{
    Temperature result = {};
    if (xSemaphoreTake(lastTemp.dataMutex, 100) == pdTRUE)
    {
        result = lastTemp;
        xSemaphoreGive(lastTemp.dataMutex);
        result.dataMutex = nullptr;
        return result;
    }
    return result;
}

void IIMUModule::printLastData() const
{
    AngVel angVel = getAngVel();
    Accel accel = getAccel();
    Temperature temp = getTemp();
    // Convert timestamps to readable format (assuming timestamps are in microseconds)
    int64_t angVelTime = angVel.timestamp / 1000; // Convert to milliseconds
    int64_t accelTime = accel.timestamp / 1000;
    int64_t tempTime = temp.timestamp / 1000;

    ESP_LOGI(TAG.data(),
             "\n🔄 IMU Data Summary"
             "\n├─ 🌀 Angular Velocity (timestamp: %lld ms)"
             "\n│  ├─ 🔄 X-axis: %.3f °/s"
             "\n│  ├─ 🔄 Y-axis: %.3f °/s"
             "\n│  └─ 🔄 Z-axis: %.3f °/s"
             "\n├─ 🚀 Acceleration (timestamp: %lld ms)"
             "\n│  ├─ ➡️ X-axis: %.3f m/s²"
             "\n│  ├─ ↕️ Y-axis: %.3f m/s²"
             "\n│  └─ ↩️ Z-axis: %.3f m/s²"
             "\n└─ 🌡️ Temperature (timestamp: %lld ms)"
             "\n   └─ 🔥 Value:  %.2f °C",
             angVelTime,
             angVel.wx, angVel.wy, angVel.wz,
             accelTime,
             accel.ax, accel.ay, accel.az,
             tempTime,
             temp.t
    );
}

