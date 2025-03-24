//
// Created by stikper on 17.03.25.
//

#ifndef IIMUMODULE_H
#define IIMUMODULE_H

#include <cstdint>
#include <esp_err.h>
#include <string>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>


class IIMUModule
{
public:
    struct AngVel
    {
        SemaphoreHandle_t dataMutex = nullptr;
        int64_t timestamp = -1;
        float wx = 0;
        float wy = 0;
        float wz = 0;
    };
    struct Accel
    {
        SemaphoreHandle_t dataMutex = nullptr;
        int64_t timestamp = -1;
        float ax = 0;
        float ay = 0;
        float az = 0;
    };
    struct Temperature
    {
        SemaphoreHandle_t dataMutex = nullptr;
        int64_t timestamp = -1;
        float t = 0;
    };

private:
    std::string TAG;

    AngVel lastAngVel;
    Accel lastAccel;
    Temperature lastTemp;

protected:
    IIMUModule();

    void updateData(int64_t timestamp, const float* accel, const float* gyro, const float* temp);

public:
    virtual ~IIMUModule();

    AngVel getAngVel() const;
    Accel getAccel() const;
    Temperature getTemp() const;

    void printLastData() const;

    virtual esp_err_t start() = 0;
    virtual esp_err_t stop() = 0;
};


#endif //IIMUMODULE_H
