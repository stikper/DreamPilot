//
// Created by stikper on 04.03.25.
//

#ifndef IGPSMODULE_H
#define IGPSMODULE_H

#include <cstdint>
#include <string>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>


class IGPSModule
{
public:
    struct GPSData
    {
        int64_t timestamp = 0;
        bool checksum = false;
        bool ignore = false;
        bool parse_error = false;
        std::string source = "";
        std::string type = "";
        bool valid = false;
        float time = 0;
        float date = 0;
        double lat = 0;
        double lon = 0;
        float alt = 0;
        float spd = 0;
        float hdg = 0;
        std::string text = "";
        int satellites = 0;
        float hdop;
    };

    struct Position
    {
        SemaphoreHandle_t dataMutex = nullptr;
        int64_t timestamp = -1;
        bool valid = false;
        float time = 0;
        double lat = 0;
        double lon = 0;
    };

    struct Velocity
    {
        SemaphoreHandle_t dataMutex = nullptr;
        int64_t timestamp = -1;
        bool valid = false;
        float time = 0;
        float spd = 0;
        float hdg = 0;
    };

    struct Altitude
    {
        SemaphoreHandle_t dataMutex = nullptr;
        int64_t timestamp = -1;
        bool valid = false;
        float time = 0;
        float alt = 0;
    };

    struct TimeDate
    {
        SemaphoreHandle_t dataMutex = nullptr;
        int64_t timestamp = -1;
        bool valid = false;
        float time = 0;
        float date = 0;
    };

private:
    std::string TAG;

    Position lastPos;
    Velocity lastVel;
    Altitude lastAlt;
    TimeDate lastTime;

protected:
    IGPSModule();

    void updateData(const GPSData& newData);

public:
    virtual ~IGPSModule();

    Position getPos() const;
    Velocity getVel() const;
    Altitude getAlt() const;
    TimeDate getTime() const;

    //TODO its for debug
    void printLastData() const;

    virtual esp_err_t start() = 0;
    virtual esp_err_t stop() = 0;
};


#endif //IGPSMODULE_H
