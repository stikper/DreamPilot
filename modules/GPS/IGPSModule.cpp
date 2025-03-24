//
// Created by stikper on 04.03.25.
//

#include "IGPSModule.h"

#include <esp_log.h>
#include <stdexcept>


IGPSModule::IGPSModule()
{
    TAG = "GPS";

    lastPos.dataMutex = xSemaphoreCreateMutex();
    lastVel.dataMutex = xSemaphoreCreateMutex();
    lastAlt.dataMutex = xSemaphoreCreateMutex();
    lastTime.dataMutex = xSemaphoreCreateMutex();
    // TODO: Test throw error
    if (lastPos.dataMutex == nullptr || lastVel.dataMutex == nullptr || lastAlt.dataMutex == nullptr ||
        lastTime.dataMutex == nullptr)
        throw std::runtime_error("Failed to create GPS data mutex");
}

IGPSModule::~IGPSModule()
{
    if (lastPos.dataMutex != nullptr)
        vSemaphoreDelete(lastPos.dataMutex);
    if (lastVel.dataMutex != nullptr)
        vSemaphoreDelete(lastVel.dataMutex);
    if (lastAlt.dataMutex != nullptr)
        vSemaphoreDelete(lastAlt.dataMutex);
    if (lastTime.dataMutex != nullptr)
        vSemaphoreDelete(lastTime.dataMutex);
}

IGPSModule::Position IGPSModule::getPos() const
{
    Position result = {};
    result.valid = false;
    if (xSemaphoreTake(lastPos.dataMutex, 100) == pdTRUE)
    {
        result = lastPos;
        xSemaphoreGive(lastPos.dataMutex);
        result.dataMutex = nullptr;
        return result;
    }
    return result;
}

IGPSModule::Velocity IGPSModule::getVel() const
{
    Velocity result = {};
    result.valid = false;
    if (xSemaphoreTake(lastVel.dataMutex, 100) == pdTRUE)
    {
        result = lastVel;
        xSemaphoreGive(lastVel.dataMutex);
        result.dataMutex = nullptr;
        return result;
    }
    return result;
}

IGPSModule::Altitude IGPSModule::getAlt() const
{
    Altitude result = {};
    result.valid = false;
    if (xSemaphoreTake(lastAlt.dataMutex, 100) == pdTRUE)
    {
        result = lastAlt;
        xSemaphoreGive(lastAlt.dataMutex);
        result.dataMutex = nullptr;
        return result;
    }
    return result;
}

IGPSModule::TimeDate IGPSModule::getTime() const
{
    TimeDate result = {};
    result.valid = false;
    if (xSemaphoreTake(lastTime.dataMutex, 100) == pdTRUE)
    {
        result = lastTime;
        xSemaphoreGive(lastTime.dataMutex);
        result.dataMutex = nullptr;
        return result;
    }
    return result;
}

void IGPSModule::updateData(const GPSData& newData)
{
    if (newData.ignore) return;
    if (!newData.checksum)
    {
        ESP_LOGW(TAG.data(), "Received bad checksum message!");
        return;
    }
    if (newData.parse_error)
    {
        ESP_LOGW(TAG.data(), "Parsing error!");
        return;
    }
    //TODO!
    if (newData.type == "GGA")
    {
        if (newData.valid)
        {
            if (xSemaphoreTake(lastPos.dataMutex, 100) == pdTRUE)
            {
                lastPos.valid = true;
                lastPos.timestamp = newData.timestamp;
                lastPos.time = newData.time;
                lastPos.lon = newData.lon;
                lastPos.lat = newData.lat;
                xSemaphoreGive(lastPos.dataMutex);
            }
            if (xSemaphoreTake(lastAlt.dataMutex, 100) == pdTRUE)
            {
                lastAlt.valid = true;
                lastAlt.timestamp = newData.timestamp;
                lastAlt.time = newData.time;
                lastAlt.alt = newData.alt;
                xSemaphoreGive(lastAlt.dataMutex);
            }
        }
    }
    else if (newData.type == "RMC")
    {
        if (newData.valid)
        {
            if (xSemaphoreTake(lastVel.dataMutex, 100) == pdTRUE)
            {
                lastVel.valid = true;
                lastVel.timestamp = newData.timestamp;
                lastVel.time = newData.time;
                lastVel.spd = newData.spd;
                lastVel.hdg = newData.hdg;
                xSemaphoreGive(lastVel.dataMutex);
            }
            if (xSemaphoreTake(lastTime.dataMutex, 100) == pdTRUE)
            {
                lastTime.valid = true;
                lastTime.timestamp = newData.timestamp;
                lastTime.time = newData.time;
                lastTime.date = newData.date;
                xSemaphoreGive(lastTime.dataMutex);
            }
        }
    }
    else if (newData.type == "TXT")
    {
        ESP_LOGV(TAG.data(), "\n📝 TXT | ✓: %s\n└─ %s",
                 newData.checksum ? "✅" : "❌",
                 newData.text.c_str());
    }
    else
    {
    }
}

void IGPSModule::printLastData() const
{
    Position pos = getPos();
    Velocity vel = getVel();
    Altitude alt = getAlt();
    TimeDate time = getTime();

    // Parse date (format: DDMMYY)
    int day = static_cast<int>(time.date) / 10000;
    int month = (static_cast<int>(time.date) / 100) % 100;
    int year = static_cast<int>(time.date) % 100 + 2000;

    // Parse time (format: HHMMSS.sss)
    int hours = static_cast<int>(time.time) / 10000;
    int minutes = (static_cast<int>(time.time) / 100) % 100;
    int seconds = static_cast<int>(time.time) % 100;
    int milliseconds = static_cast<int>((time.time - static_cast<int>(time.time)) * 1000);

    ESP_LOGI(TAG.data(),
             "\n📍 GPS Data Summary"
             "\n├─ 🎯 Position (valid: %s)"
             "\n│  ├─ 🌍 Latitude:  %.7f°"
             "\n│  └─ 🌎 Longitude: %.7f°"
             "\n├─ 🚀 Movement (valid: %s)"
             "\n│  ├─ 💨 Speed:     %.1f m/s"
             "\n│  └─ 🧭 Heading:   %.1f°"
             "\n├─ ⬆️ Altitude (valid: %s)"
             "\n│  └─ 📏 Height:    %.1f m"
             "\n└─ 🕒 Timing (valid: %s)"
             "\n   ├─ 📅 Date:      %02d.%02d.%04d"
             "\n   └─ ⏰ Time:      %02d:%02d:%02d.%03d",
             pos.valid ? "✅" : "❌",
             pos.lat, pos.lon,
             vel.valid ? "✅" : "❌",
             vel.spd, vel.hdg,
             alt.valid ? "✅" : "❌",
             alt.alt,
             time.valid ? "✅" : "❌",
             day, month, year,
             hours, minutes, seconds, milliseconds
    );
}
