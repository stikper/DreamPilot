//
// Created by stikper on 17.02.25.
//

#ifndef GPS_H
#define GPS_H


#include <esp_err.h>
#include <string>
#include <vector>
#include <deque>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <hal/uart_types.h>

#include "sdkconfig.h"


class GPS
{
    std::string TAG;
    char *uart_buffer;
    size_t uart_buffer_len;
    QueueHandle_t uartQueue;
    uart_port_t UART_PORT;
    bool running;
    struct gps_entry {
        float time;
        float lon;
        float lat;
        float alt;
        float spd;
        float hdg;
        float cnt;
    };
    std::deque<gps_entry> GGA_buffer;

public:
    GPS();
    ~GPS();

private:
    esp_err_t uartInit();
    static void uartTaskWrapper(void* param);
    void processUart();
    void processPattern();
    void parseNMEA(const std::string& nmea);
    static bool checkIntegrity(const char* nmea);
    static void parseGSV(GPS* gps, const std::string& nmea);
    static void parseGSA(GPS* gps, const std::string& nmea);
    static void parseGLL(GPS* gps, const std::string& nmea);
    static void parseGGA(GPS* gps, const std::string& nmea);
    static void parseRMC(GPS* gps, const std::string& nmea);
    static void parseVTG(GPS* gps, const std::string& nmea);
    static void parseTXT(GPS* gps, const std::string& nmea);
    static std::vector<std::string> split(const std::string &s, char delimiter);


};


#endif //GPS_H
