//
// Created by stikper on 17.02.25.
//

#ifndef GPS_H
#define GPS_H


#include <esp_err.h>
#include <string>
#include <vector>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <freertos/queue.h>
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
    std::vector<gps_entry> buffer;

public:
    GPS();
    ~GPS();

private:
    esp_err_t uartInit();
    static void uartTaskWrapper(void* param);
    void processUart();
    void processPattern();


};


#endif //GPS_H
