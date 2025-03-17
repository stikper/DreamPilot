//
// Created by stikper on 04.03.25.
//

#ifndef NEO6M_H
#define NEO6M_H

#include <string>
#include <hal/uart_types.h>

#include "IGPSModule.h"
#include "NMEAParser.h"


class NEO6M final : public IGPSModule
{
public:
    struct neo6m_config_t
    {
        int uart_buffer_size;
        uart_port_t uart_port_num;
        int uart_baud_rate;
        int uart_queue_size;
        int uart_txd;
        int uart_rxd;
        int uart_task_stack_size;
        int uart_task_priority;
        int nmea_task_stack_size;
        int nmea_task_priority;
    };

private:
    neo6m_config_t cfg;
    std::string TAG;

    char* uart_buffer;
    size_t uart_buffer_len;

    QueueHandle_t uartQueue;
    QueueHandle_t nmeaQueue;

    TaskHandle_t uart_task_handle;
    TaskHandle_t nmea_task_handle;
    bool running;

public:
    NEO6M();
    ~NEO6M() override;

private:
    esp_err_t initUART();
    esp_err_t removeUART() const;
    static void uartTaskWrapper(void* param);
    _Noreturn void processUART();
    void processPattern();

    static void nmeaTaskWrapper(void* param);
    _Noreturn void processNMEA();

    esp_err_t start() override;
    esp_err_t stop() override;
};


#endif //NEO6M_H
