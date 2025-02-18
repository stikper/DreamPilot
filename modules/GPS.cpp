//
// Created by stikper on 17.02.25.
//

#include "GPS.h"

#include <cstring>

#include "esp_log.h"
#include "driver/uart.h"




GPS::GPS()
{
    TAG = "GPS";
    ESP_LOGI(TAG.data(), "Initializing...");
    uart_buffer = new char[CONFIG_GPS_UART_BUFFER_SIZE];
    memset(uart_buffer, '\0', CONFIG_GPS_UART_BUFFER_SIZE);
    uart_buffer_len = 0;
    uartQueue = nullptr;
    UART_PORT = static_cast<uart_port_t>(CONFIG_GPS_UART_PORT_NUM);
    running = true;
    buffer = std::vector<gps_entry>(CONFIG_GPS_BUFFER_SIZE);


    ESP_LOGI(TAG.data(), "Setting up UART...");
    if(uartInit() != ESP_OK)
        ESP_LOGE(TAG.data(), "UART initializing failed!");
    else
        ESP_LOGI(TAG.data(), "UART initialized successfully!");
}

GPS::~GPS() = default;

esp_err_t GPS::uartInit()
{
    uart_config_t uart_config = {
        .baud_rate = CONFIG_GPS_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };

    if (uart_driver_install(UART_PORT, CONFIG_GPS_UART_BUFFER_SIZE * 2,
        0, CONFIG_GPS_QUEUE_SIZE, &uartQueue, 0) != ESP_OK)
    {
        ESP_LOGE(TAG.data(), "install uart driver failed!");
        return ESP_FAIL;
    }

    if (uart_param_config(UART_PORT, &uart_config) != ESP_OK)
    {
        ESP_LOGE(TAG.data(), "config uart parameter failed!");
        return ESP_FAIL;
    }

    if (uart_set_pin(UART_PORT, CONFIG_GPS_UART_TXD,
        CONFIG_GPS_UART_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK)
    {
        ESP_LOGE(TAG.data(), "config uart gpio failed!");
        return ESP_FAIL;
    }

    uart_enable_pattern_det_baud_intr(UART_PORT, '\n', 1, 9, 0, 0);

    uart_pattern_queue_reset(UART_PORT, CONFIG_GPS_QUEUE_SIZE);
    uart_flush(UART_PORT);

    xTaskCreate(uartTaskWrapper, "uart_event_task", CONFIG_GPS_TASK_STACK_SIZE, this, CONFIG_GPS_TASK_PRIORITY, nullptr);

    return ESP_OK;
}

void GPS::uartTaskWrapper(void* param)
{
    GPS* gps = static_cast<GPS*>(param);
    gps->processUart();
}

void GPS::processUart()
{
    uart_event_t event;
    while (running) {
        //Waiting for UART event.
        if (xQueueReceive(uartQueue, &event, pdMS_TO_TICKS(200))) {
            ESP_LOGV(TAG.data(), "uart[%d] event:", UART_PORT);
            switch (event.type) {
            case UART_DATA:
                break;
            case UART_FIFO_OVF:
                ESP_LOGW(TAG.data(), "HW FIFO Overflow");
                uart_flush(UART_PORT);
                xQueueReset(uartQueue);
                break;
            case UART_BUFFER_FULL:
                ESP_LOGW(TAG.data(), "Ring Buffer Full");
                uart_flush(UART_PORT);
                xQueueReset(uartQueue);
                break;
            case UART_BREAK:
                ESP_LOGW(TAG.data(), "Rx Break");
                break;
            case UART_PARITY_ERR:
                ESP_LOGW(TAG.data(), "Parity Error");
                break;
            case UART_FRAME_ERR:
                ESP_LOGW(TAG.data(), "Frame Error");
                break;
            case UART_PATTERN_DET:
                processPattern();
                break;
            default:
                ESP_LOGW(TAG.data(), "unknown uart event type: %d", event.type);
                break;

            }
        }
    }
}

void GPS::processPattern()
{
    int pos = uart_pattern_pop_pos(UART_PORT);
    if (pos != -1) {
        /* read one line(include '\n') */
        if (pos + 1 > CONFIG_GPS_UART_BUFFER_SIZE) pos = CONFIG_GPS_UART_BUFFER_SIZE - 1;
        const int read_len = uart_read_bytes(UART_PORT, uart_buffer, pos + 1, 100 / portTICK_PERIOD_MS);
        /* make sure the line is a standard string */
        uart_buffer[read_len] = '\0';
        /* Send new line to handle */
        ESP_LOGI(TAG.data(), "Received line %s", uart_buffer);
    } else {
        ESP_LOGW(TAG.data(), "Pattern Queue Size too small");
        uart_flush_input(UART_PORT);
    }
}




