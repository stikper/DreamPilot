//
// Created by stikper on 04.03.25.
//

#include "NEO6M.h"

#include <cstring>
#include <esp_log.h>
#include <stdexcept>
#include <driver/uart.h>

NEO6M::NEO6M(): cfg{}
{
    TAG = "NEO-6M";
    ESP_LOGI(TAG.data(), "Initializing...");

    // TODO: Remove hardcode
    // TODO: Add verbose logging
    // TODO: Add malloc checks
    // TODO: config sequence
    // Setting configuration
    cfg.uart_buffer_size = 1024;
    cfg.uart_port_num = UART_NUM_2;
    cfg.uart_baud_rate = 9600;
    cfg.uart_queue_size = 16;
    cfg.uart_rxd = 16;
    cfg.uart_txd = 17;
    cfg.uart_task_stack_size = 4096;
    cfg.uart_task_priority = 12;
    cfg.nmea_task_stack_size = 4096;
    cfg.nmea_task_priority = 13;

    // Create uart buffer
    uart_buffer = new char[cfg.uart_buffer_size];
    memset(uart_buffer, '\0', cfg.uart_buffer_size);

    // Initializing variables
    uart_buffer_len = 0;

    uartQueue = nullptr;
    nmeaQueue = nullptr;
    uart_task_handle = nullptr;
    nmea_task_handle = nullptr;
    running = false;

    ESP_LOGI(TAG.data(), "Module is ready to start!");
}

NEO6M::~NEO6M()
{
    // Stop nmea, uart tasks and remove queues
    stop();

    // Delete uart buffer
    delete[] uart_buffer;
    uart_buffer = nullptr;
}


esp_err_t NEO6M::initUART()
{
    // Config and install uart driver
    const uart_config_t uart_config = {
        .baud_rate = cfg.uart_baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };

    esp_err_t ret = uart_driver_install(cfg.uart_port_num, cfg.uart_buffer_size * 2, 0, cfg.uart_queue_size, &uartQueue,
                                        0);
    if (ret != ESP_OK) return ret;

    ret = uart_param_config(cfg.uart_port_num, &uart_config);
    if (ret != ESP_OK) return ret;

    ret = uart_set_pin(cfg.uart_port_num, cfg.uart_txd, cfg.uart_rxd, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) return ret;

    // Set uart pattern
    ret = uart_enable_pattern_det_baud_intr(cfg.uart_port_num, '\n', 1, 9, 0, 0);
    if (ret != ESP_OK) return ret;

    ret = uart_pattern_queue_reset(cfg.uart_port_num, cfg.uart_queue_size);
    if (ret != ESP_OK) return ret;

    ret = uart_flush(cfg.uart_port_num);
    return ret;
}

esp_err_t NEO6M::removeUART() const
{
    // Delete uart driver
    esp_err_t ret = uart_driver_delete(cfg.uart_port_num);
    return ret;
}

void NEO6M::uartTaskWrapper(void* param)
{
    auto* gps = static_cast<NEO6M*>(param);

    // Start processUART function in task
    gps->processUART();

    // Remove current task
    vTaskDelete(nullptr);
}

// ReSharper disable CppDFAUnreachableFunctionCall
_Noreturn void NEO6M::processUART()
{
    // Process UART events
    uart_event_t event;

    while (true)
    {
        // Waiting for UART event.
        if (running && xQueueReceive(uartQueue, &event, pdMS_TO_TICKS(200)) == pdTRUE)
        {
            switch (event.type)
            {
            case UART_DATA:
                break;
            case UART_FIFO_OVF:
                ESP_LOGW(TAG.data(), "HW FIFO Overflow");
                uart_flush(cfg.uart_port_num);
                xQueueReset(uartQueue);
                break;
            case UART_BUFFER_FULL:
                ESP_LOGW(TAG.data(), "Ring Buffer Full");
                uart_flush(cfg.uart_port_num);
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
                ESP_LOGW(TAG.data(), "Unknown uart event type: %d", event.type);
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelete(nullptr);
}

void NEO6M::processPattern()
{
    int pos = uart_pattern_pop_pos(cfg.uart_port_num);

    if (pos != -1)
    {
        if (pos + 1 >= cfg.uart_buffer_size)
        {
            ESP_LOGW(TAG.data(), "GPS UART buffer too small");
            pos = cfg.uart_buffer_size - 2;
        }

        const int read_len = uart_read_bytes(cfg.uart_port_num, uart_buffer, pos + 1, pdMS_TO_TICKS(200));
        uart_buffer[read_len] = '\0';
        const auto* nmea_ptr = new std::string(uart_buffer);

        // TODO: is it works?
        xQueueSend(nmeaQueue, &nmea_ptr, 0);
    }
    else
    {
        ESP_LOGW(TAG.data(), "Pattern Queue Size too small");
        uart_flush_input(cfg.uart_port_num);
    }
}

void NEO6M::nmeaTaskWrapper(void* param)
{
    auto* gps = static_cast<NEO6M*>(param);

    gps->processNMEA();

    vTaskDelete(nullptr);
}

_Noreturn void NEO6M::processNMEA()
{
    std::string* nmea_ptr;

    while (true)
    {
        //Waiting for UART event.
        if (running && xQueueReceive(nmeaQueue, &nmea_ptr, pdMS_TO_TICKS(200)) == pdTRUE)
        {
            std::string nmea(*nmea_ptr);
            delete nmea_ptr;
            using GPSData = IGPSModule::GPSData;
            //TODO Add try catch
            GPSData newData = NMEAParser::parse(nmea);
            updateData(newData);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelete(nullptr);
}

esp_err_t NEO6M::start()
{
    if (running) return ESP_OK;

    ESP_LOGI(TAG.data(), "Starting...");

    esp_err_t ret;

    ESP_LOGI(TAG.data(), "Initializing NMEA Queue...");
    nmeaQueue = xQueueCreate(10, sizeof(void*));
    const BaseType_t xReturned_1 = xTaskCreate(
        nmeaTaskWrapper,
        "nmea_parsing_task",
        cfg.nmea_task_stack_size,
        this,
        cfg.nmea_task_priority,
        &nmea_task_handle);

    if (xReturned_1 != pdPASS)
    {
        ESP_LOGE(TAG.data(), "failed to create UART task");
        ret = removeUART();
        if (ret != ESP_OK)
            ESP_LOGE(TAG.data(), "Failed to remove UART: %d", ret);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG.data(), "NMEA Queue initialized!");


    ESP_LOGI(TAG.data(), "Initializing UART...");
    ret = initUART();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG.data(), "Failed to initialize UART: %d", ret);
        return ESP_FAIL;
    }

    const BaseType_t xReturned_2 = xTaskCreate(
        uartTaskWrapper,
        "uart_event_task",
        cfg.uart_task_stack_size,
        this,
        cfg.uart_task_priority,
        &uart_task_handle);

    if (xReturned_2 != pdPASS)
    {
        ESP_LOGE(TAG.data(), "failed to create UART task");
        ret = removeUART();
        if (ret != ESP_OK)
            ESP_LOGE(TAG.data(), "Failed to remove UART: %d", ret);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG.data(), "UART Initialized!");

    running = true;
    ESP_LOGI(TAG.data(), "NEO6M GPS module started");

    return ESP_OK;
}

esp_err_t NEO6M::stop()
{
    if (!running) return ESP_OK;

    running = false;

    if (nmea_task_handle != nullptr)
    {
        // TODO!!! test deleted task delete
        vTaskDelay(pdMS_TO_TICKS(100));
        vTaskDelete(nmea_task_handle);
        nmea_task_handle = nullptr;
    }

    vQueueDelete(nmeaQueue);

    if (uart_task_handle != nullptr)
    {
        // TODO: test deleted task delete
        vTaskDelay(pdMS_TO_TICKS(100));
        vTaskDelete(uart_task_handle);
        uart_task_handle = nullptr;
    }

    esp_err_t ret = removeUART();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG.data(), "Failed to remove UART: %d", ret);
        return ESP_FAIL;
    }

    return ESP_OK;
}
