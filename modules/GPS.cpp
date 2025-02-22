//
// Created by stikper on 17.02.25.
//

#include "GPS.h"

#include <iostream>
#include <cstring>
#include <algorithm>
#include <regex>

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
    GGA_buffer = std::deque<gps_entry>();


    ESP_LOGI(TAG.data(), "Setting up UART...");
    if (uartInit() != ESP_OK)
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

    xTaskCreate(uartTaskWrapper, "uart_event_task", CONFIG_GPS_TASK_STACK_SIZE, this, CONFIG_GPS_TASK_PRIORITY,
                nullptr);

    return ESP_OK;
}

void GPS::uartTaskWrapper(void* param)
{
    GPS* gps = static_cast<GPS*>(param);
    gps->processUart();
}

// ReSharper disable once CppDFAUnreachableFunctionCall
void GPS::processUart()
{
    uart_event_t event;
    while (running)
    {
        //Waiting for UART event.
        if (xQueueReceive(uartQueue, &event, pdMS_TO_TICKS(200)))
        {
            ESP_LOGV(TAG.data(), "uart[%d] event:", UART_PORT);
            switch (event.type)
            {
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
    vTaskDelete(nullptr);
}

// ReSharper disable once CppDFAUnreachableFunctionCall
void GPS::processPattern()
{
    int pos = uart_pattern_pop_pos(UART_PORT);
    if (pos != -1)
    {
        if (pos + 1 >= CONFIG_GPS_UART_BUFFER_SIZE)
        {
            ESP_LOGW(TAG.data(), "GPS UART buffer too small");
            pos = CONFIG_GPS_UART_BUFFER_SIZE - 2;
        }
        /* read one line(include '\n') */
        const int read_len = uart_read_bytes(UART_PORT, uart_buffer, pos + 1, pdMS_TO_TICKS(200));
        /* make sure the line is a standard string */
        uart_buffer[read_len] = '\0';
        /* Send new line to handle */
        const auto nmea = std::string(uart_buffer);
        const std::regex newlines_re("\r*\n+");
        std::cout << std::endl;
        ESP_LOGD(TAG.data(), "Received line %s", std::regex_replace(nmea, newlines_re, "").data());
        parseNMEA(nmea);
    }
    else
    {
        ESP_LOGW(TAG.data(), "Pattern Queue Size too small");
        uart_flush_input(UART_PORT);
    }
}

// ReSharper disable once CppDFAUnreachableFunctionCall
bool GPS::checkIntegrity(const char* nmea)
{
    // Receiving checksum
    const char* checksumPos = strchr(nmea, '*');
    if (checksumPos == nullptr || *(checksumPos + 1) == '\0') return false;

    char* endPtr;
    unsigned long receivedChecksum = strtoul(checksumPos + 1, &endPtr, 16);
    if (endPtr == checksumPos + 1 || (endPtr - checksumPos) > 3) return false;

    // Calculating checksum
    unsigned int checksum = 0;
    for (int i = 1; nmea[i] != '*' && nmea[i] != '\0'; ++i)
    {
        checksum ^= nmea[i]; // XOR всех символов
    }

    // Comparing checksum
    return checksum == static_cast<unsigned char>(receivedChecksum);
}

// ReSharper disable once CppDFAUnreachableFunctionCall
void GPS::parseNMEA(const std::string& nmea)
{
    if (nmea.find('$') == -1 || nmea.find('*') == -1)
    {
        ESP_LOGE(TAG.data(), "Not nmea string received");
        return;
    }
#ifdef CONFIG_GPS_CHECKSUM
    if (checkIntegrity(nmea.data()))
    {
        ESP_LOGD(TAG.data(), "Integrity check passed!");
    }
    else
    {
        ESP_LOGE(TAG.data(), "Integrity check failed!");
        return;
    }
#endif

    const std::string format = nmea.substr(nmea.find('$') + 3, 3);
    ESP_LOGI(TAG.data(), "%s", format.data());

    const std::string nmea_trimmed = nmea.substr(nmea.find('$'), nmea.find('*') - nmea.find('$'));

    if (format == "GSV")
    {
        parseGSV(this, nmea_trimmed);
    }
    else if (format == "GSA")
    {
        parseGSA(this, nmea_trimmed);
    }
    else if (format == "GLL")
    {
        parseGLL(this, nmea_trimmed);
    }
    else if (format == "GGA")
    {
        parseGGA(this, nmea_trimmed);
    }
    else if (format == "RMC")
    {
        parseRMC(this, nmea_trimmed);
    }
    else if (format == "VTG")
    {
        parseVTG(this, nmea_trimmed);
    }
    else if (format == "TXT")
    {
        parseTXT(this, nmea_trimmed);
    }
    else
    {
        ESP_LOGI(TAG.data(), "Received unexpected NMEA message: %s", (nmea_trimmed + "*").data());
    }
}


// ReSharper disable once CppDFAUnreachableFunctionCall
void GPS::parseGSV(GPS* gps, const std::string& nmea)
{
}

// ReSharper disable once CppDFAUnreachableFunctionCall
void GPS::parseGSA(GPS* gps, const std::string& nmea)
{
}

// ReSharper disable once CppDFAUnreachableFunctionCall
void GPS::parseGLL(GPS* gps, const std::string& nmea)
{
}

// ReSharper disable once CppDFAUnreachableFunctionCall
void GPS::parseGGA(GPS* gps, const std::string& nmea)
{
}

// ReSharper disable once CppDFAUnreachableFunctionCall
void GPS::parseRMC(GPS* gps, const std::string& nmea)
{
}

// ReSharper disable once CppDFAUnreachableFunctionCall
void GPS::parseVTG(GPS* gps, const std::string& nmea)
{
}

// ReSharper disable once CppDFAUnreachableFunctionCall
// NOT COMPLETED
void GPS::parseTXT(GPS* gps, const std::string& nmea)
{
    const auto tokens = split(nmea, ',');
    if (tokens.size() != 5)
    {
        ESP_LOGW(gps->TAG.data(), "Error parsing TXT");
        return;
    }
    ESP_LOGI(gps->TAG.data(), "Received txt message: %s", tokens[4].data());
}

// ReSharper disable once CppDFAUnreachableFunctionCall
std::vector<std::string> GPS::split(const std::string& s, const char delimiter)
{
    std::vector<std::string> tokens;
    std::stringstream ss(s);
    std::string token;
    while (std::getline(ss, token, delimiter))
    {
        tokens.push_back(token);
    }
    return tokens;
}
