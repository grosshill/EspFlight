#pragma once
#include "driver/uart.h"
#include "esp_err.h"

#define SBUS_UART_NUM UART_NUM_0
#define SBUS_HEADER 0x0f
#define SBUS_END 0x00
#define SBUS_INTRISIC_BAUD 100000
#define SBUS_FRAME_LEN 25

esp_err_t sbus_init(uart_config_t* config)
{
    return ESP_OK;
}