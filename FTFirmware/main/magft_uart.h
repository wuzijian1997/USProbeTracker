#ifndef MAGFT_UART_H
#define MAGFT_UART_H

#include <stdio.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"

#define TXD (UART_PIN_NO_CHANGE)//35
#define RXD (UART_PIN_NO_CHANGE)//34
#define RTS (UART_PIN_NO_CHANGE)
#define CTS (UART_PIN_NO_CHANGE)

#define UART_PORT_NUM       UART_NUM_0
#define BAUD                115200
#define BUF_SIZE (1024)

/*
* Example code for using uart:

init_uart();
 write_uart("\n\nHi thereeee, this is Eddy\n\n",(size_t)29);
*/

static void init_uart() {
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    const uart_config_t uart_config = {
        .baud_rate = BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
    // ESP_LOGI("UART", "Driver installed");
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    // ESP_LOGI("UART", "Paramaters configured");
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, TXD, RXD, RTS, CTS));
    // ESP_LOGI("UART", "Pins set");
}

static int read_uart(uint8_t* data, int length, int timeoutms) {
    return uart_read_bytes(UART_PORT_NUM, (void*)data, (uint32_t)length, (TickType_t)timeoutms / portTICK_PERIOD_MS);
}

static void write_uart(char* data, size_t len) {
    int ret = uart_write_bytes(UART_PORT_NUM, data, len);
    // printf("%d, %d, %d", *data, *(data+1), *(data+2));
    if (ret < 0) ESP_ERROR_CHECK(-1);
}

#endif //MAGFT_UART_H