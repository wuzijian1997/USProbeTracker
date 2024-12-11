#include "driver/spi_master.h"
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#include "magft_pindef.h"

spi_device_handle_t vspi_handle[N_VSPI_SENSORS];
spi_device_handle_t hspi_handle[N_HSPI_SENSORS];

static void init_spi(spi_host_device_t RCV_HOST)
{
    int *SPI_PINS;
    int n_sensors;
    spi_device_handle_t *handle;

    if (RCV_HOST == VSPI_HOST) {
        SPI_PINS = VSPI_PINS;
        n_sensors = N_VSPI_SENSORS;
        handle = vspi_handle;
    }
    else {
        SPI_PINS = HSPI_PINS;
        n_sensors = N_HSPI_SENSORS;
        handle = hspi_handle;
    }

    // Configuration for the SPI bus
    spi_bus_config_t buscfg = {
        .mosi_io_num = SPI_PINS[2],
        .miso_io_num = SPI_PINS[1],
        .sclk_io_num = SPI_PINS[0],
        .quadwp_io_num = -1,
        .quadhd_io_num = -1};

    // Initialize the SPI bus and add the device we want to send stuff to.
    ESP_ERROR_CHECK(spi_bus_initialize(RCV_HOST, &buscfg, SPI_DMA_CH_AUTO));

    // Configuration for the SPI devices on the other side of the bus
    for (int i = 0; i < n_sensors; i++)
    {
        spi_device_interface_config_t devcfg = {
            .command_bits = 0,
            .address_bits = 0,
            .dummy_bits = 0,
            .clock_speed_hz = 5000000, //5MHz
            .duty_cycle_pos = 128, // 50% duty cycle
            .mode = 0,
            .spics_io_num = SPI_PINS[i+3],
            .cs_ena_posttrans = 3, // Keep the CS low 3 cycles after transaction, to stop slave from missing the last bit when CS has less propagation delay than CLK
            .queue_size = 1,
        };

        ESP_ERROR_CHECK(spi_bus_add_device(RCV_HOST, &devcfg, &handle[i]));
    }
    ESP_LOGI(TAG, "SPI init. successful");
}

// We want to extract our 12-bit number from recv and 
// recv is a 4-array of uint8_t values
// recv[0] will be 0
static uint16_t recv2int(uint8_t *recv)
{
    uint16_t val = ((recv[1] & 0x1F) << 8) | recv[2];
    return val;
}

static uint16_t int22scomp(uint16_t val)
{
    uint16_t ret;
    if (val < 0)
    {
        ret = (uint16_t)(-val);
        ret = (~(ret - 1)) | 0x8000;
    }
    else
        ret = (uint16_t)val;

    return ret;
}

// Reads data from SPI using polling transactions
// Replaces function using interrupt transactions because polling transactions are faster
// However with polling transactions CPU is busy during polling transactions
static void spi_read_sample_polling(uint16_t *dataOut, spi_host_device_t RCV_HOST)
{
    int n_sensors;
    spi_device_handle_t *handle;

    if (RCV_HOST == VSPI_HOST) { 
        n_sensors = N_VSPI_SENSORS;
        handle = vspi_handle; 
    }
    else { 
        n_sensors = N_HSPI_SENSORS;
        handle = hspi_handle; 
    }

    // Send: 0000 (1 = start bit) (1 = single ended) (0 = don't care) (00 = CH0) (15x0 = don't care)
    //                            (0 = differential)                  (10 = CH2)
    // Receive: (8x? = don't care) (2x? = don't care) 0 (B12-B8) (B7-B0)
    //          |---------- Ignore these -----------|    MSB   ->   LSB
    uint8_t setupCH0[3] = {0b00001100, 0b00000000, 0b00000000}; // Channel 0
    uint8_t setupCH2[3] = {0b00001101, 0b00000000, 0b00000000}; // Channel 2

    for (int i = 0; i < n_sensors; i++)
    {
        spi_transaction_t t;
        t.length = 24;
        t.rxlength = 24;
        t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
        t.tx_data[0] = setupCH0[0];
        t.tx_data[1] = 0x0;
        t.tx_data[2] = 0x0;

        // Read first Hall effect sensor
        spi_device_polling_transmit(handle[i], &t);
        // ESP_LOGI(TAG, "Received S1: %d\t%d\t%d\t%d\n", t.rx_data[0],t.rx_data[1],t.rx_data[2],t.rx_data[3]);
        int16_t CH0 = recv2int(t.rx_data);
        //printf("CH0: %d\n", CH0);

        // Read second Hall effect sensor
        t.tx_data[0] = setupCH2[0];
        spi_device_polling_transmit(handle[i], &t);
        // ESP_LOGI(TAG, "Received S2: %d\t%d\t%d\t%d\n", t.rx_data[0],t.rx_data[1],t.rx_data[2],t.rx_data[3]);
        int16_t CH2 = recv2int(t.rx_data);
        //printf("CH2: %d\n", CH2);

        // Calculate differential output
        //dataOut[i] = int22scomp(CH0 - CH2);
        dataOut[2*i] = CH0;
        dataOut[2*i+1] = CH2;
    }
}

// Reads data from SPI using polling transactions
// Replaced by spi_read_sample_polling()
static void spi_read_sample_interrupt(uint16_t *dataOut, spi_host_device_t RCV_HOST)
{
    int n_sensors;
    spi_device_handle_t *handle;

    if (RCV_HOST == VSPI_HOST) { 
        n_sensors = N_VSPI_SENSORS;
        handle = vspi_handle; 
    }
    else { 
        n_sensors = N_HSPI_SENSORS;
        handle = hspi_handle; 
    }

    // Send: 0000 (1 = start bit) (1 = single ended) (0 = don't care) (00 = CH0) (15x0 = don't care)
    //                            (0 = differential)                  (10 = CH2)
    // Receive: (8x? = don't care) (2x? = don't care) 0 (B12-B8) (B7-B0)
    //          |---------- Ignore these -----------|    MSB   ->   LSB
    uint8_t setupCH0[3] = {0b00001100, 0b00000000, 0b00000000}; // Channel 0
    uint8_t setupCH2[3] = {0b00001101, 0b00000000, 0b00000000}; // Channel 2

    for (int i = 0; i < n_sensors; i++)
    {
        spi_transaction_t t;
        t.length = 24;
        t.rxlength = 24;
        t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
        t.tx_data[0] = setupCH0[0];
        t.tx_data[1] = 0x0;
        t.tx_data[2] = 0x0;

        // Read first Hall effect sensor
        spi_device_transmit(handle[i], &t);
        //ESP_LOGI(TAG, "Received S1: %d\t%d\t%d\t%d\n", t.rx_data[0],t.rx_data[1],t.rx_data[2],t.rx_data[3]);
        uint16_t CH0 = recv2int(t.rx_data);
        printf("%d\n", CH0);

        // Read second Hall effect sensor
        t.tx_data[0] = setupCH2[0];
        spi_device_transmit(handle[i], &t);
        //ESP_LOGI(TAG, "Received S2: %d\t%d\t%d\t%d\n", t.rx_data[0],t.rx_data[1],t.rx_data[2],t.rx_data[3]);
        uint16_t CH2 = recv2int(t.rx_data);
        printf("%d\n", CH2);

        // Calculate differential output
        dataOut[i] = int22scomp(CH0 - CH2);
    }
}

static esp_err_t close_spi(spi_host_device_t RCV_HOST)
{
    if (RCV_HOST == VSPI_HOST) { 
        return spi_bus_remove_device(vspi_handle); 
    }
    else {
        return spi_bus_remove_device(hspi_handle); 
    }
    
}