#ifndef MAGFT_IMU_H
#define MAGFT_IMU_H

#include <stdio.h>
#include <math.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "magft_pindef.h"
#include "magft_sync.h"
#include "esp_timer.h"

// Important that IMU_ACC_RATE and IMU_GYR_RATE are the same when using FIFO
// IMU_RATE is defined in magft_pindef.h
#define IMU_ACC_SCALE 4
#define IMU_ACC_RATE IMU_RATE
#define IMU_GYR_SCALE 1000
#define IMU_GYR_RATE IMU_RATE
#define TEMP_SENS 256
#define TEMP_BIAS 25

#define TEMP_RATE_FIFO 0b00     /*!<Temp. rate batched in FIFO: 1.6Hz: 0b01, 12.5Hz: 0b10, 52Hz: 0b11, not batched in FIFO: 0b00 */
#define TS_DEC_FIFO 0b00

// Temperature refreshes at a fixed 52 Hz, so no constant for that
// We are not batching temperature in FIFO

// Convert readings with 2s complement for printing, or keep as ints for wifi communication
#define READ_RAW 1


/*
* Example code for running I2C and reading from IMU

init_imu();
ESP_LOGI(TAG,"I2C initialized successfully");

// Read WHO_AM_I register
uint8_t data[1];
ESP_ERROR_CHECK(imu_register_read(IMU_WHO_AM_I_REG_ADDR, data, 1));
ESP_LOGI(TAG, "WHO_AM_I = %X", data[0]);

// Read a sample from the IMU
print_imu_reading();

// Shut down
close_imu();
*/

/**
 * @brief Read a sequence of bytes from IMU registers
 */
static esp_err_t imu_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, IMU_ADDRESS, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}


/**
 * @brief Write a byte to an IMU register
 */
static esp_err_t imu_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, IMU_ADDRESS, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    return ret;
}

static esp_err_t is_data_available(uint8_t* d)
{
    return imu_register_read(STATUS_REG, d, 1);
}

// Read most significant and least significant 8 bit registers and combine
static uint16_t readHL(int regNumL) {
    uint8_t tempL; uint8_t tempH;
    
    ESP_ERROR_CHECK(imu_register_read(regNumL, &tempL, 1));
    ESP_ERROR_CHECK(imu_register_read(regNumL+1, &tempH, 1));

    return tempL + (tempH << 8);
}

// Convert from 2s complement to signed int
static int convert_2s_comp(uint16_t comp2) 
{
    // Check the sign bit (2^15 = 0x8000)
    if (comp2 & 0x8000) {
        return -(int)(((~comp2) + 1)&0xffff); // invert bits and add 1
    } else return (int)comp2;
}

// Read x,y,z registers for acceleration and gyroscope
static void readAG(int firstReg, int* vecOut) {
    for (int i = 0; i < 3; i++) {
        uint16_t data = readHL(firstReg+2*i);
        
        // Convert from 2's complement
        int val = READ_RAW ? data : convert_2s_comp(data);

        // Add to output vector
        vecOut[i] = val;
    }
}

static void update_fifo_mode(int fifoMode) 
{
    // Write the updated byte with updated fifoMode
    uint8_t newByte; 
    newByte = (TS_DEC_FIFO << 6) + (TEMP_RATE_FIFO << 4) + fifoMode;

    ESP_ERROR_CHECK(imu_register_write_byte(FIFO_CTRL4, newByte));
}

static void fifo_word_to_data(uint8_t* fifoWord, int* data) 
{
    for (int i = 0; i < 3; i++) {
        int rawData = (fifoWord[2*i + 2] << 8) + (fifoWord[2*i + 1]);
        data[i] = READ_RAW ? rawData : convert_2s_comp(rawData);
    }
}

/**
 * @brief try to read one sample of temperature, acceleration, and gyroscope
 * @param tempData pointer to int in which to store temperature reading 
 * @param gyroData pointer to 3-element array in which to store gyroscope reading 
 * @param accData pointer to 3-element array in which to store acceleration reading 
 * @param hasData pointer to 8 bits whose least significant 3 bits show if data was available in [temp, gyro, acc]
 */
// TODO: when watermark threshold is reached, the pattern in data tags gets irregular (before that it alternates as expected between gyr and acc data with some intermittant time and temp data)
// menuconfig > component config > FreeRTOS > Tick Rate (Hz) = 1000 (the maximum)
static uint8_t imu_read_sample_fifo(bool tempStatus, int* tempData, int* gyroData, int* accData) 
{
    uint8_t hasData = 0b000;
    uint8_t fifoStatus[2];
    uint16_t nSampFifo;

    if (tempStatus) {
        uint8_t tempRaw[2];
        if (imu_register_read(TEMP_L, &tempRaw, 2) == ESP_OK) {
            hasData += 0b100;

            uint16_t temp = (*(tempRaw + 1) << 8) + *(tempRaw);
            *tempData = READ_RAW ? temp : convert_2s_comp(temp);
        }
        else {
            printf("Data not available\n");
        }
    }

    // Read from status register (number of unread samples in FIFO)
    imu_register_read(FIFO_STATUS, &fifoStatus, 2);

    uint8_t fifo_wtm = (*(fifoStatus + 1) >> 5) & (0b100);
    uint8_t fifo_ovr = (*(fifoStatus + 1) >> 5) & (0b010);
    uint8_t fifo_full = (*(fifoStatus + 1) >> 5) & (0b001);

    // If FIFO is full we clear FIFO 
    if (fifo_wtm || fifo_ovr || fifo_full) {
        update_fifo_mode(FIFO_OFF);
        update_fifo_mode(FIFO_CONTI);
        imu_register_read(FIFO_STATUS, &fifoStatus, 2);
        printf("\nFIFO cleared and reset\n\n");
    }
    
    // The number of unread samples in FIFO
    nSampFifo = (*(fifoStatus + 1) & 0b00000011) + (*(fifoStatus));

    if (nSampFifo < 2) {
        // If FIFO doesn't have at least one set of ACC + GYR data we exit function
        return hasData;
    }
    else {
        hasData += 0b011;

        // Remove an even number of samples (to maintain that first sample is always GYRO and second is ACC)
        uint16_t nSampDump = (nSampFifo % 2 == 1) ? nSampFifo - 1 : nSampFifo;
        uint8_t fifoDump[7*nSampDump];
        imu_register_read(FIFO_TAG, &fifoDump, 7*nSampDump);
    
        // Data saved is the most recent data added to FIFO
        uint8_t *fifoData = fifoDump + 7*(nSampDump - 2);

        // First word: GYRO data; Second word: ACC data (if FIFO is not full or overrun)
        fifo_word_to_data(fifoData, gyroData);        
        fifo_word_to_data(fifoData + 7, accData);   

        //printf("FIFO Data: %d, %d, %d, %d, %d, %d, %d\n", *(tempData), *(gyroData), *(gyroData+1), *(gyroData+2), *(accData), *(accData+1), *(accData+2));
        return hasData;
    }
}


/**
 * @brief try to read one sample of temperature, acceleration, and gyroscope
 * @param tempData pointer to int in which to store temperature reading 
 * @param tempData pointer to 3-element array in which to store gyroscope reading 
 * @param tempData pointer to 3-element array in which to store acceleration reading 
 * @param hasData pointer to 8 bits whose least significant 3 bits show if data was available in [temp, gyro, acc]
 */
static uint8_t imu_read_sample(int* tempData, int* gyroData, int* accData) 
{
    uint8_t hasData;
    ESP_ERROR_CHECK(is_data_available(&hasData));

    if (hasData & 0b100) {
        // Has new temperature data
        //printf("%lld\t", esp_timer_get_time());
        uint16_t temp = readHL(TEMP_L);
        
        // Convert from 2's complement to float
        *tempData = READ_RAW ? temp : convert_2s_comp(temp);
    }    
    if (hasData & 0b010) {
        // Has new gyroscope data
        //printf("%lld\t", esp_timer_get_time());
        readAG(GYR_X_L, gyroData);
    }
    if (hasData & 0b001) {
        // Has new acceleration data
        //printf("%lld\t", esp_timer_get_time());
        readAG(ACC_X_L, accData);
    }
    //printf("Reg. Data: %d, %d, %d, %d, %d, %d\n", *(gyroData), *(gyroData+1), *(gyroData+2), *(accData), *(accData+1), *(accData+2));
    return hasData;
}

// A couple helper functions
static float absVal(float num) {
    if (num < 0) return -num;
    else return num;
}
static int find_closest_idx(float num) {
    float opts[] = {12.5, 26, 52, 104, 208, 417, 833, 1667, 3333, 6667};

    float min = 10000.0f;
    int minIdx = 0;
    for (int i = 0; i < 10; i++) {
        if (absVal(opts[i]-num) < min) {
            min = opts[i];
            minIdx = i;
        }
    }
    return minIdx;
}
static float samp2float(int sample, int fullscale) {
    return (float)sample * fullscale / 0x8000;
}
static float acc2float(int acc) 
{
    return samp2float(acc,IMU_ACC_SCALE);
}
static float gyro2float(int gyro) 
{
    return samp2float(gyro,IMU_GYR_SCALE);
}
static float temp2float(int temp)
{
    return (float)temp / TEMP_SENS + TEMP_BIAS;
}

/**
 * @brief i2c set accelerometer and gyroscope batch rates for FIFO (or set 0 to disable FIFO) 
 * @param gyrorate set gyroscope batch rate to 12.5, 26, 52, 104, 208, 417, 833, 1667, 3333, 6667 Hz or 0 to not use FIFO buffer
 * @param accRate set accelerometer batch rate to 12.5, 26, 52, 104, 208, 417, 833, 1667, 3333, 6667 Hz or 0 to not use FIFO buffer
 */
static esp_err_t set_batch_rate(float gyroRate, float accRate) 
{
    uint8_t vals[] = {0b0001, 0b0010, 0b0011, 0b0100, 0b0101, 0b0110, 0b0111, 0b1000, 0b1001, 0b1010, 0b1011};
    uint8_t gyrVal = vals[find_closest_idx(gyroRate)];
    uint8_t accVal = vals[find_closest_idx(accRate)];;

    if (gyroRate < 5)
        gyrVal = 0;
    if (accRate < 5)
        accVal = 0;
    
    uint8_t byteVal = (gyrVal<<4) + accVal;

    return imu_register_write_byte(FIFO_CTRL3, byteVal);
}

/**
 * @brief set accelerometer sampling rate, full-scale, and resolution
 * @param accRate set accelerometer sampling rate to 12.5, 26, 52, 104, 208, 417, 833, 1667, 3333, 6667 Hz or 0 to power down
 * @param accScale set accelerometer full-scale to +/- 2g, 4g, 8g, or 16g
 * @param accResolution set accelerometer resolution to output from first digital filtering stage (1), or LPF2 second filtering stage (2)
*/
static esp_err_t set_acc(float accRate, int accScale, int accResolution) {
    uint8_t vals[] = {0b0001, 0b0010, 0b0011, 0b0100, 0b0101, 0b0110, 0b0111, 0b1000, 0b1001, 0b1010};
    uint8_t rateVal = vals[find_closest_idx(accRate)] << 4;

    uint8_t scaleVal;
    if (accScale < 3)
        scaleVal = 0;
    else if (accScale < 6)
        scaleVal = 0b10;
    else if (accScale < 12)
        scaleVal = 0b11;
    else scaleVal = 0b01;

    scaleVal <<= 2;

    uint8_t resVal;
    if (accResolution <= 1) resVal = 0;
    else resVal = 0b10;

    uint8_t newByte = rateVal | scaleVal | resVal;
    return imu_register_write_byte(CTRL1_XL, newByte);
}

/**
 * @brief set gyroscope sampling rate, full-scale
 * @param gyrRate set gyroscope sampling rate to 12.5, 26, 52, 104, 208, 417, 833, 1667, 3333, 6667 Hz or 0 to power down
 * @param gyrScale set gyroscope full-scale to +/- 125, 250, 500, 1000, 2000, or 4000 dps
*/
static esp_err_t set_gyro(float gyrRate, int gyrScale) {
    uint8_t vals[] = {0b0001, 0b0010, 0b0011, 0b0100, 0b0101, 0b0110, 0b0111, 0b1000, 0b1001, 0b1010};
    uint8_t rateVal = vals[find_closest_idx(gyrRate)] << 4;

    uint8_t scaleVal;
    if (gyrScale < 175)
        scaleVal = 0b0010; // +/- 125 dps
    else if (gyrScale < 350)
        scaleVal = 0b0000; // +/- 250 dps
    else if (gyrScale < 750)
        scaleVal = 0b0100; // +/- 500 dps
    else if (gyrScale < 1500)
        scaleVal = 0b1000; // +/- 1000 dps
    else if (gyrScale < 3000) 
        scaleVal = 0b1100; // +/- 2000 dps
    else scaleVal = 0b0001; // +/- 4000 dps

    uint8_t newByte = rateVal | scaleVal;
    return imu_register_write_byte(CTRL2_G, newByte);
}



/**
 * @brief turn IMU FIFO on or off or put it in continuous mode
 * FIFO_ON => data is added to FIFO buffer until it is full
 * FIFO_OFF => data is not added to FIFO buffer (where does it go?)
 * FIFO_CONTI => data is added to FIFO buffer until it is Full. Then new data replaces oldest
 * @param tempRate batch temperature at 1.6, 12.5, or 52 Hz
 * @param fifoMode FIFO_ON, FIFO_OFF, or FIFO_CONTI
*/
static esp_err_t set_fifo_mode(int fifoMode) {
    // We are receiving data in sets of 2 words (14 bytes): we set watermark threshold to be a multiple of 14
    imu_register_write_byte(FIFO_CTRL1, 0b11111100); 
    imu_register_write_byte(FIFO_CTRL2, 0b00000000);
    
    uint8_t newByte; 
    newByte = (TS_DEC_FIFO << 6) + (TEMP_RATE_FIFO << 4) + fifoMode;

    return imu_register_write_byte(FIFO_CTRL4,newByte);
}

/**
 * @brief i2c reset IMU FIFO buffer and ESP32 I2C buffers
 */
static void imu_reset_buffers() 
{
    // Reset FIFO Ctrl buffers
    ESP_ERROR_CHECK(imu_register_write_byte(FIFO_CTRL1, 0));
    ESP_ERROR_CHECK(imu_register_write_byte(FIFO_CTRL2, 0));
    ESP_ERROR_CHECK(imu_register_write_byte(FIFO_CTRL3, 0));
    ESP_ERROR_CHECK(imu_register_write_byte(FIFO_CTRL4, 0));

    // Reset ESP-side buffers
    ESP_ERROR_CHECK(i2c_reset_tx_fifo(I2C_MASTER_NUM));
    ESP_ERROR_CHECK(i2c_reset_rx_fifo(I2C_MASTER_NUM));
}

static esp_err_t imu_reset() 
{
    // Write 0b00000001 to CTRL3_C register
    return imu_register_write_byte(0x12,1);
}

/**
 * @brief i2c master initialization
 */
static void init_imu(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_GPIO,
        .scl_io_num = SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_DISABLE, // Pulled up externally
        .scl_pullup_en = GPIO_PULLUP_DISABLE, // Pulled up externally
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    // Set up driver 
    ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0));
    
    // Set gyroscope rate to 104Hz, scale to 1000dps
    ESP_ERROR_CHECK(set_gyro(IMU_GYR_RATE, IMU_GYR_SCALE));
    // Set accelerometer rate to 104Hz, scale to +/- 4g, resolution to 1 digital filter
    ESP_ERROR_CHECK(set_acc(IMU_ACC_RATE, IMU_ACC_SCALE, 1));

    // Reset buffers
    imu_reset_buffers();
    uint8_t fifoCtrl[4];
    imu_register_read(FIFO_CTRL1, fifoCtrl, 4);
    printf("FIFO Ctrl Buffers After Reset: %d, %d, %d, %d\n", fifoCtrl[0], fifoCtrl[1], fifoCtrl[2], fifoCtrl[3]);

    set_fifo_mode(FIFO_CONTI);
    set_batch_rate(IMU_GYR_RATE, IMU_ACC_RATE);
    imu_register_read(FIFO_CTRL1, fifoCtrl, 4);
    printf("FIFO Ctrl: %d, %d, %d, %d\n", fifoCtrl[0], fifoCtrl[1], fifoCtrl[2], fifoCtrl[3]);
}

static void close_imu()
{
    ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
    ESP_LOGI(TAG, "I2C de-initialized successfully");
}

/// @brief Print an IMU reading. READ_RAW should be 0 for this
void print_imu_reading() 
{
    // I2C read IMU data
    int temp; int gyro[3]; int acc[3];
    uint8_t hasData = imu_read_sample(&temp, gyro, acc);

    if (hasData & 0b100) {
        // Has new temperature data
        ESP_LOGI(TAG, "Temperature = %.2f", temp2float(temp));
    }    
    if (hasData & 0b010) {
        // Has new gyroscope data
        ESP_LOGI(TAG, "Gyroscope = %.2f, %.2f, %.2f", gyro2float(gyro[0]), gyro2float(gyro[1]), gyro2float(gyro[2]));
    }
    if (hasData & 0b001) {
        // Has new acceleration data
        ESP_LOGI(TAG, "Accelerometer = %.2f, %.2f, %.2f", acc2float(acc[0]), acc2float(acc[1]), acc2float(acc[2]));
    }
}

/// @brief Obtain a reading from the IMU to send to host. READ_RAW should be 1 for this so values are in 2s comp
void get_imu_reading(uint16_t* sample, bool tempStatus) 
{
    // I2C read IMU data
    int temp; int gyro[3]; int acc[3];

    //uint8_t hasData = imu_read_sample(&temp, gyro, acc);
    uint8_t hasData = imu_read_sample_fifo(tempStatus, &temp, gyro, acc);

    if (hasData & 0b100) {
        // Has new temperature data
        sample[0] = (uint16_t)temp;
    }    
    if (hasData & 0b010) {
        // Has new gyroscope data
        sample[1] = (uint16_t)gyro[0];
        sample[2] = (uint16_t)gyro[1];
        sample[3] = (uint16_t)gyro[2];
    }
    if (hasData & 0b001) {
        // Has new acceleration data
        sample[4] = (uint16_t)acc[0];
        sample[5] = (uint16_t)acc[1];
        sample[6] = (uint16_t)acc[2];
    }
}

#endif //MAGFT_IMU_H