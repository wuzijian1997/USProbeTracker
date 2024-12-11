#ifndef MAGFT_PINDEF_H
#define MAGFT_PINDEF_H
/*
Pins in use
*/ 

#include "magft_settings.h"

#define TRUE 1
#define FALSE 0

#define N_VSPI_SENSORS 3
#define N_HSPI_SENSORS 3
#define N_SENSORS 6             // N_SENSORS := N_VSPI_SENSORS + N_HSPI_SENSORS

// -- PINS FOR OLD MASTER PCB --
// VSPI Pins
// Note: all pins are referenced by GPIO number, not pin number
// #define VCLK 18
// #define VMISO 19
// #define VMOSI 23

// #define V_CS1 21
// #define V_CS2 5
// #define V_CS3 17

// // HSPI Pins
// #define H_CS4 32
// #define H_CS5 25
// #define H_CS6 26

// #define HCLK 14
// #define HMISO 12
// #define HMOSI 13

// int VSPI_PINS[] = {VCLK, VMISO, VMOSI, V_CS1, V_CS2, V_CS3};
// int HSPI_PINS[] = {HCLK, HMISO, HMOSI, H_CS4, H_CS5, H_CS6};


// -- PINS FOR NEW MASTER PCB --
// VSPI Pins
// Note: all pins are referenced by GPIO number, not pin number
#define VCLK 18
#define VMISO 19 
#define VMOSI 23

#define V_CS4 21
#define V_CS5 5
#define V_CS6 17

// HSPI Pins (These are actually CS1, 2, 3)
#define H_CS1 33
#define H_CS2 32
#define H_CS3 25

#define HCLK 14
#define HMISO 12
#define HMOSI 13

int VSPI_PINS[] = {VCLK, VMISO, VMOSI, V_CS4, V_CS5, V_CS6};
int HSPI_PINS[] = {HCLK, HMISO, HMOSI, H_CS1, H_CS2, H_CS3};

// IMU I2C pins (CS is pulled high)
#define SCL_GPIO                    22
#define SDA_GPIO                    4  
#define IMU_ADDRESS                 0b1101011                  /*!< SA0 is pulled high, so LSB is 1 */
#define IMU_WHO_AM_I_REG_ADDR       0x0F                       /*!< Register addresses of the "who am I" register */   
#define FIFO_CTRL1                  0x07                       /*!< Register address of FIFO watermark threshold*/
#define FIFO_CTRL2                  0x08                       /*!< Register address of FIFO watermark threshold*/
#define FIFO_CTRL3                  0x09                       /*!< Register address of Accel/Gyro rate register */
#define FIFO_CTRL4                  0x0A                       /*!< Register address of FIFO control register*/
#define CTRL1_XL                    0x10                       /*!< Register address of accel settings register */
#define CTRL2_G                     0x11                       /*!< Register address of gyroscope settings register */
#define STATUS_REG                  0x1E                       /*!< Register showing the availability of new data */
#define TEMP_L                      0x20                       /*!< Register address of MSB of temperature reading */
#define TEMP_H                      0x21                       /*!< Register address of LSB of temperature reading */
#define GYR_X_L                     0x22                       /*!< Register address of MSB of gyroscope x reading */
#define GYR_X_H                     0x23                       /*!< Register address of LSB of gyroscope x reading */
#define GYR_Y_L                     0x24                       /*!< Register address of MSB of gyroscope y reading */
#define GYR_Y_H                     0x25                       /*!< Register address of LSB of gyroscope y reading */
#define GYR_Z_L                     0x26                       /*!< Register address of MSB of gyroscope z reading */
#define GYR_Z_H                     0x27                       /*!< Register address of LSB of gyroscope z reading */
#define ACC_X_L                     0x28                       /*!< Register address of MSB of accel x reading */
#define ACC_X_H                     0x29                       /*!< Register address of LSB of accel x reading */
#define ACC_Y_L                     0x2A                       /*!< Register address of MSB of accel y reading */
#define ACC_Y_H                     0x2B                       /*!< Register address of LSB of accel y reading */
#define ACC_Z_L                     0x2C                       /*!< Register address of MSB of accel z reading */
#define ACC_Z_H                     0x2D                       /*!< Register address of LSB of accel z reading */
#define FIFO_STATUS                 0x3A                       /*!< Register address that contains number of unread samples in FIFO */
#define FIFO_TAG                    0x78                       /*!< Register address of FIFO_DATA_OUT_TAG: 0x79-0x7E are registers where ACC and GYR data is stored */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

// I2C IMU FIFO constants
#define FIFO_ON                     1
#define FIFO_OFF                    0
#define FIFO_CONTI                  6

#define IMU_RATE                    417                         /*!< Rate at which GYR and ACC is added to FIFO (Hz); CHOOSE >= 417 HZ */

// WiFi constants
#define LOCAL_PORT_UDP 5004
#define REMOTE_PORT_UDP 5004
#define WIFI_PERIOD 50
#define UART 0
#define WIFI 1

// Dual-core constants
#define CPU0 0 // For wireless networking tasks
#define CPU1 1 // For other tasks
#define MAX_OUT_BUF 10 // Max number of samples to save before removing the old ones
#define TEMP_Q N_SENSORS // Index of measurement queue containing temperatures
#define GYRO_Q N_SENSORS+1
#define ACC_Q N_SENSORS+2


#endif //MAGFT_PINDEF_H