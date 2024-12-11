#include <stdio.h>
#include "magft_pindef.h"
#include "magft_imu.h"
#include "magft_uart.h"
#include "magft_wifi.h"
#include "magft_spi.h"
#include "magft_sync.h"
#include "magft_settings.h"
#include "wifiudp.h"
#include "esp_timer.h"

TaskHandle_t xHandle0;  // COMM_TASK: pinned to CPU0; only task we suspend/resume
TaskHandle_t xHandle1;  // MES_TASK: pinned to CPU0; runs when COMM_TASK is suspended
TaskHandle_t xHandle2;  // IMU_TASK: pinned to CPU1; significantly slower than other tasks

SemaphoreHandle_t serialMutex; //Mutex for the serial communication (used so we aren't sending serial commands at the same time)

const int tempFreq = 200;               // Take temperature reading every tempFreq IMU readings (~every second)


/**
 * @brief Initializes VSPI and HSPI and reads from N_SENSORS SPI devices
 * Runs when COMM_TASK (the higher priority task) is suspended
*/
void measure_task()
{
    // Initialize SPI buses (VSPI and HSPI) which each have max. 3 devices each
    init_spi(VSPI_HOST);
    init_spi(HSPI_HOST);

    uint16_t spiData[N_SENSORS * 2];    

    // Infinite loop
    while(1) 
    { 
        // Reads data from SPI using polling transaction
        spi_read_sample_polling(spiData, VSPI_HOST);
        spi_read_sample_polling(spiData+6, HSPI_HOST);

        //Aquire the mutex before sending over serial:
        if(xSemaphoreTake(serialMutex,portMAX_DELAY))
        {
            printf("FSN: %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
            spiData[0], spiData[1], spiData[2], spiData[3], spiData[4], spiData[5],
            spiData[6], spiData[7], spiData[8], spiData[9], spiData[10], spiData[11]);
            
            //Release the mutex
            xSemaphoreGive(serialMutex);
        }

        //printf("%lld:", esp_timer_get_time());
        

        // Save samples to queue
        //save_sample(MES_IDX,spiData);
        // COMM_TASK (the higher priority task) is suspended at this time, we resume it
        //vTaskResume( xHandle0 );
        
        vTaskDelay(1); //Maybe we get rid of this
    }

    // Close some drivers
    close_spi(VSPI_HOST);
    close_spi(HSPI_HOST);
    vTaskDelete(NULL); // Delete this task
}

/**
 * @brief Initializes IMU: reads ACC and GYR data and TEMP data every tempCount IMU readings
*/
void imu_task()
{
    // Connect to I2C
    init_imu();
    ESP_LOGI(TAG,"I2C initialized successfully");

    // Take temperature reading every tempFreq IMU readings (~every second)
    int tempCount = 0;
    bool *tempStatus = FALSE;

    // Infinite loop, fetching IMU readings every imuPeriod milliseconds
    while(1) 
    {
        uint16_t sample[7] = {0};

        if (tempCount >= tempFreq && tempFreq  != 0) {
            tempStatus = TRUE;
            tempCount = 0;
        }
        else if (tempFreq  != 0){
            tempStatus = FALSE;
            tempCount++;
        }
        get_imu_reading(sample, tempStatus);
        //print_vals(sample,7);

        // Save sample to queue
        //save_sample(IMU_IDX, sample);

        //Aquire the mutex before sending over serial
        if(xSemaphoreTake(serialMutex,portMAX_DELAY))
        {
            /*
            0=temp
            1,2,3=gyro
            4,5,6=accelerometer
            */
            printf("TGA: %d,%d,%d,%d,%d,%d,%d\n",
            sample[0],sample[1],sample[2],sample[3],sample[4],sample[5],sample[6]);
            xSemaphoreGive(serialMutex);
        }

        // For debugging queues
        // uint16_t data[8] = {0};
        // fetch_sample(data);
        // ESP_LOGI(TAG,"Fetched");
        // print_vals(data+1,7);
    }

    close_imu();
    vTaskDelete(NULL); // Delete this task
}



// Main application
void app_main(void)
{
    // Initialize UART
    init_uart();
    ESP_LOGI(TAG, "UART init. successful");
    char startMsg[] = {0x00, 0xFF, 0xFF};
    write_uart(startMsg,3);

    //Creates the mutex for serial communication
    serialMutex=xSemaphoreCreateMutex();
    if(serialMutex==NULL)
    {
        ESP_LOGE("MAIN","Failed to create serial mutex!");
    }


    // Block execution while waiting to receive any new settings
    read_settings_task();
    
    // Create queue for measurements
    //init_sample_queue();

    // Void parameter to be passed for no reason
    static uint8_t* param;

    // Create the task to handle wifi communication on core 0, storing the handle. 
    //xTaskCreatePinnedToCore( wifi_comm_task, "COMM_TASK", 3072, param, tskIDLE_PRIORITY + 1, &xHandle0 , CPU0 );
    //configASSERT( xHandle0 );

    // Start task to handle sensor measurements on core 1
    xTaskCreatePinnedToCore( measure_task, "MEASURE_TASK", 4096, param, tskIDLE_PRIORITY, &xHandle1 , CPU0 );
    configASSERT( xHandle1 );

    // Start task to handle IMU readings, not pinned to anything -> lower priority, lower frequency (?)
    xTaskCreatePinnedToCore( imu_task, "IMU_TASK", 4096, param, tskIDLE_PRIORITY, &xHandle2 , CPU1 );
    configASSERT( xHandle2 );
    //measure_task();
}