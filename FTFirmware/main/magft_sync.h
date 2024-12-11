#ifndef MAGFT_SYNC_H
#define MAGFT_SYNC_H

#include "magft_pindef.h"
#include "freertos/queue.h"

// Buffers for new measurements
// sampBuf[0-N_SENSORS-1] contain sensor readings
// sampBuf[N_SENSORS] contains IMU temperature readings
// sampBuf[N_SENSORS+1] contains IMU gyroscope readings
// sampBuf[N_SENSORS+2] contains IMU acceleration readings

QueueHandle_t sampQ[2];

#define MES_IDX 0x00
#define IMU_IDX 0x01

#define MES_LEN N_SENSORS
#define IMU_LEN 7

/*
static void test_speed_queue() {
    uint16_t ptr;
    uint16_t testSamp = 0b1101010101111011;
    printf("%lld\t", esp_timer_get_time());
    xQueueSend( testQ, &testSamp, ( TickType_t ) 0 );
    printf("%lld\t", esp_timer_get_time());
    uxQueueSpacesAvailable(testQ);
    printf("%lld\t", esp_timer_get_time());
    xQueuePeek( testQ, &ptr, ( TickType_t ) 0 );
    printf("%lld\t", esp_timer_get_time());
    xQueueReceive( testQ, &ptr, ( TickType_t ) 0 );
    printf("%lld\n", esp_timer_get_time()); 
}
*/

/// @brief Set up queue to hold all measurement samples for synchronized readout
static void init_sample_queue() 
{
    sampQ[MES_IDX] = xQueueCreate( 5, sizeof( uint16_t ) * MES_LEN );   // MEASURE_TASK data
    sampQ[IMU_IDX] = xQueueCreate( 5, sizeof( uint16_t ) * IMU_LEN );   // IMU_TASK data
}

// For debugging
void print_vals_16(uint16_t* sample, int n) 
{
    for (int i = 0; i < n; i++) {
        printf("%d\t", sample[i]);
    }
    printf("\n");
}

void print_vals_8(uint8_t* sample, int n) 
{
    for (int i = 0; i < n; i++) {
        printf("%d\t", sample[i]);
    }
    printf("\n");
}

/// @brief Push new measurement into sample queue
/// @param idx index of queue to push it into 
/// @param sample the new measurement
static void save_sample(uint8_t idx, uint16_t *sample) 
{
    int sampSize = (idx == MES_IDX) ? MES_LEN : IMU_LEN;
    if (uxQueueSpacesAvailable(sampQ[idx]) == 0)
    {
        uint16_t old[sampSize]; 
        // Queue is full. Remove oldest sample first
        xQueueReceive( sampQ[idx], old, (TickType_t) 10);
    }

    // Save new reading ( void * )
    xQueueSend( sampQ[idx],  sample, ( TickType_t ) 0 );
}


static void no_sample_available(uint16_t* sample, int len) {
    for (int i = 0; i < len; i++) {
        sample[i] = 0;
    }
}

/// @brief Put the next available samples of all the sensors in the sample array
/// @param sample Set an element to 0 if that measurement is not yet available
static bool fetch_sample(uint16_t* sample) 
{
    //print_vals(sample, 6);
    bool mesStatus = xQueueReceive( sampQ[MES_IDX], sample, (TickType_t) 0) ? TRUE : FALSE ;
    if (!mesStatus) {
        no_sample_available(sample, MES_LEN);
    }
    //print_vals(sample, 13);

    // Save the two vector values (acc & gyro)
    //printf("%lld\t", esp_timer_get_time());

    bool imuStatus = xQueueReceive( sampQ[IMU_IDX], sample + MES_LEN, (TickType_t) 0) ? TRUE : FALSE ;
    if (!imuStatus) {
        no_sample_available(sample + MES_LEN, IMU_LEN);
    } 
    //print_vals(sample, 13);
    return (mesStatus || imuStatus);
}

void print_sample(uint16_t* sample) 
{
    for (int i = 0; i < N_SENSORS; i++)
        ESP_LOGI(TAG, "Sensor %d: %d", i+1, sample[i]);
    ESP_LOGI(TAG, "Temperature = %d", sample[N_SENSORS]);
    ESP_LOGI(TAG, "Gyroscope = %d, %d, %d", sample[N_SENSORS+1],sample[N_SENSORS+2],sample[N_SENSORS+3]);
    ESP_LOGI(TAG, "Accelerometer = %d, %d, %d", sample[N_SENSORS+4],sample[N_SENSORS+5],sample[N_SENSORS+6]);
}

#endif // MAGFT_SYNC_H