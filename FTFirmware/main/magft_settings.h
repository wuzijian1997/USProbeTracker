#ifndef MAGFT_SETTINGS_H
#define MAGFT_SETTINGS_H

#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "magft_uart.h"
#include <math.h>

static const char *TAG = "MAGFT";

// Flag to permit other tasks to use UART
bool doneSettings = 0;

// Default values
bool SEND_MODE = 0;
uint8_t WIFI_SSID_LENGTH = 13;
uint8_t WIFI_PASS_LENGTH = 11;
uint8_t ESP_WIFI_SSID[32]; 
uint8_t ESP_WIFI_PASS[32];

// WiFi attributes
// uint8_t IP_ADDRESS[4] = {192, 168, 1, 75};

uint8_t IP_ADDRESS[4] = {10, 0, 0, 77};
uint8_t NETMASK_ADDRESS[4] = {255, 255, 255, 0};
uint8_t GATEWAY_ADDRESS[4] = {0};

nvs_handle_t nvshandle;

void print_current_settings() 
{
    ESP_LOGI(TAG, "\nSend mode: %d\nWiFi SSID: %s\nWiFi Password: %s\nHost IP: %d.%d.%d.%d\nSubnet Mask: %d.%d.%d.%d\nGateway Address: %d.%d.%d.%d", 
        (uint8_t)SEND_MODE, ESP_WIFI_SSID, ESP_WIFI_PASS,
        IP_ADDRESS[0],IP_ADDRESS[1],IP_ADDRESS[2],IP_ADDRESS[3],
        NETMASK_ADDRESS[0],NETMASK_ADDRESS[1],NETMASK_ADDRESS[2],NETMASK_ADDRESS[3],
        GATEWAY_ADDRESS[0],GATEWAY_ADDRESS[1],GATEWAY_ADDRESS[2],GATEWAY_ADDRESS[3]);
}

// Convenience methods to match nvs.h and Preferences.h (arduino) APIs
void putUChar(const char* key, uint8_t val) {
    ESP_ERROR_CHECK(nvs_set_u8(nvshandle, key, val));
}
void putString(const char* key, uint8_t* val, int len) {
    
    if (len == 4) {
        // This is an IP address and has no null termination
        const char ipstr[5] = {val[0], val[1], val[2], val[3], '\0'};
        ESP_ERROR_CHECK(nvs_set_str(nvshandle, key, ipstr));
    } else ESP_ERROR_CHECK(nvs_set_str(nvshandle, key, (const char*)val));
    
}
uint8_t getUChar(const char* key, uint8_t defaultVal) {
    uint8_t outval = defaultVal;
    if (nvs_get_u8(nvshandle, key, &outval) == ESP_OK) {
        return outval;
    } else return defaultVal;
}
bool getString(const char* key, char *outStr, size_t *length) {
    if (*length == 4) {
        // This is an IP address and should have no null termination
        char tempOut[5];
        size_t l5 = 5;
        bool e = (nvs_get_str(nvshandle, key, tempOut, &l5) == ESP_OK); 
        // printf("IP address: %x, %x, %x, %x, %x\n", tempOut[0], tempOut[1], tempOut[2], tempOut[3], tempOut[4]);
        if (e) {
            // Copy the array over, excluding the null termination
            for (int i = 0; i < 4; i++)
                outStr[i] = tempOut[i];
        }
        return e;
    }
    else return (nvs_get_str(nvshandle, key, outStr, length) == ESP_OK);
}
void set_str(uint8_t* dest, uint8_t* source, uint8_t length)
{
    for (int i = 0; i < length-1; i++) {
        dest[i] = source[i];
    }
    dest[length-1] = '\0';
}

void set_arr(uint8_t* dest, uint8_t* source, uint8_t length) 
{
    for (int i = 0; i < length; i++)
        dest[i] = source[i];
}

void handle_wifi_setting(uint8_t length, bool ssid)
{
    // Try to read the response, which is the new string value
    uint8_t newWifi[length];
    int numBytes = read_uart(newWifi, length, 1000);

    // Save the returned value if the length is correct
    if (numBytes == length) {
        if (ssid) {
            WIFI_SSID_LENGTH = length+1;
            set_str(ESP_WIFI_SSID, newWifi, WIFI_SSID_LENGTH);
            putString("WIFI_SSID", ESP_WIFI_SSID, WIFI_SSID_LENGTH);
            putUChar("SSID_LENGTH", WIFI_SSID_LENGTH);
            ESP_LOGI(TAG, "Set WiFi SSID to %s", ESP_WIFI_SSID);
        } else {
            WIFI_PASS_LENGTH = length+1;
            set_str(ESP_WIFI_PASS, newWifi, WIFI_PASS_LENGTH);
            putString("WIFI_PASS", ESP_WIFI_PASS, WIFI_PASS_LENGTH);
            putUChar("PASS_LENGTH", WIFI_PASS_LENGTH);
            ESP_LOGI(TAG, "Set WiFi password to %s", ESP_WIFI_PASS);
        }
    } else ESP_LOGI(TAG, "Failed to set WiFi setting");
}

void set_udp_server() 
{
    // Try to read the IP Address
    uint8_t newWifi[4];
    int numBytes = read_uart(newWifi, 4, 1000);

    // Save the returned value if the length is correct
    if (numBytes == 4) {
        set_arr(IP_ADDRESS, newWifi, 4);
        putString("WIFI_IP", IP_ADDRESS, 4);
        ESP_LOGI(TAG, "Set IP Address to %d.%d.%d.%d", IP_ADDRESS[0],IP_ADDRESS[1],IP_ADDRESS[2],IP_ADDRESS[3]);
    } else ESP_LOGI(TAG, "Failed to set IP address");

    // Try to read the subnet mask
    numBytes = read_uart(newWifi, 4, 1000);

    // Save the returned value if the length is correct
    if (numBytes == 4) {
        set_arr(NETMASK_ADDRESS, newWifi, 4);
        putString("WIFI_NETMASK", NETMASK_ADDRESS, 4);
        ESP_LOGI(TAG, "Set subnet mask to %d.%d.%d.%d", NETMASK_ADDRESS[0],NETMASK_ADDRESS[1],NETMASK_ADDRESS[2],NETMASK_ADDRESS[3]);
    } else ESP_LOGI(TAG, "Failed to set subnet mask");
    
    // Try to read the Gateway address
    numBytes = read_uart(newWifi, 4, 1000);

    // Save the returned value if the length is correct
    if (numBytes == 4) {
        set_arr(GATEWAY_ADDRESS, newWifi, 4);
        putString("WIFI_GATEWAY", GATEWAY_ADDRESS, 4);
        ESP_LOGI(TAG, "Set Gateway Address to %d.%d.%d.%d", GATEWAY_ADDRESS[0],GATEWAY_ADDRESS[1],GATEWAY_ADDRESS[2],GATEWAY_ADDRESS[3]);
    } else ESP_LOGI(TAG, "Failed to set Gateway address");
}

void set_value(uint8_t setting, uint8_t value) 
{
    switch (setting) {
        case 0:
            // Request for current settings
            // Value = 1 if we should send current settings
            if (value)
                print_current_settings();
            break;
        case 1:
            // Change communication mode (UART or UDP)
            // Value 0 = UART, 1 = UDP
            if (value == 0 || value == 1) {
                SEND_MODE = value;
                putUChar("SEND_MODE",value);
                ESP_LOGI(TAG,"Set SEND_MODE to %d", value);
            }
            break;
        case 2:
            // Change WiFi SSID
            // Value = length of incoming SSID (0 if no update)
            if (value > 0)
                handle_wifi_setting(value, 1);
            break;
        case 3:
            // Change WiFi Password
            // Value = length of incoming password (0 if no update)
            if (value > 0)
                handle_wifi_setting(value, 0);
            break;
        case 4:
            if (value == 1)
                set_udp_server();
    }
    ESP_ERROR_CHECK(nvs_commit(nvshandle));
}

void update_all_settings() {
    print_current_settings();
    SEND_MODE = getUChar("SEND_MODE", SEND_MODE);
    WIFI_SSID_LENGTH = getUChar("SSID_LENGTH", WIFI_SSID_LENGTH);
    WIFI_PASS_LENGTH = getUChar("PASS_LENGTH", WIFI_PASS_LENGTH);

    uint8_t* newSSID[WIFI_SSID_LENGTH]; // note, sizeof(uint8_t) = 1
    if (getString("WIFI_SSID", newSSID, &WIFI_SSID_LENGTH))
        set_str(ESP_WIFI_SSID, (uint8_t*)newSSID, WIFI_SSID_LENGTH);
    
    uint8_t* newPass[WIFI_PASS_LENGTH];
    if (getString("WIFI_PASS", newPass, &WIFI_PASS_LENGTH))
        set_str(ESP_WIFI_PASS, (uint8_t*)newPass, WIFI_PASS_LENGTH);

    uint8_t* newIP[4];
    size_t length = 4;
    if (getString("WIFI_IP", newIP, &length))
        set_arr(IP_ADDRESS,newIP,4);
    else ESP_LOGE(TAG,"Failed to read IP address from NVS");
    if (getString("WIFI_NETMASK", newIP, &length))
        set_arr(NETMASK_ADDRESS,newIP,4);
    else ESP_LOGE(TAG,"Failed to read subnet mask from NVS");
    if (getString("WIFI_GATEWAY", newIP, &length))
        set_arr(GATEWAY_ADDRESS,newIP,4);
    else ESP_LOGE(TAG,"Failed to read gateway address from NVS");
    print_current_settings();
}

void save_all_settings() 
{
    putUChar("SEND_MODE", SEND_MODE);
    putString("WIFI_SSID", ESP_WIFI_SSID, WIFI_SSID_LENGTH);
    putString("WIFI_PASS", ESP_WIFI_PASS, WIFI_PASS_LENGTH);
    putUChar("SSID_LENGTH", WIFI_SSID_LENGTH);
    putUChar("PASS_LENGTH", WIFI_PASS_LENGTH);
    putString("WIFI_IP", IP_ADDRESS, 4);
    putString("WIFI_NETMASK", NETMASK_ADDRESS, 4);
    putString("WIFI_GATEWAY", GATEWAY_ADDRESS, 4);
    ESP_ERROR_CHECK(nvs_commit(nvshandle));
}

void read_settings_task() 
{
    // First, set default wifi ssid and password
    char* defaultSSID = "Rogers-5G-FWA";
    set_str(ESP_WIFI_SSID,(uint8_t*)defaultSSID,WIFI_SSID_LENGTH);
    char* defaultPass = "Rogers2022!";
    set_str(ESP_WIFI_PASS,(uint8_t*)defaultPass,WIFI_PASS_LENGTH);
    

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );

    err = nvs_open("settings", NVS_READWRITE, &nvshandle);
    // printf("\n%s\n", esp_err_to_name(err));
    if (err != ESP_OK) {
        ESP_LOGI(TAG,"Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "NVS opened to read settings");

        // Load current settings from non-volatile storage (NVS)
        update_all_settings(); // Set all settings from NVS (if they don't exist this does nothing)
        save_all_settings(); // Save all settings in NVS (if they're already saved, this does nothing)

        uint8_t newSettings[10]; // # of possible settings x 2
        for (int i = 0; i < 3; i++) {
            
            // Try to read some bytes for 1 second
            int numBytes = read_uart(newSettings, 10, 1000);

            if (numBytes > 0) {
                // Parse the new settings
                for (int j = 0; j < numBytes; j += 2) {
                    if (j + 1 < numBytes)
                        set_value(newSettings[j], newSettings[j+1]);
                }

                // Stop reading
                break;
            }

            // Delay briefly to allow FreeRTOS to do whatever
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }

        // Close
        nvs_close(nvshandle);
    }

    ESP_LOGI(TAG,"Done scanning for settings changes");
    doneSettings = 1;
}

#endif //MAGFT_SETTINGS_H