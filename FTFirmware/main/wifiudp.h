#ifndef WIFIUDP_H
#define WIFIUDP_H

/* BSD Socket API Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "magft_pindef.h"

static const char *payload = "Message from ESP32 ";
struct sockaddr_in dest_addr;
int sockt;

static void init_udp()
{
    int addr_family = 0;
    int ip_protocol = 0;

    // Set up destination address
    dest_addr.sin_addr.s_addr = inet_addr((char *)IP_ADDRESS);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(REMOTE_PORT_UDP);
    addr_family = AF_INET;
    ip_protocol = IPPROTO_IP;

    // Create UDP socket
    sockt = socket(addr_family, SOCK_DGRAM, ip_protocol);
    if (sockt < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
    }

    // Set timeout
    struct timeval timeout;
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;
    setsockopt (sockt, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);

    ESP_LOGI(TAG, "Socket created, sending to %s:%d", IP_ADDRESS, REMOTE_PORT_UDP);

}

static err_t send_udp_msg(uint8_t *data, int length) 
{
    int err = sendto(sockt, (const char *)data, length, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err < 0) {
        //ESP_LOGE(TAG, "Error occurred during UDP sending: dropped %d sample", (data[2] << 8) + data[3]);
    }
    return err;
}

static void receive_udp() 
{
    char rx_buffer[32];
    struct sockaddr_storage source_addr;
    socklen_t socklen = sizeof(source_addr);

    int len = recvfrom(sockt, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

    // Error occurred during receiving
    if (len < 0) {
        //ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
    }
    // Data received
    else {
        // For testing
        rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
        //ESP_LOGI(TAG, "Received %d bytes from %s:", len, IP_ADDRESS);
        //ESP_LOGI(TAG, "%s", rx_buffer);
    }
}

static void close_udp() 
{
    if (sockt != -1) {
        ESP_LOGE(TAG, "Shutting down socket...");
        shutdown(sockt, 0);
        close(sockt);
    }
}

#endif // WIFIUDP_H