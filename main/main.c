#include <stdio.h>
#include "nrf.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "MAIN"

 

void app_main(void)
{
    uint8_t device_id[5];
    uint8_t host_address[5];

    device_id[0] = 0x34;
    device_id[1] = 0x34;
    device_id[2] = 0x34;
    device_id[3] = 0x34;
    device_id[4] = 0x34;

    host_address[0] = 0xF8;
    host_address[1] = 0x65;
    host_address[2] = 0x64;
    host_address[3] = 0x22;
    host_address[4] = 0x7B;
     uint32_t command = 1;
    uint32_t channel = 1;
    while (1)
    {
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        nrf_send_command_to_devices(command, device_id, channel, 1, host_address);
    }
}
