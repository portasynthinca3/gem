#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

#include "cpu.h"
#include "machine.h"

#define TAG "main"

void main_task(void* param) {
    cpu_disasm(BIOS_START, 512);
    while(1)
        vTaskDelay(10000);

    // while(1)
    //     cpu_run();
}

void app_main(void) {
    // reset machine
    machine_init();
    cpu_reset();
    ESP_LOGI(TAG, "machine initialized");

    // run machine
    xTaskCreate(main_task, "Emulation", 4096, NULL, 7, NULL);
    ESP_LOGI(TAG, "emulation task started");

    while(1)
        vTaskDelay(10000);
}