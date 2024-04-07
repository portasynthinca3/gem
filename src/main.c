#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

#include "cpu.h"
#include "machine.h"

#define TAG "main"

void main_task(void* param) {
    cpu_repl_init();
    machine_loop();
}

void app_main(void) {
    // reset machine
    machine_init();
    cpu_reset();
    ESP_LOGI(TAG, "machine initialized");

    // run machine
    xTaskCreatePinnedToCore(main_task, "Emulation", 4096, NULL, 0, NULL, 0);
    ESP_LOGI(TAG, "emulation task started");

    while(1)
        vTaskDelay(10000);
}