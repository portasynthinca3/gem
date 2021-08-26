#include "machine.h"
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <esp_log.h>

#define TAG "machine"

// Memory

uint8_t* main_ram = NULL;
extern const uint8_t bios_data_start[] asm("_binary_bios_bin_start");
extern const uint8_t bios_data_end[] asm("_binary_bios_bin_end");

// Public functions

void machine_init(void) {
    // allocate main RAM
    main_ram = realloc(main_ram, RAM_SIZE);
    memset(main_ram, 0, RAM_SIZE);
}

uint8_t _machine_mem_rd(uint32_t addr) {
    if(addr < RAM_SIZE)
        return *(main_ram + addr);

    if(addr >= BIOS_START)
        return *(bios_data_start + (addr - BIOS_START));

    // dead memory
    return 0;
}

void _machine_mem_wr(uint32_t addr, uint8_t val) {
    if(addr < RAM_SIZE) {
        *(main_ram + addr) = val;
        return;
    }

    // dead memory
    return;
}

#ifdef TRACE_MEM_ACCESS
uint8_t machine_mem_rd(uint32_t addr) {
    uint8_t val = _machine_mem_rd(addr);
    ESP_LOGI(TAG, "trace: RD 0x%05x: %02x", addr, val);
    return val;
}

void machine_mem_wr(uint32_t addr, uint8_t val) {
    ESP_LOGI(TAG, "trace: WR 0x%05x: %02x", addr, val);
    _machine_mem_wr(addr, val);
}
#endif