#include "machine.h"
#include "cpu.h"
#include "pic.h"
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <esp_log.h>
#include <esp_timer.h>

#define TAG "machine"

// Memory

uint8_t* main_ram = NULL;
static uint32_t ips_pre, ips;
extern const uint8_t bios_data_start[] asm("_binary_bios_bin_start");
extern const uint8_t bios_data_end[] asm("_binary_bios_bin_end");

// Private functions

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
}

void _machine_io_wr8(uint16_t port, uint8_t val) {
    if(port == 0x20 || port == 0x21)
        pic_port_wr8(port, val);
}
void _machine_io_wr16(uint16_t port, uint16_t val) {
}
uint8_t _machine_io_rd8(uint16_t port) {
    if(port == 0x21)
        pic_port_rd8(port);
    return 0;
}
uint16_t _machine_io_rd16(uint16_t port) {
    return 0;
}

void _machine_ips_timer(void* arg) {
    ips = ips_pre;
    ips_pre = 0;
    #ifdef PRINT_IPS
    ESP_LOGI(TAG, "%d IPS", ips);
    #endif
}

// Public functions

void machine_init(void) {
    // allocate main RAM
    main_ram = realloc(main_ram, RAM_SIZE);
    memset(main_ram, 0, RAM_SIZE);

    // start IPS counter timer
    const esp_timer_create_args_t timer_args = {
        .callback = &_machine_ips_timer,
        .name = "ips_counter"
    };
    esp_timer_handle_t timer;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer, 1000000));
}

void machine_loop(void) {
    while(1) {
        cpu_clock();
        pic_clock();
        ips_pre++;
    }
}

uint32_t machine_get_ips() {
    return ips;
}

#ifdef TRACE_MEM_ACCESS
uint8_t machine_mem_rd(uint32_t addr) {
    uint8_t val = _machine_mem_rd(addr);
    ESP_LOGI(TAG, "trace-mem: RD 0x%05x: %02x", addr, val);
    return val;
}

void machine_mem_wr(uint32_t addr, uint8_t val) {
    ESP_LOGI(TAG, "trace-mem: WR 0x%05x: %02x", addr, val);
    _machine_mem_wr(addr, val);
}
#endif

#ifdef TRACE_IO_ACCESS
void machine_io_wr8(uint16_t port, uint8_t val) {
    ESP_LOGI(TAG, "trace-io: WR8 0x%04x: %02x", port, val);
    _machine_io_wr8(port, val);
}
void machine_io_wr16(uint16_t port, uint16_t val) {
    ESP_LOGI(TAG, "trace-io: WR16 0x%04x: %04x", port, val);
    _machine_io_wr16(port, val);
}
uint8_t machine_io_rd8(uint16_t port) {
    uint8_t val = _machine_io_rd8(port);
    ESP_LOGI(TAG, "trace-io: RD8 0x%04x: %02x", port, val);
    return val;
}
uint16_t machine_io_rd16(uint16_t port) {
    uint16_t val = _machine_io_rd8(port);
    ESP_LOGI(TAG, "trace-io: RD16 0x%04x: %04x", port, val);
    return val;
}
#endif