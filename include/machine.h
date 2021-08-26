#pragma once

#include <stdint.h>

// Definitions

#define BIOS_START 0xfe000
#define RAM_SIZE   64 * 1024

#define READ(addr)       machine_mem_rd((addr))
#define WRITE(addr, val) machine_mem_wr((addr), (val))

// Public functions

void    machine_init  (void);
uint8_t machine_mem_rd(uint32_t addr);
void    machine_mem_wr(uint32_t addr, uint8_t val);